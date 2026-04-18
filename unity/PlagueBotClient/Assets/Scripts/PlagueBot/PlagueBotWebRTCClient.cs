using System;
using System.Collections;
using System.Text;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// PlagueBot WebRTC video receiver for Meta Quest 3.
/// Connects to the aiortc bridge on the Raspberry Pi over Tailscale,
/// receives H.264 video and displays it on a RawImage.
/// </summary>
public class PlagueBotWebRTCClient : MonoBehaviour
{
    [Header("Bridge Connection")]
    [Tooltip("Tailscale IP of the Raspberry Pi running the WebRTC bridge")]
    public string bridgeHost = "100.64.29.4";
    [Tooltip("HTTP/WebSocket port of the bridge")]
    public int bridgePort = 8080;

    [Header("Display Target")]
    [Tooltip("RawImage where the incoming video will be rendered")]
    public RawImage targetImage;

    [Header("Status (read-only)")]
    [SerializeField] private string connectionState = "Idle";
    [SerializeField] private string iceState = "Idle";
    [SerializeField] private bool trackReceived = false;

    private RTCPeerConnection pc;
    private WebSocketClient ws;
    private MediaStream receiveStream;
    private Coroutine webRtcUpdateCoroutine;
    private bool cleanedUp = false;

    void Start()
    {
        if (targetImage == null) { Debug.LogError("[PlagueBot] targetImage is not assigned!"); return; }
        webRtcUpdateCoroutine = StartCoroutine(WebRTC.Update());
        Connect();
    }

    void OnDisable() { Cleanup(); }
    void OnDestroy() { Cleanup(); }
    void OnApplicationQuit() { Cleanup(); }

    void Cleanup()
    {
        if (cleanedUp) return;
        cleanedUp = true;
        Debug.Log("[PlagueBot] Cleanup starting");
        try { if (ws != null) { ws.Close(); ws = null; } } catch (Exception e) { Debug.LogWarning($"[PlagueBot] ws close error: {e.Message}"); }
        try
        {
            if (receiveStream != null)
            {
                foreach (var track in receiveStream.GetTracks()) { track.Stop(); track.Dispose(); }
                receiveStream.Dispose(); receiveStream = null;
            }
        } catch (Exception e) { Debug.LogWarning($"[PlagueBot] stream cleanup error: {e.Message}"); }
        try
        {
            if (pc != null)
            {
                pc.OnIceCandidate = null; pc.OnIceConnectionChange = null;
                pc.OnConnectionStateChange = null; pc.OnTrack = null;
                pc.Close(); pc.Dispose(); pc = null;
            }
        } catch (Exception e) { Debug.LogWarning($"[PlagueBot] pc cleanup error: {e.Message}"); }
        if (webRtcUpdateCoroutine != null) { StopCoroutine(webRtcUpdateCoroutine); webRtcUpdateCoroutine = null; }
        if (targetImage != null) targetImage.texture = null;
        Debug.Log("[PlagueBot] Cleanup done");
    }

    public void Connect()
    {
        cleanedUp = false;
        var config = new RTCConfiguration { iceServers = new[] { new RTCIceServer { urls = new[] { "stun:stun.l.google.com:19302" } } } };
        pc = new RTCPeerConnection(ref config);
        receiveStream = new MediaStream();

        pc.OnIceCandidate = candidate =>
        {
            if (candidate == null) return;
            if (ws != null && ws.IsOpen)
            {
                var msg = new SignalingMessage { type = "candidate", candidate = candidate.Candidate, sdpMid = candidate.SdpMid, sdpMLineIndex = candidate.SdpMLineIndex ?? 0 };
                ws.Send(JsonUtility.ToJson(msg));
            }
        };
        pc.OnIceConnectionChange = state => { iceState = state.ToString(); Debug.Log($"[PlagueBot] ICE state: {state}"); };
        pc.OnConnectionStateChange = state => { connectionState = state.ToString(); Debug.Log($"[PlagueBot] PC state: {state}"); };
        pc.OnTrack = e =>
        {
            Debug.Log($"[PlagueBot] Track received: {e.Track.Kind}");
            if (e.Track is VideoStreamTrack video)
            {
                trackReceived = true;
                video.OnVideoReceived += tex => { Debug.Log($"[PlagueBot] Video texture ready: {tex.width}x{tex.height}"); if (targetImage != null) targetImage.texture = tex; };
            }
            receiveStream.AddTrack(e.Track);
        };

        pc.AddTransceiver(TrackKind.Video, new RTCRtpTransceiverInit { direction = RTCRtpTransceiverDirection.RecvOnly });
        string wsUrl = $"ws://{bridgeHost}:{bridgePort}/ws";
        Debug.Log($"[PlagueBot] Connecting to {wsUrl}");
        ws = new WebSocketClient(wsUrl);
        ws.OnOpen += OnWsOpen;
        ws.OnMessage += OnWsMessage;
        ws.OnError += err => Debug.LogError($"[PlagueBot] WS error: {err}");
        ws.OnClose += () => Debug.Log("[PlagueBot] WS closed");
        ws.Connect();
    }

    async void OnWsOpen()
    {
        if (pc == null) return;
        var op = pc.CreateOffer();
        while (!op.IsDone) await System.Threading.Tasks.Task.Yield();
        if (op.IsError) { Debug.LogError($"[PlagueBot] CreateOffer error: {op.Error.message}"); return; }
        var desc = op.Desc;
        var setLocal = pc.SetLocalDescription(ref desc);
        while (!setLocal.IsDone) await System.Threading.Tasks.Task.Yield();
        if (setLocal.IsError) { Debug.LogError($"[PlagueBot] SetLocal error: {setLocal.Error.message}"); return; }
        ws.Send(JsonUtility.ToJson(new SignalingMessage { type = "offer", sdp = desc.sdp }));
        Debug.Log("[PlagueBot] Offer sent");
    }

    async void OnWsMessage(string data)
    {
        if (pc == null) return;
        SignalingMessage msg;
        try { msg = JsonUtility.FromJson<SignalingMessage>(data); } catch (Exception e) { Debug.LogError($"[PlagueBot] bad JSON: {e.Message}"); return; }
        if (msg.type == "answer")
        {
            var desc = new RTCSessionDescription { type = RTCSdpType.Answer, sdp = msg.sdp };
            var op = pc.SetRemoteDescription(ref desc);
            while (!op.IsDone) await System.Threading.Tasks.Task.Yield();
            if (op.IsError) Debug.LogError($"[PlagueBot] SetRemote error: {op.Error.message}");
        }
        else if (msg.type == "candidate" && !string.IsNullOrEmpty(msg.candidate))
        {
            pc.AddIceCandidate(new RTCIceCandidate(new RTCIceCandidateInit { candidate = msg.candidate, sdpMid = msg.sdpMid, sdpMLineIndex = msg.sdpMLineIndex }));
        }
    }

    [Serializable]
    private class SignalingMessage { public string type; public string sdp; public string candidate; public string sdpMid; public int sdpMLineIndex; }
}
