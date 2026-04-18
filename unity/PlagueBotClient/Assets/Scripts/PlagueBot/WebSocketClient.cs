using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// Minimal WebSocket client wrapping System.Net.WebSockets.ClientWebSocket.
/// Callbacks are marshaled back to the Unity main thread.
/// </summary>
public class WebSocketClient
{
    private readonly string url;
    private ClientWebSocket socket;
    private CancellationTokenSource cts;
    private readonly System.Collections.Concurrent.ConcurrentQueue<Action> mainThreadQueue
        = new System.Collections.Concurrent.ConcurrentQueue<Action>();

    public event Action OnOpen;
    public event Action<string> OnMessage;
    public event Action<string> OnError;
    public event Action OnClose;

    public bool IsOpen => socket != null && socket.State == WebSocketState.Open;

    public WebSocketClient(string url)
    {
        this.url = url;
        var pumpGO = new GameObject("WebSocketPump");
        var pump = pumpGO.AddComponent<MainThreadPump>();
        pump.queue = mainThreadQueue;
        GameObject.DontDestroyOnLoad(pumpGO);
    }

    public async void Connect()
    {
        socket = new ClientWebSocket();
        cts = new CancellationTokenSource();
        try { await socket.ConnectAsync(new Uri(url), cts.Token); mainThreadQueue.Enqueue(() => OnOpen?.Invoke()); _ = ReceiveLoop(); }
        catch (Exception e) { mainThreadQueue.Enqueue(() => OnError?.Invoke(e.Message)); }
    }

    private async Task ReceiveLoop()
    {
        var buffer = new byte[64 * 1024];
        var sb = new StringBuilder();
        try
        {
            while (socket.State == WebSocketState.Open)
            {
                WebSocketReceiveResult result;
                sb.Clear();
                do
                {
                    result = await socket.ReceiveAsync(new ArraySegment<byte>(buffer), cts.Token);
                    if (result.MessageType == WebSocketMessageType.Close) { mainThreadQueue.Enqueue(() => OnClose?.Invoke()); return; }
                    sb.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));
                } while (!result.EndOfMessage);
                string msg = sb.ToString();
                mainThreadQueue.Enqueue(() => OnMessage?.Invoke(msg));
            }
        }
        catch (Exception e) { mainThreadQueue.Enqueue(() => OnError?.Invoke(e.Message)); }
    }

    public async void Send(string message)
    {
        if (!IsOpen) return;
        var bytes = Encoding.UTF8.GetBytes(message);
        try { await socket.SendAsync(new ArraySegment<byte>(bytes), WebSocketMessageType.Text, true, cts.Token); }
        catch (Exception e) { mainThreadQueue.Enqueue(() => OnError?.Invoke(e.Message)); }
    }

    public async void Close()
    {
        if (socket == null) return;
        try { cts?.Cancel(); if (socket.State == WebSocketState.Open) await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client closing", CancellationToken.None); }
        catch { }
        finally { socket?.Dispose(); socket = null; }
    }

    private class MainThreadPump : MonoBehaviour
    {
        public System.Collections.Concurrent.ConcurrentQueue<Action> queue;
        void Update() { while (queue != null && queue.TryDequeue(out var action)) { try { action?.Invoke(); } catch (Exception e) { Debug.LogException(e); } } }
    }
}
