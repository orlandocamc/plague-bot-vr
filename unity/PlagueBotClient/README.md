# PlagueBot Unity Client

Unity 2022.3.58f1 client for the PlagueBot WebRTC stream, targeting Meta Quest 3.

## Requirements

- Unity 2022.3.58f1 (or any 2022 LTS with same package support)
- Package `com.unity.webrtc` 3.0.0-pre.7 or newer
- Tailscale installed on the Quest (sideload via ADB)
- Meta XR SDK (for the VR mockup UI)

## Setup

1. Create an empty 3D project in Unity.
2. Install the WebRTC package:
   - Window → Package Manager
   - "+" → Add package by name
   - Name: `com.unity.webrtc`
   - Version: `3.0.0-pre.7`
3. Copy `Assets/Scripts/PlagueBot/` from this repo into your Unity project under `Assets/Scripts/`.
4. In your scene:
   - Create an empty GameObject named `PlagueBotStreamController`.
   - Add Component → `PlagueBotWebRTCClient`.
   - Set `Bridge Host` to the Tailscale IP of the Raspberry Pi.
   - Set `Bridge Port` to `8080`.
   - Drag a `RawImage` UI element into `Target Image` — that's where the video renders.

## Player Settings (Android / Quest)

Edit → Project Settings → Player → Android tab → Other Settings:
- Internet Access: **Require**
- Allow downloads over HTTP: **enabled** (required for `ws://` cleartext)
- Scripting Backend: **IL2CPP**
- Target Architectures: **ARM64** only
- Minimum API Level: **29+**
- Target API Level: **33+**

## Testing

1. Start the Pi bridge (via systemd service or `ros2 launch plaguebot_vision plaguebot.launch.py`).
2. Confirm `http://<pi-tailscale-ip>:8080` shows video in a browser.
3. In Unity: press Play. After 2-3 seconds the Console should log:
   - `[PlagueBot] Connecting to ws://...`
   - `[PlagueBot] PC state: Connected`
   - `[PlagueBot] Video texture ready: 640x480`
4. The RawImage now displays live video.

## Stopping Play in the Editor (IMPORTANT)

`com.unity.webrtc` on Linux can hang the Editor on Stop. Before hitting Stop:
1. In the Inspector, uncheck the enable checkbox of `PlagueBotWebRTCClient` (triggers `OnDisable` → `Cleanup`).
2. Wait 2 seconds for `[PlagueBot] Cleanup done` in Console.
3. Then press Stop.

## Deploy to Quest 3

1. Quest 3 must have Tailscale installed and connected.
2. File → Build Settings → Android → Build And Run with Quest connected via USB.
3. Open the app in the Quest. Video should appear within ~5 seconds.

## Signaling Protocol

The client talks to the Pi bridge via WebSocket at `ws://<host>:<port>/ws` using JSON messages:

```json
{ "type": "offer", "sdp": "..." }
{ "type": "answer", "sdp": "..." }
{ "type": "candidate", "candidate": "candidate:...", "sdpMid": "0", "sdpMLineIndex": 0 }
```

## Scripts

| Script | Description |
|---|---|
| `PlagueBotWebRTCClient.cs` | Manages RTCPeerConnection, negotiates SDP with Pi, receives video track and renders it on a RawImage |
| `WebSocketClient.cs` | Minimal WebSocket client using System.Net.WebSockets, marshals callbacks to Unity main thread |
