# PlagueBot Unity Client

Unity 2022.3.58f1 client for the PlagueBot WebRTC stream, targeting Meta Quest 3.

## Requirements

- Unity 2022.3.58f1
- Package `com.unity.webrtc` 3.0.0-pre.7 or newer
- Tailscale installed on the Quest (sideload via ADB)
- Meta XR SDK

## Setup

1. Create an empty 3D project in Unity.
2. Install the WebRTC package: Window → Package Manager → "+" → Add package by name → `com.unity.webrtc`
3. Copy `Assets/Scripts/PlagueBot/` into your Unity project.
4. In your scene:
   - Create an empty GameObject named `PlagueBotStreamController`
   - Add Component → `PlagueBotWebRTCClient`
   - Set `Bridge Host` to the Tailscale IP of the Raspberry Pi
   - Set `Bridge Port` to `8080`
   - Drag a `RawImage` UI element into `Target Image`

## Player Settings (Android / Quest 3)

- Internet Access: Require
- Allow downloads over HTTP: enabled (required for ws:// cleartext)
- Scripting Backend: IL2CPP
- Target Architectures: ARM64 only
- Minimum API Level: 29+

## Scripts

| Script | Description |
|---|---|
| `PlagueBotWebRTCClient.cs` | Manages RTCPeerConnection, negotiates SDP with Pi, receives video track and renders it on a RawImage |
| `WebSocketClient.cs` | Minimal WebSocket client using System.Net.WebSockets, marshals callbacks to Unity main thread |
