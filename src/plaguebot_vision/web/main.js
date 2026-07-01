const videoRealsense = document.getElementById('video-realsense');
const videoKiyo = document.getElementById('video-kiyo');
const statusLog = document.getElementById('status-log');

function log(msg) {
    console.log(msg);
    if (statusLog) {
        statusLog.innerText = `[SYS] ${msg}`;
    }
}

async function startWebRTC() {
    log('Initializing dual-stream WebRTC...');

    const pc = new RTCPeerConnection({ sdpSemantics: 'unified-plan' });

    // Track 0 → RealSense, Track 1 → Razer Kiyo (matches server addTrack order)
    // Each track gets its own MediaStream to avoid both videos sharing the same stream.
    let trackIndex = 0;
    pc.addEventListener('track', (evt) => {
        if (evt.track.kind !== 'video') return;
        const stream = new MediaStream([evt.track]);
        if (trackIndex === 0) {
            videoRealsense.srcObject = stream;
            log('RealSense stream received');
        } else {
            videoKiyo.srcObject = stream;
            log('Razer Kiyo stream received');
        }
        trackIndex++;
    });

    pc.addEventListener('connectionstatechange', () => {
        log(`Connection state: ${pc.connectionState}`);
    });

    // Two recvonly transceivers — one per camera, order matches server tracks
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('video', { direction: 'recvonly' });

    try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        log('Sending offer to server...');
        const response = await fetch('/offer', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                sdp: pc.localDescription.sdp,
                type: pc.localDescription.type,
            }),
        });

        const answer = await response.json();
        await pc.setRemoteDescription(answer);
        log('WebRTC connection established.');
    } catch (e) {
        log(`Error: ${e.message}`);
        console.error(e);
    }
}

window.addEventListener('load', startWebRTC);
