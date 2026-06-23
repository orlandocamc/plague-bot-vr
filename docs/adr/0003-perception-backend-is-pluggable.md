# Perception backend is pluggable (Hailo on the robot, CPU in sim)

The original SPEC (§6.3, Binding Constraint #5) fixed AI inference to "YOLOv8n via
NCNN on CPU only — no Hailo, no MindSpore". That constraint no longer matches the
hardware: the physical Mobile Base carries a **Hailo AI accelerator** on its
Raspberry Pi, and a trained, tested Detection model already runs there as a
compiled `.hef`. A trained `best.pt` (from Eliot) also exists for CPU testing.

So `plaguebot_perception`'s `perception_node` selects its inference engine at
runtime via a `backend` parameter rather than hard-coding NCNN:

| backend | Engine | Where it runs | Model |
|---|---|---|---|
| `mock`  | none — returns a synthetic Detection on the nearest plant | sim / CI | — |
| `torch` | Ultralytics PyTorch | dev workstation, sim | `best.pt` |
| `ncnn`  | NCNN (CPU) | dev workstation, sim | `best.pt` exported to NCNN |
| `hailo` | HailoRT | real robot (Raspberry Pi) | `best.hef` |

**Why mock is the sim default.** The greenhouse world's plants are plain foliage
boxes with no tomato/disease texture, so a real model cannot produce a meaningful
Detection in simulation. `mock` lets the Mission state machine
(NAVIGATE→DEPLOY→SCAN→DETECT→IK_POSITION→RETURN) be exercised end-to-end in sim
without a real model. Real Detection is validated on the physical robot with `hailo`.

The service/topic contract (`/perception/detect`, `/perception/detections`) is
identical across backends, so the Mission node is backend-agnostic.

This supersedes SPEC Binding Constraint #5 and the "no Hailo" wording in §6.3.
