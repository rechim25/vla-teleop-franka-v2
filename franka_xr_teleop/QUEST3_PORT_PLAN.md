# Quest 3 Integration Plan (XRoboToolkit -> Franka Teleop)

## Goal
Enable Meta Quest 3 teleoperation input into the existing XRoboToolkit PC-Service + `franka_xr_teleop` bridge pipeline.

Current status:
- Workstation side is functional (`franka_xr_teleop_bridge` connects to PC-Service).
- XRoboToolkit Unity client codebase is Pico-focused and does not run as-is on Quest 3.

## Key Findings from Source Analysis

1. Pico SDK dependency is hard-coded in multiple places:
- `Packages/manifest.json` includes `com.unity.xr.picoxr`.
- Tracking and device APIs use `Unity.XR.PXR` / `PXR_*` (`TrackingData.cs`, `Main.cs`, `UIOperate.cs`).

2. Network/protocol path is reusable:
- `Assets/Scripts/Network/TcpHandler.cs` uses C# sockets and sends framed packets to PC-Service.
- Tracking payload format is wrapped as:
  - command: `PACKET_CCMD_TO_CONTROLLER_FUNCTION` (`0x6D`)
  - JSON body: `{ "functionName": "Tracking", "value": "<tracking-json-string>" }`
- Packet framing lives in `Assets/Scripts/Network/PackageHandle.cs`.

3. Teleop-critical data path:
- `TrackingData.Get()` generates `Head`, `Controller`, `Hand`, `Body`, `Motion`, `timeStampNs`, `Input`.
- For v1 Franka teleop we only need reliable right-controller pose/buttons + timestamp.

## Migration Strategy
Do not change workstation protocol or Franka bridge semantics first.
Port headset-side runtime and tracking providers while preserving the existing PC-Service wire protocol.

## Work Plan

### Phase 1: Quest Runtime Bootstrap (Unity project compiles on Quest)
1. Create branch `quest3-port` inside `third_party/XRoboToolkit-Unity-Client`.
2. Replace Pico XR package usage:
- remove `com.unity.xr.picoxr` from `Packages/manifest.json`.
- keep/add `com.unity.xr.openxr`.
- add Meta XR package (OpenXR + controller profiles).
3. Disable Pico enterprise/camera code paths for teleop MVP:
- gate/remove `PXR_Enterprise` and `PXR_Manager` calls in `Assets/Scripts/Main.cs` and `Assets/Scripts/UI/UIOperate.cs`.

Exit criteria:
- APK builds and launches on Quest 3 without Pico SDK symbols.

### Phase 2: Input Abstraction (replace PXR APIs cleanly)
1. Introduce provider interface:
- `Assets/Scripts/XR/IXrTrackingProvider.cs`
- methods for head pose, controller pose/buttons/trigger, timestamp.
2. Implement Quest provider:
- `Assets/Scripts/XR/QuestTrackingProvider.cs`
- use OpenXR/Unity input APIs for left/right controller states.
3. Refactor `Assets/Scripts/TrackingData.cs`:
- remove direct `PXR_*` calls from teleop path.
- populate existing JSON schema fields from provider output.
- preserve coordinate convention expected by PC-Service consumers.

Exit criteria:
- `TrackingData.Get()` produces stable JSON on Quest 3.

### Phase 3: Preserve PC-Service Protocol Compatibility
1. Keep `TcpHandler.cs` packet framing and command IDs unchanged.
2. Keep `"functionName":"Tracking"` envelope unchanged.
3. Validate handshake sequence remains:
- `PACKET_CCMD_CONNECT` (`0x19`)
- `PACKET_CCMD_SEND_VERSION` (`0x6C`)
- tracking via `0x6D`.

Exit criteria:
- PC-Service accepts Quest client and reports device connected.

### Phase 4: Verify with Franka Bridge (no robot motion first)
1. Run PC-Service.
2. Run bridge dry-run:
- `./franka_xr_teleop_bridge --dry-run`
3. Confirm:
- `server_connected=1`
- `device_connected=1`
- `rx_count` increases
- deadman/clutch/gripper fields change from Quest inputs.

Exit criteria:
- End-to-end Quest -> PC-Service -> bridge command flow is live.

### Phase 5: Safety-Gated Robot Teleop
1. Run bridge with `--no-motion` first.
2. Validate deadman/clutch semantics against expected mapping:
- right A: deadman
- right B: clutch
- right trigger: gripper scalar
3. Enable motion with conservative limits only after stable stream.

Exit criteria:
- Safe bounded teleop with hold on deadman release and stream timeout.

## Initial File Change List (Quest MVP)
- `third_party/XRoboToolkit-Unity-Client/Packages/manifest.json`
- `third_party/XRoboToolkit-Unity-Client/Assets/Scripts/Main.cs`
- `third_party/XRoboToolkit-Unity-Client/Assets/Scripts/UI/UIOperate.cs`
- `third_party/XRoboToolkit-Unity-Client/Assets/Scripts/TrackingData.cs`
- `third_party/XRoboToolkit-Unity-Client/Assets/Scripts/XR/IXrTrackingProvider.cs` (new)
- `third_party/XRoboToolkit-Unity-Client/Assets/Scripts/XR/QuestTrackingProvider.cs` (new)

## Risks
1. Coordinate-frame mismatch between Quest and existing expected tracking frame.
2. Button semantic mismatch (A/B/trigger mapping differences).
3. Hidden Pico-only assumptions in UI/camera subsystems.
4. PC-Service protocol tolerance to missing optional fields (Body/Hand/Motion).

## Validation Checklist
1. Quest app starts and connects to PC-Service.
2. Tracking stream can be toggled from UI.
3. Bridge dry-run shows device connected and active packet flow.
4. Robot does not move unless deadman is asserted.
5. Packet loss/disconnect causes hold behavior.
