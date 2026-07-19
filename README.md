# StackChan MATLAB Arduino Add-on

Control the official [M5Stack StackChan](https://docs.m5stack.com/ja/StackChan)
(CoreS3 + StackChan body) from MATLAB: body LEDs, head servos, avatar face,
speech balloon, IMU, battery monitor, and camera snapshots.

## Prerequisites

1. **MATLAB R2026b or newer** (developed on R2026b Prerelease)
2. **MATLAB Support Package for Arduino Hardware** — install via
   Add-Ons > Get Hardware Support Packages. During its hardware setup,
   **select ESP32 boards** so the ESP32 Arduino core gets installed.
3. Internet access for the one-time library install.
4. (Only for `faceTracking.m`) Computer Vision Toolbox.

## Install

Copy this whole folder somewhere permanent, then in MATLAB:

```matlab
cd StackChan_lib
installStackChanAddon
```

This installs four Arduino libraries into the support package's own
arduino-cli environment (M5Unified 0.2.13, M5GFX 0.2.19, M5Stack_Avatar 0.10.0,
and StackChan-BSP 1.1.0 from GitHub) and adds this folder to your MATLAB path.
Versions are pinned to a tested combination — safe to re-run anytime.

## Connect

Find your serial port (macOS: `/dev/cu.usbmodem…`, Windows: `COM…`), then:

```matlab
a  = arduino('/dev/cu.usbmodem101', 'ESP32-S3-DevKitC', ...
             'Libraries', 'StackChanFolder/StackChan');
sc = addon(a, 'StackChanFolder/StackChan');
```

The first connection compiles and flashes the firmware — expect a few minutes.

```matlab
showRgbColor(sc, 255, 0, 0);     % body LEDs red (BSP-style names)
startAvatar(sc);                 % show the face
setExpression(sc, 'happy');
setSpeechText(sc, 'Hello!');
sc.Motion.move(45, 30);          % yaw -128..128, pitch 0..90 (deg)
sc.FrameSize = 'qvga';           % capture size: '96x96' ... 'qvga' (320x240)
img = snapshot(sc);              % camera image at the selected size
showImage(sc);                   % show the captured photo on the LCD
startVideo(sc);                  % smooth live view (~20 fps, board-side loop)
stopVideo(sc);                   % freeze on the last frame
showAvatar(sc);                  % switch the LCD back to the face
```

Method names mirror the original C++ APIs (StackChan-BSP `Motion`,
m5stack-avatar, M5Unified) so sketch experience transfers directly.
Full API and naming notes documented in the header comment of
`+arduinoioaddons/+StackChanFolder/StackChan.m`.

## Demos

- `testStackChan.m` — full device test (LEDs, LCD, servos, IMU, battery, avatar, camera)

## Troubleshooting

- **"Cannot program board" after a crash:** the wedged firmware can ignore the
  bootloader reset. Hold the CoreS3 power button ~6 s to power off (USB unplug
  isn't enough — the body battery keeps it alive), press to power on, then
  reconnect with `arduino(..., 'ForceBuildOn', true)`.
- **Add-on not listed by `listArduinoLibraries`:** make sure the
  `+arduinoioaddons` folder was copied intact and its parent folder is on the
  MATLAB path (re-run `installStackChanAddon`).
