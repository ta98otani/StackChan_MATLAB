// StackChan.h
// StackChan add-on: MATLAB Arduino server extension for the official
// M5Stack StackChan robot (CoreS3 + StackChan body).
//
// All device access goes through the original vendor libraries:
//   StackChan-BSP  — body LEDs (PY32L020 IO-expander @ I2C 0x6F), serial-bus
//                    servos (Motion, 0.1-degree units), INA226 battery monitor
//   M5Unified      — PMU, LCD, IMU; uses its own internal I2C (G11/G12),
//                    so MATLAB's I2C/Wire bus is never touched
//   m5stack-avatar — the original StackChan face by meganetaaan
#include "LibraryBase.h"
#include <M5Unified.h>
#include <M5StackChan.h>
#include <Avatar.h>
#include "esp_camera.h"  // esp32-camera, bundled with the arduino-esp32 core
#include "esp_log.h"
#include "esp_timer.h"   // (kept for future timing diagnostics)
#include "esp_system.h"  // esp_reset_reason for SC_SYS_INFO diagnostics

// Command IDs — must match the constants in StackChan.m
#define SC_LED_WRITE_ALL   0x02  // [r, g, b] -> all 12 LEDs
#define SC_LED_WRITE_PIXEL 0x03  // [index0, r, g, b]
#define SC_LED_CLEAR       0x04  // [dummy]
#define SC_TEST_READ       0x0F  // returns greeting string
#define SC_M5_BEGIN        0x10  // [dummy]; init robot body (lazy init)
#define SC_LCD_FILL        0x11  // [r, g, b]; fill LCD color
#define SC_SYS_INFO        0x13  // -> [resetReason u8, freeHeap u32, maxAlloc u32]
#define SC_SERVO_MOVE      0x20  // [yaw int16, pitch int16, speed uint16] 0.1deg
#define SC_SERVO_HOME      0x21  // [speed uint16]
#define SC_SERVO_GET       0x22  // -> [yaw int16, pitch int16] 0.1deg feedback
#define SC_SERVO_ROTATE    0x23  // [velocity int16] continuous yaw
#define SC_SERVO_STOP      0x24  // [dummy]
#define SC_IMU_READ        0x30  // -> 9 float32: accel xyz, gyro xyz, mag xyz
#define SC_BATT_READ       0x31  // -> 2 float32: voltage V, current mA
#define SC_CAM_INIT        0x40  // [framesize_t+1, 0=default] -> status: 1 PSRAM, 2 DRAM, 0 fail; re-inits if size differs
#define SC_CAM_CAPTURE     0x41  // [mode, gray] -> [w uint16, h uint16, len uint32]
                                 // gray=1: frame converted to 8-bit luma,
                                 // len = w*h (else RGB565, len = w*h*2)
#define SC_CAM_READ        0x42  // [offset uint32, count uint16] -> frame bytes
#define SC_CAM_SHOW        0x43  // [freshCapture] draw held frame on LCD (stops avatar)
#define SC_VIDEO_START     0x44  // [dummy] board-side capture+draw loop (stops avatar)
#define SC_VIDEO_STOP      0x45  // [dummy] -> [frames uint32] shown since start
#define SC_ANNOT_SET       0x46  // [count, per annot: x,y,w,h u16, r,g,b, len, label]
                                 // camera-pixel coords, 0-based; count 0 clears

#define SC_MAX_ANNOTS       8
#define SC_ANNOT_LABEL_MAX  23
#define SC_AVATAR_START    0x50  // [dummy]
#define SC_AVATAR_STOP     0x51  // [dummy]
#define SC_AVATAR_EXPR     0x52  // [expression 0-5], m5avatar::Expression order
#define SC_AVATAR_SPEECH   0x53  // [len, chars...]; len 0 clears balloon
#define SC_AVATAR_MOUTH    0x54  // [ratio 0-100]
#define SC_AVATAR_GAZE     0x55  // [h int8 -100..100, v int8]

// Trace messages, printed when the arduino object is created with 'Trace',true
const char MSG_SC_BEGIN[] PROGMEM = "StackChan::begin() done\n";
const char MSG_SC_MOVE[]  PROGMEM = "StackChan::moveHead(yaw=%d, pitch=%d, speed=%d)\n";
const char MSG_SC_CAM1[]  PROGMEM = "cam: enter, freeHeap=%u largest=%u\n";
const char MSG_SC_CAM2[]  PROGMEM = "cam: I2C released\n";
const char MSG_SC_CAM3[]  PROGMEM = "cam: esp_camera_init -> %d\n";
const char MSG_SC_CAM4[]  PROGMEM = "cam: done, freeHeap=%u\n";

class StackChan : public LibraryBase {
public:
    bool m5Started;
    m5avatar::Avatar* avatar;
    bool avatarRunning;
    char speechBuf[64];  // avatar keeps the pointer; text must stay alive
    bool camReady;
    byte camStatus;
    camera_fb_t* camFb;  // frame held on-board between capture and reads
    uint32_t camLen;     // bytes SC_CAM_READ may stream from camFb (differs
                         // from camFb->len after grayscale conversion)
    framesize_t camFrameSize;
    bool imageOnScreen;  // last thing drawn was a camera frame (skip clears)
    bool videoMode;      // loop() captures + draws continuously
    uint32_t videoFrames;

    // Number of DMA frame buffers the camera was initialized with. With 2
    // (small sizes) the driver captures continuously into the spare buffer
    // and recycles the queued frame (GRAB_LATEST), so esp_camera_fb_get
    // returns a just-completed frame almost instantly — grabFreshFrame
    // exploits that with a low-latency get-new-then-return-old swap.
    int camFbCount;

    // Display copy of the last captured frame, so showImage/annotations can
    // redraw the EXACT frame MATLAB received even after the in-place
    // grayscale conversion trashed the fb, or the buffer was recycled.
    // Allocated at camera init; if DRAM is too tight (large FrameSize) it
    // stays NULL and grayscale mode falls back to color.
    uint8_t* dispBuf;
    uint32_t dispLen;
    uint16_t dispW, dispH;
    bool dispValid;

    // Overlay annotations (insertObjectAnnotation-style), drawn on top of
    // every camera frame put on the LCD until replaced or cleared
    struct ScAnnot {
        uint16_t x, y, w, h;   // camera-pixel coords, 0-based
        uint8_t r, g, b;
        char label[SC_ANNOT_LABEL_MAX + 1];
    };
    ScAnnot annots[SC_MAX_ANNOTS];
    byte annotCount;
    uint8_t bootResetReason;   // esp_reset_reason() sampled at boot

    // Initializer list, not in-class init (safer on ESP32/ARM toolchains)
    StackChan(MWArduinoClass& a) : m5Started(false), avatar(NULL), avatarRunning(false),
                                   camReady(false), camStatus(0), camFb(NULL),
                                   camLen(0), camFrameSize(FRAMESIZE_QQVGA),
                                   imageOnScreen(false), videoMode(false), videoFrames(0),
                                   camFbCount(1),
                                   dispBuf(NULL), dispLen(0), dispW(0), dispH(0),
                                   dispValid(false),
                                   annotCount(0), bootResetReason(0xFF) {
        libName = "StackChanFolder/StackChan";
        a.registerLibrary(this);
        speechBuf[0] = '\0';
    }

    // Must stay (nearly) empty: hardware init here breaks the MATLAB
    // server handshake. Recording the reset reason is safe and lets
    // SC_SYS_INFO tell a panic reboot from a normal power-up.
    void setup() {
        bootResetReason = (uint8_t)esp_reset_reason();
    }

    // Called by the Arduino server between serial commands. In video mode
    // the board streams by itself: return the drawn frame, grab the next,
    // draw. Return-BEFORE-get keeps both DMA buffers rolling, which is
    // what sustains the full sensor rate (~20 fps at QQVGA); since this
    // loop consumes as fast as the sensor produces, the queued frame it
    // gets is at most ~1-2 frame periods old — smoothness matters more
    // than absolute freshness for a live view. (Captures for MATLAB use
    // grabFreshFrame's get-then-return instead: freshest possible frame.)
    void loop() {
        if (videoMode && camReady) {
            if (camFb != NULL) {
                esp_camera_fb_return(camFb);
                camFb = NULL;
            }
            camFb = esp_camera_fb_get();
            if (camFb != NULL) {
                drawCamFrame();
                videoFrames++;
            }
        }
    }

    // Swap the held frame for the next one: return-then-get, the standard
    // driver pattern, and the ONLY place capture-path frames are grabbed.
    // The held frame is kept between captures on purpose: the driver then
    // rolls its one remaining buffer (fb_count=2, GRAB_LATEST recycles
    // the queued frame every VSYNC), so the frame this returns is the
    // latest complete one (~1 frame period old) and the get is ~instant.
    // Fancier schemes tried here — get-before-return, timestamp-based
    // double-gets, a background prefetch task — either served stale
    // frames (visible display lag) or hard-hung the board within a few
    // hundred cycles (suspected ISR race in the driver's buffer
    // recycling under rapid return/get churn). Keep this boring.
    bool grabFreshFrame() {
        if (camFb != NULL) {
            esp_camera_fb_return(camFb);
            camFb = NULL;
        }
        camFb = esp_camera_fb_get();
        return camFb != NULL;
    }

    // Draw the held camera frame scaled to fill the LCD. The screen is
    // cleared only on the first frame after something else was on it —
    // clearing every frame is what made the live view flicker black.
    // Camera DMA data is big-endian RGB565 (the panel's native order), so
    // byte swapping stays off, as in the official CoreS3 camera example.
    void drawFrameBuf(uint8_t* buf, int w, int h) {
        float zoomX = (float)M5.Display.width() / w;
        float zoomY = (float)M5.Display.height() / h;
        float zoom = (zoomX < zoomY) ? zoomX : zoomY;
        int dx = (M5.Display.width()  - (int)(w * zoom)) / 2;
        int dy = (M5.Display.height() - (int)(h * zoom)) / 2;
        M5.Display.setSwapBytes(false);
        M5.Display.startWrite();
        if (!imageOnScreen) {
            M5.Display.fillScreen(TFT_BLACK);
        }
        if (zoom == 1.0f) {   // exact fit (e.g. QVGA): plain centered push
            M5.Display.pushImage(dx, dy, w, h, (uint16_t*)buf);
        } else {
            M5.Display.pushImageRotateZoom(
                M5.Display.width() / 2.0f, M5.Display.height() / 2.0f,
                w / 2.0f, h / 2.0f,
                0.0f, zoom, zoom,
                w, h, (uint16_t*)buf);
        }
        drawAnnots(zoom, dx, dy);
        M5.Display.endWrite();
        imageOnScreen = true;
    }

    // Live-view path: draw the frame buffer currently held from the driver
    void drawCamFrame() {
        if (camFb != NULL) {
            drawFrameBuf(camFb->buf, camFb->width, camFb->height);
        }
    }

    // Still-image path: prefer the display copy (the exact frame MATLAB
    // received — its fb may already be back with the driver), else the
    // held frame. Returns false when there is nothing to draw.
    bool drawStillFrame() {
        if (dispValid && dispBuf != NULL) {
            drawFrameBuf(dispBuf, dispW, dispH);
            return true;
        }
        if (camFb != NULL) {
            drawCamFrame();
            return true;
        }
        return false;
    }

    // Cooperatively halt the avatar render/facial tasks at a frame
    // boundary. NEVER use Avatar::suspend() while the speech balloon can
    // be on screen: vTaskSuspend freezes the task at a random instruction
    // (mid heap op / display transaction) and intermittently kills or
    // wedges the board. Avatar::stop() lets the tasks exit cleanly; the
    // delay covers one worst-case frame. Restart with avatar->start()
    // (never resume(): the old task handles are gone after stop()).
    // Note: each start() leaks one small DriveContext (library limitation).
    void avatarHalt(unsigned long settleMs) {
        avatar->stop();
        delay(settleMs);
    }

    // Little-endian helpers for multi-byte payload fields from MATLAB
    static int16_t readInt16(byte* p) {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }
    static uint16_t readUint16(byte* p) {
        return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    }
    static uint32_t readUint32(byte* p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }

    // Draw the stored annotations over the camera image on the LCD,
    // insertObjectAnnotation-style: 2-px rectangle plus a filled label tag
    // above it (inside it when there is no room above). Coordinates are
    // camera pixels, mapped through the same zoom/offset as the image.
    void drawAnnots(float zoom, int dx, int dy) {
        if (annotCount == 0) {
            return;
        }
        M5.Display.setFont(&fonts::Font2);
        M5.Display.setTextColor(TFT_BLACK);
        M5.Display.setTextDatum(top_left);
        for (byte i = 0; i < annotCount; i++) {
            ScAnnot& an = annots[i];
            int x = dx + (int)(an.x * zoom);
            int y = dy + (int)(an.y * zoom);
            int w = (int)(an.w * zoom);
            int h = (int)(an.h * zoom);
            uint32_t c = M5.Display.color888(an.r, an.g, an.b);
            M5.Display.drawRect(x, y, w, h, c);
            if (w > 4 && h > 4) {
                M5.Display.drawRect(x + 1, y + 1, w - 2, h - 2, c);
            }
            if (an.label[0] != '\0') {
                int tw = M5.Display.textWidth(an.label);
                int th = M5.Display.fontHeight();
                int ly = y - th - 2;
                if (ly < dy) {
                    ly = y + 2;   // no room above the box: put it inside
                }
                M5.Display.fillRect(x, ly, tw + 4, th + 2, c);
                M5.Display.drawString(an.label, x + 2, ly + 1);
            }
        }
    }

    // Camera bring-up, pin map and In_I2C.release() taken verbatim from the
    // original M5CoreS3 GC0308 driver (the SCCB control bus shares the
    // internal I2C with the PMU/touch, so M5Unified must release it first)
    bool cameraInit(framesize_t frameSize) {
        camera_config_t config = {};
        config.pin_pwdn = -1;
        config.pin_reset = -1;
        config.pin_xclk = -1;       // GC0308 on CoreS3 is self-clocked
        config.pin_sccb_sda = 12;   // shared internal I2C
        config.pin_sccb_scl = 11;
        config.pin_d7 = 47;
        config.pin_d6 = 48;
        config.pin_d5 = 16;
        config.pin_d4 = 15;
        config.pin_d3 = 42;
        config.pin_d2 = 41;
        config.pin_d1 = 40;
        config.pin_d0 = 39;
        config.pin_vsync = 46;
        config.pin_href = 38;
        config.pin_pclk = 45;
        config.xclk_freq_hz = 20000000;
        config.ledc_timer = LEDC_TIMER_0;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.pixel_format = PIXFORMAT_RGB565;
        config.jpeg_quality = 0;
        config.grab_mode = CAMERA_GRAB_LATEST;  // snapshots must be fresh
        config.sccb_i2c_port = -1;
        config.frame_size = frameSize;
        if (psramFound()) {
            config.fb_location = CAMERA_FB_IN_PSRAM;
            config.fb_count = 2;
        } else {
            // No PSRAM on MATLAB's ESP32-S3 build: the RGB565 frame must fit
            // in a contiguous DRAM block (QQVGA 38 KB ... QVGA 154 KB).
            // Small frames get a second buffer: the sensor then captures
            // continuously into the spare one and fb_get returns a complete
            // frame almost instantly, instead of waiting up to two frame
            // periods (VSYNC alignment + full DMA) after every fb_return.
            config.fb_location = CAMERA_FB_IN_DRAM;
            config.fb_count = (frameSize <= FRAMESIZE_128X128) ? 2 : 1;
        }
        camFbCount = config.fb_count;   // drives grabFreshFrame's strategy
        debugPrint(MSG_SC_CAM1, (unsigned)ESP.getFreeHeap(),
                   (unsigned)ESP.getMaxAllocHeap());
        M5.In_I2C.release();
        debugPrint(MSG_SC_CAM2);
        esp_err_t err = esp_camera_init(&config);
        debugPrint(MSG_SC_CAM3, (int)err);
        if (err == ESP_OK) {
            // The GC0308 powers up with horizontal mirroring enabled
            // (selfie-style), so captures read left-right reversed.
            // Disable it: snapshots then match the scene (text the
            // right way round). Runs on every (re)init.
            sensor_t* s = esp_camera_sensor_get();
            if (s != NULL && s->set_hmirror != NULL) {
                s->set_hmirror(s, 0);
            }
        }
        debugPrint(MSG_SC_CAM4, (unsigned)ESP.getFreeHeap());
        return err == ESP_OK;
    }

    void commandHandler(byte cmdID, byte* dataIn, unsigned int payloadSize) {
        switch (cmdID) {
            case SC_M5_BEGIN: {
                // Init M5Unified first with serial logging disabled (MATLAB
                // owns the USB-CDC serial). M5StackChan.begin() calls
                // M5.begin() again internally, but M5Unified guards against
                // double init, so our safe config wins.
                if (!m5Started) {
                    auto cfg = M5.config();
                    cfg.serial_baudrate = 0;   // never touch MATLAB's serial
                    cfg.output_power = true;
                    cfg.internal_imu = true;   // BMI270 + BMM150
                    cfg.internal_mic = false;
                    cfg.internal_spk = false;
                    M5.begin(cfg);
                    // ESP-IDF error logs (e.g. the camera driver's "Failed to
                    // get the frame on time!") would go out the USB-CDC port
                    // and corrupt the MATLAB server protocol stream
                    esp_log_level_set("*", ESP_LOG_NONE);
                    // Full robot body init: touch sensor, PY32 IO-expander
                    // (LEDs + servo power), serial-bus servos, battery monitor
                    M5StackChan.begin();
                    m5Started = true;
                    debugPrint(MSG_SC_BEGIN);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }

            // ------------------------- LEDs / LCD -------------------------
            case SC_LED_WRITE_ALL: {
                if (m5Started) {
                    M5StackChan.showRgbColor(dataIn[0], dataIn[1], dataIn[2]);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_LED_WRITE_PIXEL: {
                if (m5Started) {
                    M5StackChan.setRgbColor(dataIn[0], dataIn[1], dataIn[2], dataIn[3]);
                    M5StackChan.refreshRgb();
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_LED_CLEAR: {
                if (m5Started) {
                    M5StackChan.showRgbColor(0, 0, 0);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_LCD_FILL: {
                if (m5Started) {
                    // A solid fill means "take over the screen", so stop
                    // whoever else is drawing first. The avatar renders
                    // from its own task, and concurrent M5GFX access
                    // from two tasks HANGS the board (found the hard
                    // way: fillScreen while the face was up).
                    videoMode = false;
                    if (avatarRunning && avatar != NULL) {
                        avatarHalt(150);
                        avatarRunning = false;
                    }
                    imageOnScreen = false;
                    M5.Display.fillScreen(
                        M5.Display.color888(dataIn[0], dataIn[1], dataIn[2]));
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }

            // ------------------------ Head servos ------------------------
            case SC_SERVO_MOVE: {
                int16_t yaw = readInt16(dataIn);
                int16_t pitch = readInt16(dataIn + 2);
                uint16_t speed = readUint16(dataIn + 4);
                if (m5Started) {
                    M5StackChan.Motion.move(yaw, pitch, speed);
                }
                debugPrint(MSG_SC_MOVE, yaw, pitch, speed);
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_SERVO_HOME: {
                if (m5Started) {
                    M5StackChan.Motion.goHome(readUint16(dataIn));
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_SERVO_GET: {
                // Feedback servo positions in 0.1-degree units
                int16_t out[2] = {0, 0};
                if (m5Started) {
                    out[0] = (int16_t)M5StackChan.Motion.getCurrentYawAngle();
                    out[1] = (int16_t)M5StackChan.Motion.getCurrentPitchAngle();
                }
                sendResponseMsg(cmdID, (byte*)out, sizeof(out));
                break;
            }
            case SC_SERVO_ROTATE: {
                if (m5Started) {
                    M5StackChan.Motion.rotateYaw(readInt16(dataIn));
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_SERVO_STOP: {
                if (m5Started) {
                    M5StackChan.Motion.stop();
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }

            // -------------------------- Sensors --------------------------
            case SC_IMU_READ: {
                float vals[9] = {0};
                if (m5Started) {
                    M5.Imu.update();
                    M5.Imu.getAccel(&vals[0], &vals[1], &vals[2]);
                    M5.Imu.getGyro(&vals[3], &vals[4], &vals[5]);
                    M5.Imu.getMag(&vals[6], &vals[7], &vals[8]);
                }
                sendResponseMsg(cmdID, (byte*)vals, sizeof(vals));
                break;
            }
            case SC_BATT_READ: {
                float vals[2] = {0};
                if (m5Started) {
                    vals[0] = M5StackChan.getBatteryVoltage();
                    vals[1] = M5StackChan.getBatteryCurrent();
                }
                sendResponseMsg(cmdID, (byte*)vals, sizeof(vals));
                break;
            }

            // --------------------------- Camera ---------------------------
            case SC_CAM_INIT: {
                // dataIn[0]: requested size as framesize_t + 1 (0 = default:
                // QVGA with PSRAM, QQVGA without). A different size on an
                // already-running camera triggers a full deinit + re-init.
                byte status = 0;
                if (m5Started) {
                    framesize_t fs = psramFound() ? FRAMESIZE_QVGA
                                                  : FRAMESIZE_QQVGA;
                    if (payloadSize >= 1 && dataIn[0] != 0) {
                        fs = (framesize_t)(dataIn[0] - 1);
                    }
                    if (camReady && fs == camFrameSize) {
                        status = camStatus;
                    } else {
                        // Camera bring-up does large DMA allocations; the
                        // avatar render task running concurrently crashes the
                        // board, so halt it around esp_camera_init.
                        videoMode = false;
                        bool resumeAvatar = avatarRunning;
                        if (resumeAvatar) {
                            avatarHalt(100);
                        }
                        if (camReady) {   // size change: tear down first
                            if (camFb != NULL) {
                                esp_camera_fb_return(camFb);
                                camFb = NULL;
                            }
                            esp_camera_deinit();
                            camReady = false;
                        }
                        if (dispBuf != NULL) {   // sized for the old frames
                            free(dispBuf);
                            dispBuf = NULL;
                        }
                        dispValid = false;
                        if (cameraInit(fs)) {
                            camReady = true;
                            camFrameSize = fs;
                            camStatus = psramFound() ? 1 : 2;
                            status = camStatus;
                            // Grab and hold a frame right away: it seeds
                            // grabFreshFrame's swap, and with fb_count=1 it
                            // keeps the camera DMA quiescent while the
                            // avatar renders. (With fb_count=2 the sensor
                            // captures continuously by design — soak-tested
                            // stable alongside the avatar; the old "DMA
                            // kills the avatar" crash was really the
                            // speechFont bug, see SC_AVATAR_START.)
                            camFb = esp_camera_fb_get();
                            // Display copy enables snapshot pipelining (the
                            // fb can go back to the driver right after the
                            // transfer). If DRAM is too tight for a second
                            // frame, snapshots just stay synchronous.
                            if (camFb != NULL) {
                                dispLen = camFb->len;
                                dispBuf = (uint8_t*)malloc(dispLen);
                            }
                            // Frame dimensions changed: clear the screen on
                            // the next draw so old borders don't linger,
                            // and drop annotations (their coords are in the
                            // old frame's pixel scale)
                            imageOnScreen = false;
                            annotCount = 0;
                        }
                        if (resumeAvatar) {
                            avatar->start();
                        }
                    }
                }
                sendResponseMsg(cmdID, &status, 1);
                break;
            }
            case SC_CAM_CAPTURE: {
                // Grab the freshest frame and hold it for the chunked
                // reads (~instant with two DMA buffers, see grabFreshFrame).
                // dataIn[0]: 2 = leave the avatar running during the grab;
                // anything else = cooperatively halt it at a frame
                // boundary (raw suspend variants proved unsafe — avatarHalt).
                // dataIn[1]: 1 = convert the frame to 8-bit luma before the
                // reads (half the serial bytes; the LCD keeps full color
                // via the display copy).
                byte mode = (payloadSize >= 1) ? dataIn[0] : 3;
                byte gray = (payloadSize >= 2) ? dataIn[1] : 0;
                uint16_t wh[2] = {0, 0};
                uint32_t len = 0;
                if (camReady) {
                    // Streaming must not swap camFb while the chunked
                    // SC_CAM_READ transfers of this frame are in flight
                    videoMode = false;
                    bool pauseAvatar = avatarRunning && avatar != NULL && mode != 2;
                    if (pauseAvatar) {
                        avatarHalt(150);
                    }
                    grabFreshFrame();
                    if (pauseAvatar) {
                        avatar->start();
                    }
                    if (camFb != NULL) {
                        wh[0] = camFb->width;
                        wh[1] = camFb->height;
                        len = camFb->len;
                        // Keep the display copy in sync with the frame
                        // MATLAB is about to receive (~1 ms memcpy)
                        bool haveCopy = dispBuf != NULL && camFb->len <= dispLen;
                        if (haveCopy) {
                            memcpy(dispBuf, camFb->buf, camFb->len);
                            dispW = camFb->width;
                            dispH = camFb->height;
                            dispValid = true;
                        }
                        if (gray && haveCopy) {
                            // In-place big-endian RGB565 -> 8-bit luma.
                            // Trashing the fb is fine: the driver overwrites
                            // it with the next DMA frame, and the display
                            // copy above keeps the color for the LCD.
                            // Forward pass is safe: writes b[i], reads
                            // b[2i], b[2i+1] with 2i >= i.
                            uint8_t* b = camFb->buf;
                            uint32_t n = (uint32_t)camFb->width * camFb->height;
                            for (uint32_t i = 0; i < n; i++) {
                                uint16_t v = ((uint16_t)b[2*i] << 8) | b[2*i + 1];
                                uint16_t r = (v >> 11) << 3;
                                uint16_t g = ((v >> 5) & 0x3F) << 2;
                                uint16_t bl = (v & 0x1F) << 3;
                                b[i] = (uint8_t)((77u*r + 150u*g + 29u*bl) >> 8);
                            }
                            len = n;
                        }
                        camLen = len;
                    }
                }
                byte out[8];
                memcpy(out, wh, 4);
                memcpy(out + 4, &len, 4);
                sendResponseMsg(cmdID, out, sizeof(out));
                break;
            }
            case SC_CAM_READ: {
                // Stream part of the held frame: [offset uint32, count uint16]
                uint32_t offset = readUint32(dataIn);
                uint16_t count = readUint16(dataIn + 4);
                if (camFb != NULL && camLen != 0 && offset + count <= camLen) {
                    sendResponseMsg(cmdID, camFb->buf + offset, count);
                } else {
                    byte err = 0;
                    sendResponseMsg(cmdID, &err, 1);
                }
                break;
            }

            case SC_CAM_SHOW: {
                // Draw the held camera frame full-screen on the LCD. The
                // image takes over the screen, so the avatar is stopped
                // for good; SC_AVATAR_START (showAvatar) brings it back.
                // dataIn[0]: 1 = grab a fresh frame first, 0 = show as held.
                byte ok = 0;
                if (camReady) {
                    videoMode = false;   // explicit draw overrides streaming
                    if (avatarRunning && avatar != NULL) {
                        avatarHalt(150);
                        avatarRunning = false;
                    }
                    if (payloadSize >= 1 && dataIn[0] != 0) {
                        if (grabFreshFrame() && dispBuf != NULL &&
                                camFb->len <= dispLen) {
                            // Keep the display copy current so annotation
                            // refreshes redraw this same frame
                            memcpy(dispBuf, camFb->buf, camFb->len);
                            dispW = camFb->width;
                            dispH = camFb->height;
                            dispValid = true;
                        }
                    }
                    if (drawStillFrame()) {
                        ok = 1;
                    }
                }
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_VIDEO_START: {
                // Hand the LCD to the board-side streaming loop (see loop()).
                byte ok = 0;
                if (camReady) {
                    if (avatarRunning && avatar != NULL) {
                        avatarHalt(150);
                        avatarRunning = false;
                    }
                    videoFrames = 0;
                    videoMode = true;
                    ok = 1;
                }
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_ANNOT_SET: {
                // Replace-all semantics, like one insertObjectAnnotation
                // call: [count, then per annotation x,y,w,h uint16 LE,
                // r,g,b, labelLen, label chars]. count 0 clears.
                byte n = (payloadSize >= 1) ? dataIn[0] : 0;
                if (n > SC_MAX_ANNOTS) {
                    n = SC_MAX_ANNOTS;
                }
                byte* p = dataIn + 1;
                byte* end = dataIn + payloadSize;
                byte stored = 0;
                for (byte i = 0; i < n; i++) {
                    if (p + 12 > end) {
                        break;
                    }
                    ScAnnot& an = annots[stored];
                    an.x = readUint16(p);
                    an.y = readUint16(p + 2);
                    an.w = readUint16(p + 4);
                    an.h = readUint16(p + 6);
                    an.r = p[8];
                    an.g = p[9];
                    an.b = p[10];
                    byte len = p[11];
                    p += 12;
                    if (p + len > end) {
                        break;
                    }
                    byte cl = (len > SC_ANNOT_LABEL_MAX) ? SC_ANNOT_LABEL_MAX
                                                         : len;
                    memcpy(an.label, p, cl);
                    an.label[cl] = '\0';
                    p += len;
                    stored++;
                }
                annotCount = stored;
                // Refresh a static image right away so the overlay updates
                // without another showImage; streamed frames pick it up on
                // their own in loop()
                if (imageOnScreen && !videoMode) {
                    drawStillFrame();
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_VIDEO_STOP: {
                // Stop streaming; the last frame stays on the LCD. Replies
                // with the number of frames shown since SC_VIDEO_START.
                videoMode = false;
                sendResponseMsg(cmdID, (byte*)&videoFrames, sizeof(videoFrames));
                break;
            }

            // -------------------- Avatar (the face) ----------------------
            case SC_AVATAR_START: {
                videoMode = false;      // face takes the screen back
                imageOnScreen = false;
                if (m5Started && !avatarRunning) {
                    if (avatar == NULL) {
                        avatar = new m5avatar::Avatar();
                        // m5stack-avatar 0.10.0 never initializes its
                        // speechFont member: drawing the speech balloon
                        // dereferences a garbage pointer and panics the
                        // board (heap-content dependent, so it only shows
                        // up in some call orders). Always set a real font.
                        avatar->setSpeechFont(&fonts::Font2);
                        avatar->init();   // starts the drawing tasks
                    } else {
                        // Tasks were cooperatively stopped; the old handles
                        // are gone, so resume() would be use-after-delete
                        avatar->start();
                    }
                    avatarRunning = true;
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_AVATAR_STOP: {
                if (avatar != NULL && avatarRunning) {
                    avatarHalt(100);   // frame boundary, then screen is ours
                    avatarRunning = false;
                    imageOnScreen = false;
                    M5.Display.fillScreen(TFT_BLACK);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_AVATAR_EXPR: {
                if (avatar != NULL) {
                    avatar->setExpression((m5avatar::Expression)dataIn[0]);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_AVATAR_SPEECH: {
                // Length-prefixed text. Avatar::speechText is a String the
                // render task copies EVERY frame; reassigning a non-empty
                // text while it runs frees the buffer mid-copy and crashes
                // the board. Halt the tasks around the write.
                if (avatar != NULL) {
                    byte len = dataIn[0];
                    if (len > sizeof(speechBuf) - 1) {
                        len = sizeof(speechBuf) - 1;
                    }
                    char tmp[sizeof(speechBuf)];
                    memcpy(tmp, dataIn + 1, len);
                    tmp[len] = '\0';
                    if (strcmp(tmp, speechBuf) != 0) {  // skip no-op rewrites
                        memcpy(speechBuf, tmp, len + 1);
                        if (avatarRunning) {
                            avatarHalt(100);
                            avatar->setSpeechText(speechBuf);
                            avatar->start();
                        } else {
                            avatar->setSpeechText(speechBuf);
                        }
                    }
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_AVATAR_MOUTH: {
                if (avatar != NULL) {
                    avatar->setMouthOpenRatio(dataIn[0] / 100.0f);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }
            case SC_AVATAR_GAZE: {
                if (avatar != NULL) {
                    float h = ((int8_t)dataIn[0]) / 100.0f;
                    float v = ((int8_t)dataIn[1]) / 100.0f;
                    avatar->setLeftGaze(v, h);
                    avatar->setRightGaze(v, h);
                }
                byte ok = 1;
                sendResponseMsg(cmdID, &ok, 1);
                break;
            }

            case SC_SYS_INFO: {
                // Diagnostics: why did the board last reset (esp_reset_reason
                // values: 1 poweron, 3 sw, 4 panic, 5 int-wdt, 6 task-wdt,
                // 9 brownout, ...) and how does the heap look.
                byte out[9];
                out[0] = bootResetReason;
                uint32_t freeHeap = (uint32_t)ESP.getFreeHeap();
                uint32_t maxAlloc = (uint32_t)ESP.getMaxAllocHeap();
                memcpy(out + 1, &freeHeap, 4);
                memcpy(out + 5, &maxAlloc, 4);
                sendResponseMsg(cmdID, out, sizeof(out));
                break;
            }
            case SC_TEST_READ: {
                byte message[] = "Hello from StackChan";
                sendResponseMsg(cmdID, message, sizeof(message) - 1);
                break;
            }
            default: {
                break;
            }
        }
    }
};
