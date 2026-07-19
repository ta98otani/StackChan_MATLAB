% StackChan.m
% Custom Arduino add-on for the official M5Stack StackChan robot
% (M5Stack CoreS3 core + StackChan body, ESP32-S3).
%
% Wraps original vendor libraries with thin MATLAB commands:
%   - StackChan-BSP (M5Stack):   body LEDs (PY32L020 @ I2C 0x6F), serial-bus
%                                servos (Motion), battery monitor (INA226)
%   - M5Unified (M5Stack):       PMU, LCD, IMU (BMI270 + BMM150)
%   - m5stack-avatar (meganetaaan, the original StackChan author): the face
%
% Method names follow the original C++ APIs (StackChan-BSP for body and
% servos, m5stack-avatar for the face, M5Unified for the IMU) so sketch
% experience transfers directly. Differences that matter:
%   - setRgbColor uses the BSP's 0-BASED index (0-5 left ear, 6-11
%     right); the old 1-based writePixelColor still works but is hidden.
%   - refreshRgb exists but is a no-op: colors are pushed immediately.
%   - readHeadPosition / readIMU / readBattery are MATLAB-side combined
%     getters (one serial round trip) — prefer them in tight loops over
%     the per-value BSP getters they wrap.
%   - setGaze takes (vertical, horizontal) like the avatar library's
%     setLeftGaze/setRightGaze — before 2026-07-19 it was (h, v).
%   - The camera has no BSP equivalent, so it follows MATLAB's webcam
%     idiom (snapshot etc.); FrameSize values are esp32-camera names.
%   - Legacy names (writeColor, writePixelColor, clearLED, moveHead,
%     goHome, rotateHead, stopHead, setMouthOpen) still run old scripts.
%
% Usage:
%   a  = arduino('/dev/cu.usbmodem101', 'ESP32-S3-DevKitC', ...
%                'Libraries', 'StackChanFolder/StackChan');
%   sc = addon(a, 'StackChanFolder/StackChan');
%
%   % Body LEDs / LCD (BSP names)
%   showRgbColor(sc, 255, 0, 0);          % all 12 body LEDs red
%   setRgbColor(sc, 0, 0, 0, 255);        % one LED, 0-based like the BSP
%   showRgbColor(sc, 0, 0, 0);            % all off
%   fillScreen(sc, 0, 0, 0);
%
%   % Head servos: sc.Motion mirrors M5StackChan.Motion
%   sc.Motion.move(45, 30);               % yaw -128..128, pitch 0..90 (deg),
%                                         %   speed 0..1000 (default 500)
%   sc.Motion.moveYaw(-20);               % one axis at a time
%   sc.Motion.movePitch(50);
%   yaw = sc.Motion.getCurrentYawAngle(); % feedback servos
%   [yaw, pitch] = readHeadPosition(sc);  % ...both in one round trip
%   sc.Motion.goHome();
%   sc.Motion.rotateYaw(-300);            % continuous spin (yaw only)
%   sc.Motion.stop();
%
%   % Sensors (M5.Imu / BSP names)
%   accel = getAccel(sc);                 % [x y z] in g (getGyro, getMag too)
%   [accel, gyro, mag] = readIMU(sc);     % all nine in one round trip
%   v  = getBatteryVoltage(sc);
%   mA = getBatteryCurrent(sc);
%
%   % Camera (GC0308) — MATLAB webcam idiom
%   sc.FrameSize = 'qvga';                % capture resolution, up to 320x240
%   sc.ColorMode = 'grayscale';           % HxW luma: ~2x faster than color
%   img = snapshot(sc);                   % capture; auto-inits the camera
%   showImage(sc);                        % show the captured photo on the LCD
%   showImage(sc, 'new');                 % capture a fresh frame and show it
%   startVideo(sc);                       % smooth live view (board-side loop)
%   n = stopVideo(sc);                    % stop; returns frames shown
%   setAnnotation(sc, [40 30 60 45], 'ball');  % ROI overlay on the LCD
%   clearAnnotation(sc);                  % remove overlays
%   showAvatar(sc);                       % switch the LCD back to the face
%
%   % Avatar (m5stack-avatar names)
%   startAvatar(sc);                      % avatar.init() / start()
%   setExpression(sc, 'happy');           % happy/angry/sad/doubt/sleepy/neutral
%   setSpeechText(sc, 'Hello MATLAB!');   % speech balloon ('' to clear)
%   setMouthOpenRatio(sc, 0.6);           % 0..1
%   setGaze(sc, -0.2, 0.8);               % (vertical, horizontal), -1..1
%   stopAvatar(sc);

classdef StackChan < matlabshared.addon.LibraryBase

    properties(Access = private, Constant = true)
        % Command IDs — must match the #define values in src/StackChan.h.
        LED_WRITE_ALL   = hex2dec('02')
        LED_WRITE_PIXEL = hex2dec('03')
        LED_CLEAR       = hex2dec('04')
        TEST_READ       = hex2dec('0F')
        M5_BEGIN        = hex2dec('10')
        LCD_FILL        = hex2dec('11')
        SYS_INFO        = hex2dec('13')
        SERVO_MOVE      = hex2dec('20')
        SERVO_HOME      = hex2dec('21')
        SERVO_GET       = hex2dec('22')
        SERVO_ROTATE    = hex2dec('23')
        SERVO_STOP      = hex2dec('24')
        IMU_READ        = hex2dec('30')
        BATT_READ       = hex2dec('31')
        CAM_INIT        = hex2dec('40')
        CAM_CAPTURE     = hex2dec('41')
        CAM_READ        = hex2dec('42')
        CAM_SHOW        = hex2dec('43')
        VIDEO_START     = hex2dec('44')
        VIDEO_STOP      = hex2dec('45')
        ANNOT_SET       = hex2dec('46')
        AVATAR_START    = hex2dec('50')
        AVATAR_STOP     = hex2dec('51')
        AVATAR_EXPR     = hex2dec('52')
        AVATAR_SPEECH   = hex2dec('53')
        AVATAR_MOUTH    = hex2dec('54')
        AVATAR_GAZE     = hex2dec('55')
        % Order matches m5avatar::Expression enum in Expression.h
        ExpressionNames = {'happy', 'angry', 'sad', 'doubt', 'sleepy', 'neutral'}
    end

    properties(Access = protected, Constant = true)
        LibraryName = 'StackChanFolder/StackChan'
        % Empty: the BSP uses M5Unified's own internal I2C (G11/G12),
        % not MATLAB's I2C bus, so no MATLAB bus dependency exists.
        DependentLibraries = {}
        % Folder names inside arduinoio.CLIRoot/user/libraries + header file
        LibraryHeaderFiles = {'StackChan-BSP/M5StackChan.h', ...
            'M5Unified/M5Unified.h', 'M5GFX/M5GFX.h', ...
            'M5Stack_Avatar/Avatar.h'}
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'StackChan.h')
        CppClassName = 'StackChan'
    end

    properties(SetAccess = private)
        NumLEDs = 12       % Body LED count (setRgbColor index 0-5 left ear,
                           % 6-11 right ear, as in the BSP)
        CameraReady = false
        % Head-servo control named like the C++ BSP: sc.Motion.move(...),
        % sc.Motion.goHome(), sc.Motion.getCurrentYawAngle(), ... — see
        % StackChanMotion for the full surface.
        Motion
    end

    properties(SetAccess = private, Hidden)
        % Last commanded head pose (deg): lets the Motion facade's
        % single-axis moves (moveYaw/movePitch) hold the other axis
        LastYawCmd   = 0
        LastPitchCmd = 0
    end

    properties
        % How snapshot pauses the avatar during the frame grab:
        %   3 (default) cooperatively stop/restart the avatar tasks at a
        %     frame boundary (~0.2 s extra per frame, never crashed)
        %   2 don't pause the avatar at all
        %   (any other value behaves like 3; the raw task-suspend modes
        %   proved fatal with the speech balloon and were removed)
        CaptureMode = 3

        % Camera capture resolution. One of:
        %   '96x96' | 'qqvga' (160x120, default) | '128x128'
        %   'qcif' (176x144) | 'hqvga' (240x176) | '240x240'
        %   'qvga' (320x240)
        % Applied on the next camera use (snapshot / showImage /
        % startVideo), which re-initializes the camera if it was already
        % running at a different size (~2 s). Without PSRAM the RGB565
        % frame must fit in one DRAM block (38 KB at qqvga, 154 KB at
        % qvga), so the largest sizes can fail on a busy heap — the error
        % message will say so; pick a smaller size then.
        FrameSize = 'qqvga'

        % Pixel format of returned snapshots:
        %   'color'     (default) HxWx3 uint8 RGB
        %   'grayscale' HxW uint8 luma, converted on the board
        % Grayscale halves the bytes over serial (~1.6x faster snapshots)
        % and is what detectors like vision.CascadeObjectDetector want
        % anyway. The LCD (showImage / annotations) always shows the
        % full-color frame regardless. Falls back to color if the board
        % has no room for its display copy (largest FrameSize values).
        ColorMode = 'color'
    end

    properties(Hidden)
        % Bytes per CAM_READ transfer. Each transfer is one full command
        % round trip (~7 ms), so bigger chunks -> faster snapshots; the
        % board streams straight from the frame buffer with no size cap
        % below 65535 (count is uint16 on the wire). Speed saturates
        % around 9600; reduce if reads ever come back short/corrupted.
        CamChunkSize (1, 1) double {mustBeInteger, ...
            mustBeInRange(CamChunkSize, 64, 65535)} = 38400
    end

    properties(Access = private, Constant = true, Hidden = true)
        % FrameSize names and their esp32-camera framesize_t enum values
        % (sensor.h). Only sizes whose RGB565 frame can fit in DRAM.
        FrameSizeNames = {'96x96', 'qqvga', '128x128', 'qcif', ...
            'hqvga', '240x240', 'qvga'}
        FrameSizeCodes = [0, 1, 2, 3, 4, 5, 6]
    end

    properties(Access = private)
        % True once the board holds a frame from a real capture (snapshot or
        % showImage), as opposed to the stale frame grabbed at camera init
        HasSnapshot = false
        % FrameSize the camera was actually initialized with; a mismatch
        % against FrameSize triggers a re-init on the next camera use
        AppliedFrameSize = ''
    end

    methods
        function obj = StackChan(parentObj)
            % Connect to the StackChan add-on and initialize the robot body.
            obj.Parent = parentObj;

            % Lazy hardware init (never in the C++ setup()): starts M5Unified
            % (PMU, LCD, IMU) with serial logging off, then M5StackChan.begin()
            % (touch, IO-expander/LEDs, servos, battery monitor). Takes ~2 s.
            sendCmd(obj, obj.M5_BEGIN, uint8(0));
            obj.Motion = arduinoioaddons.StackChanFolder.StackChanMotion(obj);
        end

        %% ------------------------- LEDs / LCD --------------------------
        function showRgbColor(obj, r, g, b)
            % Set all 12 body LEDs to one color (0-255 each) — same name
            % as the BSP's M5StackChan.showRgbColor. All off:
            % showRgbColor(sc, 0, 0, 0).
            validateRGB(obj, r, g, b);
            sendCmd(obj, obj.LED_WRITE_ALL, ...
                uint8([r, g, b]));
        end

        function setRgbColor(obj, index, r, g, b)
            % Set one body LED — same name and 0-BASED index as the
            % BSP's M5StackChan.setRgbColor: 0-5 left ear, 6-11 right
            % ear. Unlike the BSP, the color shows immediately (see
            % refreshRgb).
            validateattributes(index, {'numeric'}, ...
                {'scalar', 'integer', '>=', 0, '<=', obj.NumLEDs - 1});
            validateRGB(obj, r, g, b);
            sendCmd(obj, obj.LED_WRITE_PIXEL, ...
                uint8([index, r, g, b]));
        end

        function refreshRgb(~)
            % No-op, kept for BSP source compatibility: on this add-on
            % every setRgbColor/showRgbColor pushes the LEDs at once, so
            % C++-style "setRgbColor... then refreshRgb()" sequences run
            % unchanged.
        end

        function fillScreen(obj, r, g, b)
            % Fill the CoreS3 LCD with a solid color (each value 0-255).
            % Takes over the screen: a running avatar or live video is
            % stopped first (use showAvatar to bring the face back).
            validateRGB(obj, r, g, b);
            sendCmd(obj, obj.LCD_FILL, uint8([r, g, b]));
        end

        %% ------------------------- Head servos -------------------------
        % Head motion lives on the Motion facade, named like the C++
        % BSP: sc.Motion.move / moveYaw / movePitch / goHome / stop /
        % rotateYaw / getCurrentYawAngle / getCurrentPitchAngle — see
        % StackChanMotion.m. Only the combined feedback read lives here:

        function [yawDeg, pitchDeg] = readHeadPosition(obj)
            % Both feedback-servo angles (deg) in ONE serial round trip —
            % use this in control loops; Motion.getCurrentYawAngle and
            % Motion.getCurrentPitchAngle cost one round trip each.
            response = sendCmdChecked(obj, obj.SERVO_GET, uint8(0), 4);
            raw = uint8(response(1:4)');
            yawDeg   = double(typecast(raw(1:2), 'int16')) / 10;
            pitchDeg = double(typecast(raw(3:4), 'int16')) / 10;
        end

        %% --------------------------- Sensors ---------------------------
        function [accel, gyro, mag] = readIMU(obj)
            % Read the IMU. accel = [x y z] in g, gyro = [x y z] in deg/s,
            % mag = [x y z] in uT (BMI270 + BMM150 via M5Unified).
            response = sendCmdChecked(obj, obj.IMU_READ, uint8(0), 36);
            vals = double(typecast(uint8(response(1:36)'), 'single'));
            accel = vals(1:3);
            gyro  = vals(4:6);
            mag   = vals(7:9);
        end

        function [voltage, current_mA] = readBattery(obj)
            % Battery voltage (V) and current (mA, positive =
            % discharging) from the body's INA226 monitor, in one serial
            % round trip. BSP-named getters below wrap this.
            response = sendCmdChecked(obj, obj.BATT_READ, uint8(0), 8);
            vals = double(typecast(uint8(response(1:8)'), 'single'));
            voltage = vals(1);
            current_mA = vals(2);
        end

        function voltage = getBatteryVoltage(obj)
            % Battery voltage (V) — same name as the BSP.
            [voltage, ~] = readBattery(obj);
        end

        function current_mA = getBatteryCurrent(obj)
            % Battery current (mA, positive = discharging) — same name
            % as the BSP.
            [~, current_mA] = readBattery(obj);
        end

        function accel = getAccel(obj)
            % Accelerometer [x y z] in g — named like M5.Imu.getAccel.
            [accel, ~, ~] = readIMU(obj);
        end

        function gyro = getGyro(obj)
            % Gyro [x y z] in deg/s — named like M5.Imu.getGyro.
            [~, gyro, ~] = readIMU(obj);
        end

        function mag = getMag(obj)
            % Magnetometer [x y z] in uT — named like M5.Imu.getMag.
            [~, ~, mag] = readIMU(obj);
        end

        %% --------------------------- Camera -----------------------------
        function set.FrameSize(obj, value)
            obj.FrameSize = validatestring(value, {'96x96', 'qqvga', ...
                '128x128', 'qcif', 'hqvga', '240x240', 'qvga'});
        end

        function set.ColorMode(obj, value)
            obj.ColorMode = validatestring(value, {'color', 'grayscale'});
        end

        function initCamera(obj)
            % Initialize (or re-initialize) the GC0308 camera at the
            % resolution given by the FrameSize property. Called
            % automatically by snapshot / showImage / startVideo, also
            % when FrameSize changed since the camera last initialized.
            obj.CameraReady = false;
            code = obj.FrameSizeCodes(strcmp(obj.FrameSizeNames, ...
                obj.FrameSize));
            response = sendCmd(obj, obj.CAM_INIT, ...
                uint8(code + 1));   % board expects framesize_t + 1
            if response(1) == 0
                error('StackChan:cameraInitFailed', ...
                    ['Camera init failed for FrameSize ''%s''. Large ' ...
                     'frames need one contiguous DRAM block and can fail ' ...
                     'on a busy heap - try a smaller FrameSize.'], ...
                    obj.FrameSize);
            end
            obj.CameraReady = true;
            obj.AppliedFrameSize = obj.FrameSize;
            obj.HasSnapshot = false;  % held frame is the init grab
        end

        function img = snapshot(obj)
            % Capture one camera frame and return it as an image: HxWx3
            % uint8 RGB, or HxW uint8 when ColorMode is 'grayscale'.
            % Size follows the FrameSize property (default 160x120).
            % Captures are pipelined: the board grabs the next frame in
            % the background while this one is transferred and processed,
            % so a snapshot loop runs near the sensor frame rate.
            ensureCamera(obj);
            gray = strcmp(obj.ColorMode, 'grayscale');
            % Grab a frame on the board; reply is [w u16, h u16, len u32]
            response = sendCmdChecked(obj, obj.CAM_CAPTURE, ...
                uint8([obj.CaptureMode, gray]), 8);
            raw = uint8(response(1:8)');
            w   = double(typecast(raw(1:2), 'uint16'));
            h   = double(typecast(raw(3:4), 'uint16'));
            len = double(typecast(raw(5:8), 'uint32'));
            if len == 0
                error('StackChan:captureFailed', 'Camera capture failed.');
            end
            % Pull the frame over in chunks
            data = zeros(1, len, 'uint8');
            offset = 0;
            while offset < len
                n = min(obj.CamChunkSize, len - offset);
                payload = [typecast(uint32(offset), 'uint8'), ...
                           typecast(uint16(n), 'uint8')];
                r = sendCmdChecked(obj, obj.CAM_READ, payload, n);
                data(offset+1 : offset+n) = uint8(r(1:n)');
                offset = offset + n;
            end
            if len == w * h
                % Board delivered 8-bit luma (grayscale mode)
                img = reshape(data, w, h)';
            else
                % RGB565, big-endian from the camera DMA; unpack to RGB888
                hi = uint16(data(1:2:end));
                lo = uint16(data(2:2:end));
                v  = bitor(bitshift(hi, 8), lo);
                R = uint8(double(bitshift(v, -11))           * (255 / 31));
                G = uint8(double(bitand(bitshift(v, -5), 63)) * (255 / 63));
                B = uint8(double(bitand(v, 31))               * (255 / 31));
                img = cat(3, reshape(R, w, h)', reshape(G, w, h)', reshape(B, w, h)');
            end
            obj.HasSnapshot = true;
        end

        function showImage(obj, mode)
            % Show the captured camera image full-screen on the LCD,
            % replacing the avatar face. Use showAvatar to switch back.
            %   showImage(sc)         show the frame from the last snapshot
            %                         (captures a new frame if none yet)
            %   showImage(sc, 'new')  capture a fresh frame, then show it
            % The image is drawn from the frame already held on the board,
            % so no pixel data crosses the serial link — it appears at once.
            ensureCamera(obj);
            if nargin < 2
                captureNew = ~obj.HasSnapshot;
            else
                validatestring(mode, {'new'});
                captureNew = true;
            end
            response = sendCmd(obj, obj.CAM_SHOW, ...
                uint8(captureNew));
            if response(1) == 0
                error('StackChan:showImageFailed', ...
                    'Could not display the camera image (no frame available).');
            end
            obj.HasSnapshot = true;
        end

        function showAvatar(obj)
            % Switch the LCD back to the animated avatar face
            % (counterpart of showImage; same as startAvatar).
            % Also ends a startVideo live view if one is running.
            startAvatar(obj);
        end

        function startVideo(obj)
            % Stream the camera to the LCD continuously (live view).
            % The board captures and draws frames by itself at the full
            % sensor rate — MATLAB is not in the loop, so this is smooth
            % where a showImage(sc,'new') loop is not. The avatar stops;
            % MATLAB commands still work during streaming (with a few
            % tens of ms extra latency). End with stopVideo (last frame
            % stays on screen) or showAvatar. Note: snapshot and showImage
            % also end the stream, since they need a stable frame.
            ensureCamera(obj);
            response = sendCmd(obj, obj.VIDEO_START, ...
                uint8(0));
            if response(1) == 0
                error('StackChan:videoStartFailed', ...
                    'Could not start the live view (camera not ready).');
            end
        end

        function setAnnotation(obj, bbox, label, color)
            % Overlay rectangles and labels on the camera image shown on
            % the LCD, like insertObjectAnnotation but drawn by the board:
            % only the coordinates cross the serial link, so the overlay
            % is free and persists on every frame (showImage and video)
            % until replaced by the next call or clearAnnotation.
            %   setAnnotation(sc, bbox)               yellow rectangles
            %   setAnnotation(sc, bbox, label)        with text tags
            %   setAnnotation(sc, bbox, label, color) RGB 0-255 per row
            % bbox is Mx4 [x y w h] (max 8 rows) in camera image pixels,
            % 1-based like MATLAB image coordinates. label is a string or
            % cellstr (max 23 chars each); color is Mx3 or 1x3.
            validateattributes(bbox, {'numeric'}, {'2d', 'ncols', 4});
            m = size(bbox, 1);
            assert(m >= 1 && m <= 8, 'StackChan:tooManyAnnotations', ...
                'bbox must have 1 to 8 rows.');
            if nargin < 3 || isempty(label)
                label = repmat({''}, m, 1);
            elseif ischar(label) || isstring(label)
                label = cellstr(label);
            end
            if isscalar(label)
                label = repmat(label, m, 1);
            end
            assert(numel(label) == m, 'StackChan:labelCount', ...
                'Need one label per bbox row (or a single shared one).');
            if nargin < 4
                color = [255 255 0];   % classic annotation yellow
            end
            validateattributes(color, {'numeric'}, ...
                {'2d', 'ncols', 3, '>=', 0, '<=', 255});
            if size(color, 1) == 1
                color = repmat(color, m, 1);
            end
            payload = uint8(m);
            for k = 1:m
                bb = max(round(bbox(k, :)) - [1 1 0 0], 0);  % to 0-based
                txt = char(label{k});
                txt = txt(1:min(end, 23));
                payload = [payload, ...
                    typecast(uint16(bb), 'uint8'), ...
                    uint8(round(color(k, :))), ...
                    uint8(numel(txt)), uint8(txt)]; %#ok<AGROW> (M <= 8)
            end
            sendCmd(obj, obj.ANNOT_SET, payload);
        end

        function clearAnnotation(obj)
            % Remove all LCD annotations set by setAnnotation.
            sendCmd(obj, obj.ANNOT_SET, uint8(0));
        end

        function nFrames = stopVideo(obj)
            % Stop the live view; the last frame stays on the LCD.
            % Returns the number of frames shown since startVideo.
            response = sendCmdChecked(obj, obj.VIDEO_STOP, uint8(0), 4);
            nFrames = double(typecast(uint8(response(1:4)'), 'uint32'));
            obj.HasSnapshot = true;  % board holds the last streamed frame
        end

        %% ------------------- Avatar (StackChan face) -------------------
        function startAvatar(obj)
            % Start the animated StackChan face (m5stack-avatar) on the LCD.
            sendCmd(obj, obj.AVATAR_START, uint8(0));
        end

        function stopAvatar(obj)
            % Stop the avatar and release the screen.
            sendCmd(obj, obj.AVATAR_STOP, uint8(0));
        end

        function setExpression(obj, name)
            % Set the face expression:
            % 'happy' | 'angry' | 'sad' | 'doubt' | 'sleepy' | 'neutral'
            name = validatestring(name, obj.ExpressionNames);
            idx = find(strcmp(name, obj.ExpressionNames)) - 1;  % 0-based enum
            sendCmd(obj, obj.AVATAR_EXPR, uint8(idx));
        end

        function setSpeechText(obj, txt)
            % Show text in the avatar's speech balloon. '' clears it.
            txt = char(txt);
            validateattributes(numel(txt), {'numeric'}, {'<=', 60});
            % Length-prefixed so an empty string still sends >= 1 byte
            sendCmd(obj, obj.AVATAR_SPEECH, ...
                uint8([numel(txt), txt]));
        end

        function setMouthOpenRatio(obj, ratio)
            % Open the avatar's mouth: 0 (closed) .. 1 (fully open) —
            % same name as m5stack-avatar's Avatar::setMouthOpenRatio.
            validateattributes(ratio, {'numeric'}, {'scalar', '>=', 0, '<=', 1});
            sendCmd(obj, obj.AVATAR_MOUTH, ...
                uint8(round(ratio * 100)));
        end

        function setGaze(obj, vertical, horizontal)
            % Point BOTH of the avatar's eyes; each value -1..1, (0,0) =
            % straight ahead. The ARGUMENT ORDER (vertical, horizontal)
            % matches m5stack-avatar's setLeftGaze/setRightGaze/getGaze.
            % NOTE: before 2026-07-19 this method took (horizontal,
            % vertical) — swap the arguments in old call sites.
            validateattributes(vertical, {'numeric'}, {'scalar', '>=', -1, '<=', 1});
            validateattributes(horizontal, {'numeric'}, {'scalar', '>=', -1, '<=', 1});
            sendCmd(obj, obj.AVATAR_GAZE, ...
                [typecast(int8(round(horizontal * 100)), 'uint8'), ...
                 typecast(int8(round(vertical * 100)), 'uint8')]);
        end

        %% ---------------------------- Misc -----------------------------
        function [reason, freeHeap, maxAlloc] = sysInfo(obj)
            % Board diagnostics. reason = why the board last reset, per
            % esp_reset_reason: 1 power-on, 3 software, 4 panic,
            % 5 interrupt-watchdog, 6 task-watchdog, 9 brownout.
            % freeHeap / maxAlloc (bytes) show total free DRAM and the
            % largest contiguous block (fragmentation indicator).
            response = sendCmdChecked(obj, obj.SYS_INFO, uint8(0), 9);
            raw = uint8(response(1:9)');
            reason   = double(raw(1));
            freeHeap = double(typecast(raw(2:5), 'uint32'));
            maxAlloc = double(typecast(raw(6:9), 'uint32'));
        end

        function message = testRead(obj)
            % Round-trip connectivity test; returns a greeting string.
            response = sendCmd(obj, obj.TEST_READ, []);
            message = char(response');
        end
    end

    methods(Hidden)
        %% ---- Motion implementations (called via sc.Motion) and ----
        %% ---- legacy names kept so pre-2026-07-19 scripts still run ----
        function moveHead(obj, yawDeg, pitchDeg, speed)
            % Implementation behind sc.Motion.move (legacy direct name).
            % yaw: -128..128 (positive = left), pitch: 0..90 (up),
            % speed: 0..1000 (default 500).
            if nargin < 4
                speed = 500;
            end
            validateattributes(yawDeg, {'numeric'}, {'scalar', '>=', -128, '<=', 128});
            validateattributes(pitchDeg, {'numeric'}, {'scalar', '>=', 0, '<=', 90});
            validateSpeed(obj, speed);
            % BSP Motion uses 0.1-degree units, int16 little-endian
            sendCmd(obj, obj.SERVO_MOVE, ...
                [typecast(int16(round(yawDeg * 10)), 'uint8'), ...
                 typecast(int16(round(pitchDeg * 10)), 'uint8'), ...
                 typecast(uint16(speed), 'uint8')]);
            obj.LastYawCmd   = yawDeg;
            obj.LastPitchCmd = pitchDeg;
        end

        function goHome(obj, speed)
            % Implementation behind sc.Motion.goHome (legacy direct name).
            if nargin < 2
                speed = 500;
            end
            validateSpeed(obj, speed);
            sendCmd(obj, obj.SERVO_HOME, ...
                typecast(uint16(speed), 'uint8'));
            obj.LastYawCmd   = 0;
            obj.LastPitchCmd = 0;
        end

        function rotateHead(obj, velocity)
            % Implementation behind sc.Motion.rotateYaw (legacy name).
            validateattributes(velocity, {'numeric'}, ...
                {'scalar', 'integer', '>=', -1000, '<=', 1000});
            sendCmd(obj, obj.SERVO_ROTATE, ...
                typecast(int16(velocity), 'uint8'));
        end

        function stopHead(obj)
            % Implementation behind sc.Motion.stop (legacy direct name).
            sendCmd(obj, obj.SERVO_STOP, uint8(0));
        end

        function writeColor(obj, r, g, b)
            % Legacy alias of showRgbColor.
            showRgbColor(obj, r, g, b);
        end

        function writePixelColor(obj, index, r, g, b)
            % Legacy alias of setRgbColor with a 1-BASED index (1-12);
            % the BSP-style setRgbColor is 0-based (0-11).
            validateattributes(index, {'numeric'}, ...
                {'scalar', 'integer', 'positive', '<=', obj.NumLEDs});
            setRgbColor(obj, index - 1, r, g, b);
        end

        function clearLED(obj)
            % Legacy: all body LEDs off (same as showRgbColor(sc,0,0,0)).
            sendCmd(obj, obj.LED_CLEAR, uint8(0));
        end

        function setMouthOpen(obj, ratio)
            % Legacy alias of setMouthOpenRatio.
            setMouthOpenRatio(obj, ratio);
        end

        function flushLink(obj)
            % Drain the IO protocol's streaming buffer to recover the
            % serial link without reconnecting. Stray bytes accumulate
            % there over long runs — duplicate command responses (the
            % client silently re-sends a request the board hasn't
            % answered within 2 s, then both replies arrive) and any
            % ISR-context console output — and nothing ever drains it,
            % so once full EVERY later command errors with
            % 'The internal buffers to store data is full'. sendCmd
            % calls this automatically; it is public only for manual
            % recovery from the command line.
            ws = warning('off', 'MATLAB:structOnObject');
            restoreWarn = onCleanup(@() warning(ws));
            protocol = struct(obj.Parent).Protocol;
            flushBuffers(protocol);
            clearDataStorage(protocol);
            clearTransportBuffer(protocol);
        end
    end

    methods(Access = private)
        function response = sendCmdChecked(obj, cmdID, payload, minBytes)
            % sendCmd plus response-length validation. A short response
            % means the reply stream desynced — typically a stray
            % duplicate reply after the transport's silent 2-s retry
            % fired (seen when a sensor stall blocks the board's command
            % loop mid-scan) — so every later reply is off by one.
            % Flushing the protocol buffers realigns the stream; then
            % try once more.
            response = sendCmd(obj, cmdID, payload);
            if numel(response) >= minBytes
                return;
            end
            flushLink(obj);
            response = sendCmd(obj, cmdID, payload);
            if numel(response) < minBytes
                error('StackChan:shortResponse', ...
                    ['Board sent %d bytes where %d were expected ' ...
                     '(command 0x%02X), twice in a row.'], ...
                    numel(response), minBytes, cmdID);
            end
        end

        function response = sendCmd(obj, cmdID, payload)
            % sendCommand with self-healing: on the streaming-buffer-full
            % error, flush the protocol buffers and retry once. All
            % StackChan commands are idempotent, so a retry (which the
            % board may execute a second time) is safe.
            try
                response = sendCommand(obj, obj.LibraryName, cmdID, payload);
            catch ME
                if ~contains(ME.identifier, 'StreamingBufferFull') && ...
                        ~contains(ME.message, 'internal buffers')
                    rethrow(ME);
                end
                flushLink(obj);
                response = sendCommand(obj, obj.LibraryName, cmdID, payload);
            end
        end


        function ensureCamera(obj)
            % Init the camera if it isn't running yet, or re-init it if
            % the FrameSize property changed since it was initialized.
            if ~obj.CameraReady || ~strcmp(obj.AppliedFrameSize, obj.FrameSize)
                initCamera(obj);
            end
        end

        function validateRGB(~, r, g, b)
            validateattributes(r, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 255});
            validateattributes(g, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 255});
            validateattributes(b, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 255});
        end

        function validateSpeed(~, speed)
            validateattributes(speed, {'numeric'}, ...
                {'scalar', 'integer', '>=', 0, '<=', 1000});
        end
    end
end
