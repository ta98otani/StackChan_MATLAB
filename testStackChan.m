% testStackChan.m
% Full device test AND API tour for the official M5Stack StackChan via
% the custom add-on 'StackChanFolder/StackChan'.
%
% Method names mirror the original C++ libraries (StackChan-BSP for the
% body, m5stack-avatar for the face, M5Unified for the IMU), so this file
% doubles as a usage example for most of the MATLAB API: body LEDs, LCD,
% head servos (the sc.Motion facade + feedback reads), IMU, battery,
% camera (frame sizes, grayscale, LCD display, annotation overlays, live
% video), the avatar face, and board diagnostics.

%% Connect (reuse an existing connection if it is still alive)
port  = '/dev/cu.usbmodem101';
board = 'ESP32-S3-DevKitM';
if exist('sc', 'var') && isvalid(sc)
    try
        testRead(sc);   % probe: isvalid alone misses a dropped USB port
    catch
        clear sc a      % stale connection: rebuild both from scratch
        pause(2);       % let the OS release the serial port first
    end
end
if ~exist('a', 'var') || ~isvalid(a)
    a = arduino(port, board, 'Libraries', 'StackChanFolder/StackChan');
end
if ~exist('sc', 'var') || ~isvalid(sc)
    sc = addon(a, 'StackChanFolder/StackChan');  % body init, ~2 s
end
fprintf('Board says: %s\n', testRead(sc));

%% Board diagnostics
[reason, freeHeap, maxAlloc] = sysInfo(sc);
fprintf(['Reset reason %d (1 power-on, 4 panic, 9 brownout, 11 USB), ' ...
    'free heap %d B, largest block %d B\n'], reason, freeHeap, maxAlloc);

%% Body LEDs — BSP names; setRgbColor is 0-BASED (0-5 left, 6-11 right)
disp('--- LEDs: color sweep + chase ---');
showRgbColor(sc, 255, 0, 0); pause(0.7);
showRgbColor(sc, 0, 255, 0); pause(0.7);
showRgbColor(sc, 0, 0, 255); pause(0.7);
for idx = 0:sc.NumLEDs - 1
    showRgbColor(sc, 0, 0, 0);
    setRgbColor(sc, idx, 255, 120, 0);
    refreshRgb(sc);   % no-op here (colors push immediately); kept so
    pause(0.1);       %   BSP-style set-then-refresh code runs unchanged
end
showRgbColor(sc, 0, 0, 0);

%% LCD
disp('--- LCD fill ---');
fillScreen(sc, 255, 0, 0); pause(0.4);
fillScreen(sc, 0, 255, 0); pause(0.4);
fillScreen(sc, 0, 0, 255); pause(0.4);
fillScreen(sc, 0, 0, 0);

%% Sensors — M5.Imu / BSP getter names; the read* variants fetch all
%% values in ONE serial round trip (prefer those inside control loops)
disp('--- IMU / battery ---');
accel = getAccel(sc);
gyro  = getGyro(sc);
mag   = getMag(sc);
fprintf('Accel [g]:  %7.3f %7.3f %7.3f\n', accel);
fprintf('Gyro [dps]: %7.2f %7.2f %7.2f\n', gyro);
fprintf('Mag [uT]:   %7.2f %7.2f %7.2f\n', mag);
fprintf('Battery: %.2f V, %.1f mA\n', ...
    getBatteryVoltage(sc), getBatteryCurrent(sc));
% Combined equivalents: [accel, gyro, mag] = readIMU(sc);
%                       [v, mA] = readBattery(sc);

%% Head servos — sc.Motion mirrors M5StackChan.Motion
disp('--- Head servos (feedback-verified) ---');
sc.Motion.goHome(); pause(2);
targets = [60 45; -60 10; 0 30];
for k = 1:size(targets, 1)
    sc.Motion.move(targets(k, 1), targets(k, 2)); pause(2);
    y = sc.Motion.getCurrentYawAngle();    % feedback servos, one round
    p = sc.Motion.getCurrentPitchAngle();  %   trip per value...
    fprintf('Commanded (%4d,%3d) -> measured (%6.1f,%6.1f)\n', ...
        targets(k, 1), targets(k, 2), y, p);
end
[y, p] = readHeadPosition(sc);             % ...or both in ONE round trip
fprintf('readHeadPosition:          measured (%6.1f,%6.1f)\n', y, p);
sc.Motion.moveYaw(40); pause(1.5);     % single-axis moves hold the
sc.Motion.movePitch(50); pause(1.5);   %   other axis's last command
disp('Continuous yaw spin...');
sc.Motion.rotateYaw(200); pause(1.5);  % yaw is the only 360-capable axis
sc.Motion.stop(); pause(0.5);
sc.Motion.goHome(); pause(2);

%% Camera — MATLAB webcam idiom (the BSP has no camera API). The
%% big-frame test runs BEFORE the avatar section on purpose: avatar
%% allocations can fragment DRAM and block qvga's 154 KB frame buffer.
disp('--- Camera ---');
try
    sc.FrameSize = 'qvga';              % 320x240 (largest)
    img = snapshot(sc);
    fprintf('qvga color snapshot: %dx%dx%d\n', ...
        size(img, 2), size(img, 1), size(img, 3));
catch ME
    fprintf('qvga snapshot skipped (%s)\n', ME.message);
end
sc.FrameSize = 'qqvga';                 % 160x120, fastest (re-init ~2 s)
sc.ColorMode = 'grayscale';             % HxW uint8 luma, half the bytes
g = snapshot(sc);
fprintf('qqvga grayscale snapshot: %dx%d %s\n', ...
    size(g, 2), size(g, 1), class(g));
sc.ColorMode = 'color';
img = snapshot(sc);
figure('Name', 'StackChan camera'); image(img); axis image;

% Board-side display + annotation overlays (the LCD always shows the
% full-color frame, even when snapshots are grayscale)
showImage(sc);                          % frame from the last snapshot
setAnnotation(sc, [40 30 60 45], 'hello', [0 255 0]);  % drawn by the board
pause(2);
showImage(sc, 'new');                   % fresh frame; overlay persists
pause(2);

% Live view: the board captures and draws by itself (~20 fps); overlays
% and other commands keep working while it streams
disp('Live video with annotation overlay...');
setAnnotation(sc, [60 40 40 40], 'live', [255 255 0]);
startVideo(sc);
t0 = tic; pause(5);
nFrames = stopVideo(sc);
fprintf('Video: %.1f fps\n', double(nFrames) / toc(t0));
clearAnnotation(sc);

%% Avatar face — m5stack-avatar method names
disp('--- Avatar ---');
startAvatar(sc); pause(1);
setSpeechText(sc, 'Hello MATLAB!');
expressions = {'happy', 'angry', 'sad', 'doubt', 'sleepy', 'neutral'};
for k = 1:numel(expressions)
    fprintf('Expression: %s\n', expressions{k});
    setExpression(sc, expressions{k});
    pause(1.2);
end
setSpeechText(sc, '');
% setGaze(sc, vertical, horizontal) — the avatar library's argument
% order (same as setLeftGaze/setRightGaze); positive vertical = down
setGaze(sc, 0, 1);    pause(0.7);   % eyes right
setGaze(sc, 0, -1);   pause(0.7);   % eyes left
setGaze(sc, -0.8, 0); pause(0.7);   % eyes up
setGaze(sc, 0, 0);
for r = [0.2 0.6 1.0 0.6 0.2 0]     % mouth open/close like speaking
    setMouthOpenRatio(sc, r); pause(0.15);
end

%% Photo <-> face switching (snapshot works while the avatar runs)
img = snapshot(sc);          % capture works with the face on screen
showImage(sc); pause(2);     % the photo replaces the face...
showAvatar(sc);              % ...and the face comes back

disp('StackChan device test finished (avatar left running).');
