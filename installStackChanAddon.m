% installStackChanAddon.m
% One-time setup for the StackChan MATLAB Arduino add-on.
%
% Run this script from the StackChan_lib folder after installing:
%   1. MATLAB Support Package for Arduino Hardware
%      (during its setup, select ESP32 boards so the ESP32-S3 core is installed)
%
% It then:
%   - installs the required Arduino libraries into the support package's
%     private arduino-cli environment (pinned to known-working versions):
%       M5Unified 0.2.13, M5GFX 0.2.19, M5Stack_Avatar 0.10.0 (Arduino registry)
%       StackChan-BSP 1.1.0 (from GitHub, not in the registry)
%   - adds this folder (which holds +arduinoioaddons) to the MATLAB path
%   - verifies the add-on is visible to listArduinoLibraries
%
% Afterwards, connect with:
%   a  = arduino('<port>', 'ESP32-S3-DevKitC', 'Libraries', 'StackChanFolder/StackChan');
%   sc = addon(a, 'StackChanFolder/StackChan');

registryLibs = {'M5Unified@0.2.13', 'M5GFX@0.2.19', 'M5Stack_Avatar@0.10.0'};
bspGitUrl    = 'https://github.com/m5stack/StackChan-BSP.git#1.1.0';

%% Locate the Arduino support package's arduino-cli
try
    cliRoot = arduinoio.CLIRoot;
catch
    error(['MATLAB Support Package for Arduino Hardware not found. ', ...
           'Install it via Add-Ons > Get Hardware Support Packages, ', ...
           'select ESP32 boards during setup, then re-run this script.']);
end

if ispc
    cliBin = fullfile(cliRoot, 'arduino-cli.exe');
else
    cliBin = fullfile(cliRoot, 'arduino-cli');
end
cliCfg = fullfile(cliRoot, 'arduino-cli.yaml');
if ~isfile(cliBin)
    error('arduino-cli not found at %s — unexpected support package layout.', cliBin);
end
fprintf('Using arduino-cli at:\n  %s\n\n', cliBin);

runCli = @(args) system(sprintf('"%s" --config-file "%s" %s', cliBin, cliCfg, args));

%% Install registry libraries (pinned versions)
fprintf('Installing Arduino libraries from the registry...\n');
[status, ~] = runCli('lib update-index');
if status ~= 0
    warning('Library index update failed (offline?). Trying installs anyway.');
end
% --no-deps: without it arduino-cli "helpfully" upgrades pinned libraries
% to the newest versions their dependents allow, defeating the pins.
status = runCli(['lib install --no-deps ', strjoin(cellfun(@(s) ['"' s '"'], ...
    registryLibs, 'UniformOutput', false), ' ')]);
if status ~= 0
    error('Registry library install failed. Check the messages above (network access is required).');
end

%% Install StackChan-BSP from GitHub (not in the Arduino registry)
bspDir = fullfile(cliRoot, 'user', 'libraries', 'StackChan-BSP');
if isfolder(bspDir)
    fprintf('StackChan-BSP already installed — skipping.\n');
else
    fprintf('Installing StackChan-BSP from GitHub...\n');
    % Installing from a git URL requires this opt-in (per-process env var,
    % so the support package's arduino-cli.yaml is left untouched).
    setenv('ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL', 'true');
    cleanupEnv = onCleanup(@() setenv('ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL', ''));
    status = runCli(['lib install --git-url "', bspGitUrl, '"']);
    clear cleanupEnv
    if status ~= 0 || ~isfolder(bspDir)
        error('StackChan-BSP install failed. Check network access to github.com and the messages above.');
    end
end

%% Add this folder (containing +arduinoioaddons) to the MATLAB path
thisDir = fileparts(mfilename('fullpath'));
if ~isfolder(fullfile(thisDir, '+arduinoioaddons'))
    error('+arduinoioaddons not found next to this script — run installStackChanAddon.m from the StackChan_lib folder.');
end
addpath(thisDir);
if savepath ~= 0
    warning(['Could not save the MATLAB path (permission issue). ', ...
             'The add-on works for this session; add "addpath(''%s'')" to your startup.m ', ...
             'to make it permanent.'], thisDir);
end

%% Verify
fprintf('\nVerifying registration...\n');
libs = listArduinoLibraries;
if any(strcmp(libs, 'StackChanFolder/StackChan'))
    fprintf(['\nSuccess! StackChan add-on is installed.\n\n', ...
             'Connect with:\n', ...
             '  a  = arduino(''<port>'', ''ESP32-S3-DevKitC'', ''Libraries'', ''StackChanFolder/StackChan'');\n', ...
             '  sc = addon(a, ''StackChanFolder/StackChan'');\n\n', ...
             'The first arduino() call compiles and flashes the firmware (takes a few minutes).\n', ...
             'Then try testLED.m or testStackChan.m.\n']);
else
    error(['Libraries installed, but ''StackChanFolder/StackChan'' is not listed by ', ...
           'listArduinoLibraries. Check that the +arduinoioaddons folder was shared intact.']);
end
