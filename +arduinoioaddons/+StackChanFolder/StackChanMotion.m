% StackChanMotion.m
% Head-servo facade named exactly like the StackChan-BSP C++ API: what a
% sketch writes as M5StackChan.Motion.move(...) is sc.Motion.move(...)
% here, with the same argument order and the same speed default (500).
%
%   sc.Motion.move(45, 30);            % yaw, pitch (deg), speed optional
%   sc.Motion.moveYaw(-20);            % one axis; the other holds its
%   sc.Motion.movePitch(50);           %   last commanded value
%   sc.Motion.goHome();
%   sc.Motion.rotateYaw(-300);         % continuous spin (yaw only)
%   sc.Motion.stop();
%   yaw   = sc.Motion.getCurrentYawAngle();    % feedback servos; one
%   pitch = sc.Motion.getCurrentPitchAngle();  %   serial round trip each
%
% The moveX/moveY/rotateX/getCurrentXAngle/getCurrentYAngle aliases from
% the BSP exist too (X = yaw, Y = pitch). Reading both angles at once is
% cheaper via readHeadPosition(sc) (a single round trip).
%
% Not exposed (no firmware command yet): lookAtNormalized, lookAtPoint,
% isXMoving/isYMoving, setTorqueEnabled, setAutoTorqueReleaseEnabled.
%
% Obtained from the add-on's Motion property — do not construct directly.

classdef StackChanMotion < handle

    properties(Access = private)
        Owner   % the StackChan add-on this facade drives
    end

    methods(Access = ?arduinoioaddons.StackChanFolder.StackChan)
        function obj = StackChanMotion(owner)
            obj.Owner = owner;
        end
    end

    methods
        function move(obj, yawAngle, pitchAngle, speed)
            % Animated move to (yaw, pitch) in degrees.
            % yaw -128..128 (positive = left), pitch 0..90 (up),
            % speed 0..1000 (default 500, like the BSP).
            if nargin < 4
                speed = 500;
            end
            moveHead(obj.Owner, yawAngle, pitchAngle, speed);
        end

        function moveYaw(obj, angle, speed)
            % Yaw-only move; pitch holds its last commanded value.
            if nargin < 3
                speed = 500;
            end
            moveHead(obj.Owner, angle, obj.Owner.LastPitchCmd, speed);
        end

        function moveX(obj, angle, speed)
            % BSP alias: X = yaw.
            if nargin < 3
                speed = 500;
            end
            moveYaw(obj, angle, speed);
        end

        function movePitch(obj, angle, speed)
            % Pitch-only move; yaw holds its last commanded value.
            if nargin < 3
                speed = 500;
            end
            moveHead(obj.Owner, obj.Owner.LastYawCmd, angle, speed);
        end

        function moveY(obj, angle, speed)
            % BSP alias: Y = pitch.
            if nargin < 3
                speed = 500;
            end
            movePitch(obj, angle, speed);
        end

        function goHome(obj, speed)
            % Move the head back to (0, 0).
            if nargin < 2
                speed = 500;
            end
            goHome(obj.Owner, speed);
        end

        function stop(obj)
            % Stop any head movement immediately.
            stopHead(obj.Owner);
        end

        function rotateYaw(obj, velocity)
            % Spin continuously around yaw (the only 360-capable axis).
            % velocity -1000..1000, negative = clockwise; end with stop.
            rotateHead(obj.Owner, velocity);
        end

        function rotateX(obj, velocity)
            % BSP alias: X = yaw.
            rotateYaw(obj, velocity);
        end

        function angle = getCurrentYawAngle(obj)
            % Yaw (deg) from the feedback servo.
            [angle, ~] = readHeadPosition(obj.Owner);
        end

        function angle = getCurrentXAngle(obj)
            % BSP alias: X = yaw.
            angle = getCurrentYawAngle(obj);
        end

        function angle = getCurrentPitchAngle(obj)
            % Pitch (deg) from the feedback servo. Note: reads ~+19 deg
            % near the low end of the range on this unit (known bias).
            [~, angle] = readHeadPosition(obj.Owner);
        end

        function angle = getCurrentYAngle(obj)
            % BSP alias: Y = pitch.
            angle = getCurrentPitchAngle(obj);
        end
    end
end
