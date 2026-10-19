function p = runway_params()
%RUNWAY_PARAMS  Single source of truth for geometry and tuning constants.
%   Used by the scene generator, detector, tracker, guidance, logger and
%   overlay, so every Simulink block and every script sees the same numbers.
%#codegen

% ---- camera / image ------------------------------------------------------
p.H        = 120;     % image rows
p.W        = 160;     % image columns
p.cx       = 80;      % optical-axis column (image centre) [px]
p.horizon  = 40;      % horizon row in the synthetic scene [px]
p.hfovDeg  = 60;      % camera horizontal field of view [deg]
p.Ts       = 0.1;     % frame period [s]  (10 frames per second)

% ---- runway --------------------------------------------------------------
p.rwWidthM = 45;      % runway width [m]

% ---- detector (Sobel + Hough) -------------------------------------------
p.roiTop   = 45;      % first image row searched for runway edges
p.thrRel   = 0.35;    % edge threshold = fraction of strongest edge in ROI
p.thrMin   = 0.12;    % absolute floor for the edge threshold
p.thMin    = 5;       % min |theta| of the Hough line normal [deg]
p.thMax    = 60;      % max |theta| of the Hough line normal [deg]
p.minVotes = 40;      % min smoothed Hough votes to accept a line
p.minWidth = 20;      % min runway width at the bottom row [px]

% ---- tracker (constant-velocity Kalman filter) -------------------------
p.sigmaAccel = 8.0;   % process noise: acceleration std [px/s^2] (tuned, see README)
p.sigmaMeas  = 1.0;   % measurement noise std [px]                (tuned, see README)
p.gateMin    = 12;    % minimum gate half-width [px]
p.gateSigma  = 4;     % gate half-width in innovation std devs
p.maxMiss    = 5;     % frames without update before lock is dropped
p.minUpdates = 3;     % updates needed before lock is declared
p.prior      = [30; 78; 130; 82];   % initial [xLb; xLt; xRb; xRt] [px]
end
