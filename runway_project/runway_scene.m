function [img, truth] = runway_scene(t, vis, sigma)
%RUNWAY_SCENE  Synthetic approach-view of a runway (stand-in for camera video).
%   [img, truth] = runway_scene(t, vis, sigma)
%     t      time [s]
%     vis    visibility / contrast scale, 1 = clear, 0.3 = heavy haze
%     sigma  std of additive Gaussian sensor noise (image range 0..1)
%     img    H x W grey image in 0..1
%     truth  [cb; w; vx]  runway centre at bottom row, half-width at bottom
%            row, vanishing-point column  (all in pixels)
%
%   Pinhole camera, flat ground, no roll/pitch.  The aircraft drifts
%   laterally (cb moves), approaches (w grows) and yaws slightly (vx moves).
%   Clutter: dashed centre-line, a taxiway joining on the left, noise, haze.
%#codegen
p = runway_params();
H = p.H;  W = p.W;  hr = p.horizon;

cb = p.cx + 15*sin(0.35*t);        % runway centre at the bottom row  [px]
w  = 30 + 1.2*t;                   % runway half-width at bottom row  [px]
vx = p.cx + 4*sin(0.2*t + 1);      % vanishing-point column           [px]
truth = [cb; w; vx];

img = zeros(H, W);
img(1:hr, :)   = 0.75;             % sky
img(hr+1:H, :) = 0.35;             % grass

nR   = H - hr;                     % number of rows below the horizon
rows = (hr+1:H).';                 % rows below the horizon (column vector)
frac = (rows - hr) / (H - hr);     % 0 at horizon -> 1 at bottom row
cen  = vx + (cb - vx)*frac;        % runway centre-line column in every row
half = w*frac;                     % runway half-width in every row

% Expand everything to full nR x W matrices explicitly. (Simulink MATLAB
% Function blocks in R2019a do not support implicit array expansion.)
X    = repmat(1:W, nR, 1);         % column index of every pixel
CEN  = repmat(cen,  1, W);
HALF = repmat(half, 1, W);

% anti-aliased runway surface (coverage 0..1) so edges are sub-pixel exact
cover = min(max(HALF - abs(X - CEN) + 0.5, 0), 1);
band  = 0.35 + 0.25*cover;         % grass 0.35 -> runway 0.60

% dashed centre-line, low contrast, only where it is big enough to resolve
phase = mod(4./max(frac, 0.05) - 3*t, 1);
dash  = double(phase < 0.5 & frac > 0.3);
lwh   = max(0.5, 0.02*half);
DASH  = repmat(dash, 1, W);
LWH   = repmat(lwh,  1, W);
dcov  = min(max(LWH - abs(X - CEN) + 0.5, 0), 1);
band  = band + 0.12*(DASH .* dcov);

% taxiway joining from the left (distractor, rows 95..98, same grey as runway)
r1 = 95 - hr;  r2 = 98 - hr;
taxi = min(max((CEN - HALF) - X + 0.5, 0), 1);
band(r1:r2, :) = band(r1:r2, :) + 0.25*taxi(r1:r2, :);

img(hr+1:H, :) = band;

% haze (contrast loss) and sensor noise
img = 0.5 + vis*(img - 0.5);
img = img + sigma*randn(H, W);
img = min(max(img, 0), 1);
end
