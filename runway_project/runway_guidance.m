function g = runway_guidance(xhat, lock)
%RUNWAY_GUIDANCE  Turn tracked runway edges into landing-assist cues.
%   g = [dev_m; yaw_deg; wHalf_px; vpx; vpy; valid]
%     dev_m     lateral deviation from the extended centre-line [m],
%               positive = aircraft is RIGHT of the centre-line
%     yaw_deg   angle between camera axis and runway direction [deg],
%               from the vanishing point of the two edges
%     wHalf_px  runway half-width at the bottom row [px] (grows on approach,
%               a crude range cue)
%     vpx, vpy  vanishing point of the two edges [px]
%     valid     = lock
%
%   Lateral deviation uses similar triangles at the bottom image row:
%   (pixel offset of centre-line)/(pixel half-width) = (metric offset)/(runway
%   half-width).  Valid for a camera aligned with the runway axis.
%#codegen
p  = runway_params();
f  = p.cx / tand(p.hfovDeg/2);           % focal length in pixels
xLb = xhat(1);  xLt = xhat(2);
xRb = xhat(3);  xRt = xhat(4);

wHalf = (xRb - xLb)/2;
cbEst = (xLb + xRb)/2;
dev   = (p.cx - cbEst) / max(wHalf, 1) * p.rwWidthM/2;

dy  = p.H - p.roiTop;
sL  = (xLt - xLb)/dy;                    % edge slope, px of x per row going up
sR  = (xRt - xRb)/dy;
den = sL - sR;
if abs(den) > 1e-3
    u   = (xRb - xLb)/den;               % rows above the bottom row where edges meet
    vpy = p.H - u;
    vpx = xLb + sL*u;
else
    vpy = 0;
    vpx = p.cx;
end
yaw = atand((vpx - p.cx)/f);

g = [dev; yaw; wHalf; vpx; vpy; lock];
end
