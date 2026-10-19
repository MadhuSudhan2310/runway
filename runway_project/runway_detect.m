function [z, ok, votes] = runway_detect(img)
%RUNWAY_DETECT  Find the left and right runway edges with Sobel + Hough.
%   [z, ok, votes] = runway_detect(img)
%     img    H x W grey image, 0..1
%     z      [xLb; xLt; xRb; xRt]  column of each edge at the bottom row (b)
%            and at the top of the search region (t), in pixels
%     ok     [okL; okR]  1 when that edge was found
%     votes  [vL; vR]    smoothed Hough peak height (confidence)
%
%   Pipeline: Sobel gradient -> threshold -> Hough transform in two
%   separate theta windows (left edge leans one way, right edge the other)
%   -> peak picking with sub-bin refinement -> geometry sanity check.
%   Pure MATLAB (no toolbox calls) so it is Simulink code-generation safe.
%#codegen
p = runway_params();
[H, W] = size(img);

% ---- 1. Sobel gradient magnitude ---------------------------------------
gx = zeros(H, W);
gy = zeros(H, W);
gx(2:H-1, 2:W-1) = (img(1:H-2, 3:W) + 2*img(2:H-1, 3:W) + img(3:H, 3:W)) ...
                 - (img(1:H-2, 1:W-2) + 2*img(2:H-1, 1:W-2) + img(3:H, 1:W-2));
gy(2:H-1, 2:W-1) = (img(3:H, 1:W-2) + 2*img(3:H, 2:W-1) + img(3:H, 3:W)) ...
                 - (img(1:H-2, 1:W-2) + 2*img(1:H-2, 2:W-1) + img(1:H-2, 3:W));
mag = sqrt(gx.^2 + gy.^2);

% ---- 2. threshold inside the region of interest (below the horizon) ----
roi = mag(p.roiTop:H, :);
thr = max(p.thrRel*max(roi(:)), p.thrMin);
[yy, xx] = find(roi > thr);
yy = yy + p.roiTop - 1;                 % back to image row numbers

% ---- 3. Hough accumulators: line  x*cos(th) + y*sin(th) = rho ----------
thL  = p.thMin:p.thMax;                 % left edge  : normal tilts to +theta
thR  = -p.thMax:-p.thMin;               % right edge : normal tilts to -theta
nth  = numel(thL);
cL = cosd(thL);  sL = sind(thL);
cR = cosd(thR);  sR = sind(thR);
off = ceil(sqrt(H^2 + W^2));            % rho offset so indices are >= 1
nr  = 2*off + 1;
AL  = zeros(nr, nth);
AR  = zeros(nr, nth);
for n = 1:numel(yy)
    x = xx(n);
    y = yy(n);
    for k = 1:nth
        r = round(x*cL(k) + y*sL(k)) + off + 1;
        AL(r, k) = AL(r, k) + 1;
        r = round(x*cR(k) + y*sR(k)) + off + 1;
        AR(r, k) = AR(r, k) + 1;
    end
end

% ---- 4. strongest line in each window -----------------------------------
[rhoL, thetaL, vL] = hough_peak(AL, thL(1), off);
[rhoR, thetaR, vR] = hough_peak(AR, thR(1), off);
okL = vL >= p.minVotes;
okR = vR >= p.minVotes;

% ---- 5. line -> x position at the bottom row and at the ROI top row ----
yb = H;
yt = p.roiTop;
xLb = (rhoL - yb*sind(thetaL)) / cosd(thetaL);
xLt = (rhoL - yt*sind(thetaL)) / cosd(thetaL);
xRb = (rhoR - yb*sind(thetaR)) / cosd(thetaR);
xRt = (rhoR - yt*sind(thetaR)) / cosd(thetaR);

% ---- 6. sanity: the runway must have a sensible width -------------------
if okL && okR && (xRb - xLb) < p.minWidth
    if vL >= vR
        okR = false;
    else
        okL = false;
    end
end

z = zeros(4, 1);
if okL
    z(1) = xLb;  z(2) = xLt;
end
if okR
    z(3) = xRb;  z(4) = xRt;
end
ok    = [double(okL); double(okR)];
votes = [vL; vR];
end

function [rho, theta, vmax] = hough_peak(A, th0, off)
% Peak of a 3x3-smoothed accumulator, refined to sub-bin by a centroid.
[nr, nth] = size(A);
S = A;                                   % smooth along rho
S(2:nr, :)   = S(2:nr, :)   + A(1:nr-1, :);
S(1:nr-1, :) = S(1:nr-1, :) + A(2:nr, :);
T = S;                                   % smooth along theta
T(:, 2:nth)   = T(:, 2:nth)   + S(:, 1:nth-1);
T(:, 1:nth-1) = T(:, 1:nth-1) + S(:, 2:nth);

[vmax, idx] = max(T(:));
ik = ceil(idx / nr);
ir = idx - (ik - 1)*nr;

r0 = max(ir-1, 1);  r1 = min(ir+1, nr);
k0 = max(ik-1, 1);  k1 = min(ik+1, nth);
sw = 0;  sr = 0;  sk = 0;
for a = r0:r1
    for b = k0:k1
        wgt = A(a, b);
        sw = sw + wgt;
        sr = sr + wgt*a;
        sk = sk + wgt*b;
    end
end
if sw > 0
    rIdx = sr / sw;
    kIdx = sk / sw;
else
    rIdx = ir;
    kIdx = ik;
end
rho   = rIdx - off - 1;                  % index -> rho in pixels
theta = th0 + (kIdx - 1);                % index -> theta in degrees (1 deg step)
end
