function [xhat, lock] = runway_tracker(z, ok, dt)
%RUNWAY_TRACKER  Kalman-filter tracking of the two runway edges.
%   [xhat, lock] = runway_tracker(z, ok, dt)
%     z     [xLb; xLt; xRb; xRt]  Hough measurement (pixels)
%     ok    [okL; okR]            measurement valid flags
%     dt    frame period [s]
%     xhat  filtered [xLb; xLt; xRb; xRt]
%     lock  1 when both edges are being tracked reliably
%
%   Four independent constant-velocity filters (position + velocity each).
%   Each edge is gated on innovation: if either of its two points jumps
%   too far from the prediction the whole edge measurement is rejected and
%   the filter coasts on its prediction (this rejects clutter and brief
%   dropouts).  The lock flag drops after p.maxMiss consecutive rejections.
%#codegen
p = runway_params();
persistent x P nupd miss
if isempty(x)
    x    = [p.prior, zeros(4, 1)];           % [pos vel] for each channel
    P    = repmat([40^2, 0, 10^2], 4, 1);    % [p11 p12 p22] for each channel
    nupd = zeros(2, 1);
    miss = zeros(2, 1);
end
q = p.sigmaAccel^2;
R = p.sigmaMeas^2;

% ---- predict -------------------------------------------------------------
xp = zeros(4, 1);
vp = zeros(4, 1);
Pp = zeros(4, 3);
for i = 1:4
    xp(i)   = x(i, 1) + dt*x(i, 2);
    vp(i)   = x(i, 2);
    Pp(i,1) = P(i,1) + 2*dt*P(i,2) + dt^2*P(i,3) + q*dt^4/4;
    Pp(i,2) = P(i,2) + dt*P(i,3) + q*dt^3/2;
    Pp(i,3) = P(i,3) + q*dt^2;
end

% ---- gate and update, one runway edge at a time --------------------------
for s = 1:2
    i1 = 2*s - 1;
    i2 = 2*s;
    accept = ok(s) > 0.5;
    if accept
        for i = i1:i2
            S    = Pp(i,1) + R;
            gate = max(p.gateMin, p.gateSigma*sqrt(S));
            if abs(z(i) - xp(i)) > gate
                accept = false;
            end
        end
    end
    if accept
        for i = i1:i2
            S   = Pp(i,1) + R;
            K1  = Pp(i,1) / S;
            K2  = Pp(i,2) / S;
            inn = z(i) - xp(i);
            xp(i)   = xp(i) + K1*inn;
            vp(i)   = vp(i) + K2*inn;
            Pp(i,3) = Pp(i,3) - K2*Pp(i,2);
            Pp(i,2) = (1 - K1)*Pp(i,2);
            Pp(i,1) = (1 - K1)*Pp(i,1);
        end
        nupd(s) = nupd(s) + 1;
        miss(s) = 0;
    else
        miss(s) = miss(s) + 1;
    end
end

x = [xp, vp];
P = Pp;
xhat = xp;
lock = double(all(nupd >= p.minUpdates) && all(miss <= p.maxMiss));
end
