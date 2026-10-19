function [y, s_cb, s_err, s_dev] = runway_logger(t, truth, z, ok, votes, xhat, lock, g)
%RUNWAY_LOGGER  Collect truth, measurement and tracker outputs for logging/scopes.
%   y      24x1 log vector (column map below)
%   s_cb   [true; measured; tracked] runway centre at bottom row [px]
%   s_err  [measured - true; tracked - true] centre error [px]
%   s_dev  [true; estimated] lateral deviation [m]
%
%   y(1) t            y(2) cb_true      y(3) w_true      y(4) vx_true
%   y(5:8) z          y(9:10) ok        y(11:12) votes   y(13:16) xhat
%   y(17) lock        y(18) dev_est[m]  y(19) yaw_est[deg]
%   y(20) wHalf_est   y(21) vpx_est     y(22) vpy_est
%   y(23) dev_true[m] y(24) yaw_true[deg]
%#codegen
p = runway_params();
f = p.cx / tand(p.hfovDeg/2);

cbT = truth(1);  wT = truth(2);  vxT = truth(3);
devT = (p.cx - cbT) / wT * p.rwWidthM/2;
yawT = atand((vxT - p.cx)/f);

if ok(1) > 0.5 && ok(2) > 0.5
    cbM = (z(1) + z(3))/2;
else
    cbM = NaN;
end
cbK = (xhat(1) + xhat(3))/2;

y = [t; cbT; wT; vxT; z(:); ok(:); votes(:); xhat(:); lock; ...
     g(1); g(2); g(3); g(4); g(5); devT; yawT];

s_cb  = [cbT; cbM; cbK];
s_err = [cbM - cbT; cbK - cbT];
s_dev = [devT; g(1)];
end
