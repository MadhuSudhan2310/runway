function R = runway_report(L, I, doPlot)
%RUNWAY_REPORT  Metrics and plots from a run log (Simulink or standalone).
%   R = runway_report(L, I, doPlot)
%   L : N x 24 log matrix (see runway_logger.m),  I : H x W x N frames
if nargin < 3, doPlot = true; end
t    = L(:,1);   cbT = L(:,2);   wT = L(:,3);   vxT = L(:,4);
z    = L(:,5:8); ok  = L(:,9:10); xh = L(:,13:16); lock = L(:,17) > 0.5;
devE = L(:,18);  yawE = L(:,19);  devT = L(:,23);  yawT = L(:,24);

both    = ok(:,1) > 0.5 & ok(:,2) > 0.5;      % both edges detected this frame
settled = t >= 1.0;                            % ignore start-up transient
cbM = (z(:,1) + z(:,3))/2;
cbK = (xh(:,1) + xh(:,3))/2;
wK  = (xh(:,3) - xh(:,1))/2;

wM  = (z(:,3) - z(:,1))/2;
% a raw detection is "correct" if centre is within 5 px and half-width within 8 px
good = both & abs(cbM - cbT) < 5 & abs(wM - wT) < 8;

m1 = both & settled;
m2 = settled & lock;
R.detRate    = 100*mean(both);
R.goodRate   = 100*mean(good);
R.lockRate   = 100*mean(lock);
R.rmseMeas   = sqrt(mean((cbM(m1) - cbT(m1)).^2));
R.rmseTrack  = sqrt(mean((cbK(settled) - cbT(settled)).^2));
R.rmseWidth  = sqrt(mean((wK(settled) - wT(settled)).^2));
R.rmseDevM   = sqrt(mean((devE(m2) - devT(m2)).^2));
R.rmseYawDeg = sqrt(mean((yawE(m2) - yawT(m2)).^2));

fprintf('\n=== Runway detection & tracking: results ===\n');
fprintf(' frames                          : %d  (%.1f s)\n', numel(t), t(end));
fprintf(' both edges passed vote threshold: %.1f %%\n', R.detRate);
fprintf(' raw detection correct (<5 px)   : %.1f %%\n', R.goodRate);
fprintf(' tracker locked                  : %.1f %%\n', R.lockRate);
fprintf(' centre error, raw measurement   : %.2f px RMSE\n', R.rmseMeas);
fprintf(' centre error, Kalman tracked    : %.2f px RMSE\n', R.rmseTrack);
fprintf(' half-width error, tracked       : %.2f px RMSE\n', R.rmseWidth);
fprintf(' lateral deviation error         : %.2f m RMSE\n', R.rmseDevM);
fprintf(' yaw (vanishing point) error     : %.2f deg RMSE\n\n', R.rmseYawDeg);

if ~doPlot, return; end

figure('Name', 'Runway tracking results', 'Position', [60 60 1000 700]);
subplot(2,2,1);
plot(t, cbT, 'k-', t, cbM, 'r.', t, cbK, 'g-', 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('centre at bottom row [px]');
legend('truth', 'raw Hough', 'Kalman', 'Location', 'best');
title('Runway centre line');

subplot(2,2,2);
plot(t, cbM - cbT, 'r.', t, cbK - cbT, 'g-', 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('error [px]');
legend('raw Hough', 'Kalman', 'Location', 'best');
title(sprintf('Centre error: raw %.2f px, tracked %.2f px RMSE', R.rmseMeas, R.rmseTrack));

subplot(2,2,3);
plot(t, devT, 'k-', t, devE, 'b-', 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('lateral deviation [m]');
legend('truth', 'estimated', 'Location', 'best');
title('Lateral deviation (+ = right of centre-line)');

subplot(2,2,4);
plot(t, yawT, 'k-', t, yawE, 'm-', 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('yaw cue [deg]');
legend('truth', 'estimated', 'Location', 'best');
title('Yaw from vanishing point');

if nargin >= 2 && ~isempty(I)
    figure('Name', 'Sample frames', 'Position', [100 100 1000 300]);
    pick = round(linspace(1, size(I,3), 4));
    for q = 1:4
        k = pick(q);
        rgb = runway_overlay(I(:,:,k), z(k,:).', ok(k,:).', xh(k,:).', double(lock(k)));
        subplot(1,4,q); image(rgb); axis image off;
        title(sprintf('t = %.1f s', t(k)));
    end
end
end
