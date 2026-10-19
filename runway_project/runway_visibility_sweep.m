function S = runway_visibility_sweep(vis, sigma)
%RUNWAY_VISIBILITY_SWEEP  Detection rate and error versus visibility (haze).
%   S = runway_visibility_sweep(vis, sigma)
%   Produces the "robustness" result for the report: how raw Hough detection
%   and the Kalman-tracked output degrade as contrast falls.
if nargin < 1, vis   = [1.0 0.8 0.6 0.45 0.35 0.25]; end
if nargin < 2, sigma = 0.03; end
n = numel(vis);
S.vis = vis;
S.detRate = zeros(1, n);  S.lockRate = zeros(1, n);
S.rmseMeas = zeros(1, n); S.rmseTrack = zeros(1, n);
for k = 1:n
    [L, ~] = runway_run_standalone(vis(k), sigma, 20, 1);
    R = runway_report(L, [], false);
    S.detRate(k)   = R.detRate;
    S.lockRate(k)  = R.lockRate;
    S.rmseMeas(k)  = R.rmseMeas;
    S.rmseTrack(k) = R.rmseTrack;
end
fprintf('\n vis    det%%   lock%%   raw[px]  tracked[px]\n');
for k = 1:n
    fprintf(' %.2f  %6.1f  %6.1f  %7.2f  %9.2f\n', vis(k), S.detRate(k), ...
        S.lockRate(k), S.rmseMeas(k), S.rmseTrack(k));
end
figure('Name', 'Visibility sweep');
subplot(1,2,1); plot(vis, S.detRate, 'o-', vis, S.lockRate, 's-', 'LineWidth', 1.2);
set(gca, 'XDir', 'reverse'); grid on; xlabel('visibility (contrast scale)'); ylabel('%');
legend('both edges detected', 'tracker locked', 'Location', 'best');
subplot(1,2,2); plot(vis, S.rmseMeas, 'o-', vis, S.rmseTrack, 's-', 'LineWidth', 1.2);
set(gca, 'XDir', 'reverse'); grid on; xlabel('visibility (contrast scale)'); ylabel('centre error RMSE [px]');
legend('raw Hough', 'Kalman', 'Location', 'best');
end
