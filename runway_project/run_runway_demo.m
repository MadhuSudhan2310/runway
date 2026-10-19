function out = run_runway_demo(vis, sigma)
%RUN_RUNWAY_DEMO  Build the Simulink model, simulate it, and report results.
%   out = run_runway_demo(vis, sigma)
%   Try run_runway_demo(1.0) for clear weather and run_runway_demo(0.4) for haze.
if nargin < 1, vis   = 1.0;  end
if nargin < 2, sigma = 0.03; end

build_runway_model(vis, sigma);
out = sim('runway_landing_assist');

L = squeeze(out.get('runlog'));
if size(L, 1) ~= numel(out.tout), L = L.'; end       % make it N x 24
I = squeeze(out.get('imglog'));                      % H x W x N
runway_report(L, I, true);
end
