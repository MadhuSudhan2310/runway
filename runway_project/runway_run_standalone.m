function [L, I] = runway_run_standalone(vis, sigma, tEnd, seed)
%RUNWAY_RUN_STANDALONE  Run the whole pipeline frame by frame, no Simulink.
%   [L, I] = runway_run_standalone(vis, sigma, tEnd, seed)
%   L : N x 24 log matrix (column map in runway_logger.m)
%   I : H x W x N stack of the generated frames
%   Calls exactly the same functions the Simulink blocks call.
if nargin < 1, vis   = 1.0;  end
if nargin < 2, sigma = 0.03; end
if nargin < 3, tEnd  = 20;   end
if nargin < 4, seed  = 1;    end
p = runway_params();
N = round(tEnd/p.Ts) + 1;

try
    rng(seed);
catch
    randn('state', seed);
end
clear functions                      % reset persistent state of the tracker

L = zeros(N, 24);
I = zeros(p.H, p.W, N);
for k = 1:N
    t = (k-1)*p.Ts;
    [img, truth]     = runway_scene(t, vis, sigma);
    [z, ok, votes]   = runway_detect(img);
    [xhat, lock]     = runway_tracker(z, ok, p.Ts);
    g                = runway_guidance(xhat, lock);
    y                = runway_logger(t, truth, z, ok, votes, xhat, lock, g);
    L(k, :)  = y.';
    I(:, :, k) = img;
end
end
