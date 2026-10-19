% TEST_RUNWAY_STANDALONE  Run and evaluate the runway pipeline without Simulink.
%   Use this first: if the numbers look right here, any problem later is in
%   the Simulink wiring, not in the algorithm.
clear; clc; close all;
[L, I] = runway_run_standalone(1.0, 0.03, 20, 1);
R = runway_report(L, I, true);
