function build_runway_model(vis, sigma)
%BUILD_RUNWAY_MODEL  Create the Simulink model "runway_landing_assist.slx".
%   build_runway_model(vis, sigma)
%     vis    visibility / contrast scale, 1 = clear (default 1.0)
%     sigma  sensor-noise std, image range 0..1 (default 0.03)
%
%   Block diagram (all MATLAB Function blocks just call the .m files, so the
%   algorithm lives in one place and can be edited without touching Simulink):
%
%   Clock ---------------+-> Scene ---> Detector ---> Tracker ---> Guidance
%   Visibility ----------+     |            |            |            |
%   NoiseSigma ----------+     |            |            |            |
%                              +------------+------------+------------+--> Logger --> RunLog, 3 Scopes
%                              +--> Overlay (red = Hough, green = tracked) --> Video Viewer
%
%   Needs: MATLAB + Simulink + Stateflow (MATLAB Function block).
%   Computer Vision Toolbox is only needed for the optional Video Viewer.
if nargin < 1, vis   = 1.0;  end
if nargin < 2, sigma = 0.03; end
p   = runway_params();
mdl = 'runway_landing_assist';
blk = @(n) [mdl '/' n];

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(p.Ts), ...
    'StopTime', '20', 'ReturnWorkspaceOutputs', 'on');

% ---- sources ------------------------------------------------------------
add_block('simulink/Sources/Digital Clock', blk('Clock'), ...
    'SampleTime', num2str(p.Ts), 'Position', [30 50 90 80]);
add_block('simulink/Sources/Constant', blk('Visibility'), ...
    'Value', num2str(vis), 'Position', [30 110 90 140]);
add_block('simulink/Sources/Constant', blk('NoiseSigma'), ...
    'Value', num2str(sigma), 'Position', [30 170 90 200]);
add_block('simulink/Sources/Constant', blk('FramePeriod'), ...
    'Value', num2str(p.Ts), 'Position', [400 250 460 280]);

% ---- MATLAB Function blocks (thin wrappers around the .m files) --------
mf = 'simulink/User-Defined Functions/MATLAB Function';

add_block(mf, blk('Scene'), 'Position', [160 40 300 200]);
setScript(blk('Scene'), { ...
    'function [img, truth] = fcn(t, vis, sigma)', ...
    '%#codegen', ...
    '[img, truth] = runway_scene(t, vis, sigma);'});

add_block(mf, blk('Detector'), 'Position', [380 40 520 140]);
setScript(blk('Detector'), { ...
    'function [z, ok, votes] = fcn(img)', ...
    '%#codegen', ...
    '[z, ok, votes] = runway_detect(img);'});

add_block(mf, blk('Tracker'), 'Position', [600 40 740 160]);
setScript(blk('Tracker'), { ...
    'function [xhat, lock] = fcn(z, ok, dt)', ...
    '%#codegen', ...
    '[xhat, lock] = runway_tracker(z, ok, dt);'});

add_block(mf, blk('Guidance'), 'Position', [820 40 960 120]);
setScript(blk('Guidance'), { ...
    'function g = fcn(xhat, lock)', ...
    '%#codegen', ...
    'g = runway_guidance(xhat, lock);'});

add_block(mf, blk('Overlay'), 'Position', [600 300 740 440]);
setScript(blk('Overlay'), { ...
    'function rgb = fcn(img, z, ok, xhat, lock)', ...
    '%#codegen', ...
    'rgb = runway_overlay(img, z, ok, xhat, lock);'});

add_block(mf, blk('Logger'), 'Position', [1040 40 1200 360]);
setScript(blk('Logger'), { ...
    'function [y, s_cb, s_err, s_dev] = fcn(t, truth, z, ok, votes, xhat, lock, g)', ...
    '%#codegen', ...
    '[y, s_cb, s_err, s_dev] = runway_logger(t, truth, z, ok, votes, xhat, lock, g);'});

% ---- sinks ---------------------------------------------------------------
add_block('simulink/Sinks/To Workspace', blk('RunLog'), 'VariableName', 'runlog', ...
    'SaveFormat', 'Array', 'SampleTime', num2str(p.Ts), 'Position', [1300 40 1380 70]);
add_block('simulink/Sinks/To Workspace', blk('ImgLog'), 'VariableName', 'imglog', ...
    'SaveFormat', 'Array', 'SampleTime', num2str(p.Ts), 'Position', [380 200 460 230]);
add_block('simulink/Sinks/Scope', blk('Scope_Centre'), 'Position', [1300 130 1340 170]);
add_block('simulink/Sinks/Scope', blk('Scope_Error'),  'Position', [1300 200 1340 240]);
add_block('simulink/Sinks/Scope', blk('Scope_Deviation'), 'Position', [1300 270 1340 310]);

% optional live video display (Computer Vision Toolbox); library path differs
% between releases, so try the likely ones and fall back to a terminator
hasViewer = false;
cands = {'visionsinks/Video Viewer', 'vision/Video Viewer', 'vipsinks/Video Viewer'};
for k = 1:numel(cands)
    try
        add_block(cands{k}, blk('Video Viewer'), 'Position', [820 330 900 380]);
        hasViewer = true;
        break
    catch
    end
end
if ~hasViewer
    add_block('simulink/Sinks/Terminator', blk('Video Viewer'), 'Position', [820 330 840 350]);
    warning(['Video Viewer block not found by path. Drag it in from Library Browser > ' ...
             'Computer Vision Toolbox > Sinks and connect it to Overlay/1.']);
end

% ---- wiring --------------------------------------------------------------
ln = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
ln('Clock/1',      'Scene/1');
ln('Visibility/1', 'Scene/2');
ln('NoiseSigma/1', 'Scene/3');

ln('Scene/1',    'Detector/1');
ln('Scene/1',    'ImgLog/1');
ln('Detector/1', 'Tracker/1');
ln('Detector/2', 'Tracker/2');
ln('FramePeriod/1', 'Tracker/3');
ln('Tracker/1',  'Guidance/1');
ln('Tracker/2',  'Guidance/2');

ln('Scene/1',    'Overlay/1');
ln('Detector/1', 'Overlay/2');
ln('Detector/2', 'Overlay/3');
ln('Tracker/1',  'Overlay/4');
ln('Tracker/2',  'Overlay/5');
ln('Overlay/1',  'Video Viewer/1');

ln('Clock/1',    'Logger/1');
ln('Scene/2',    'Logger/2');
ln('Detector/1', 'Logger/3');
ln('Detector/2', 'Logger/4');
ln('Detector/3', 'Logger/5');
ln('Tracker/1',  'Logger/6');
ln('Tracker/2',  'Logger/7');
ln('Guidance/1', 'Logger/8');

ln('Logger/1', 'RunLog/1');
ln('Logger/2', 'Scope_Centre/1');
ln('Logger/3', 'Scope_Error/1');
ln('Logger/4', 'Scope_Deviation/1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
open_system(mdl);
fprintf('Model "%s" created and saved in %s\n', mdl, pwd);
end

function setScript(blockPath, lines)
% Put source text into a MATLAB Function block (Stateflow EMChart object).
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(ch)
    error('Could not find MATLAB Function block %s', blockPath);
end
ch.Script = strjoin(lines, newline);
end
