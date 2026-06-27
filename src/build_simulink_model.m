function build_simulink_model()
%BUILD_SIMULINK_MODEL Builds an executable Simulink harness for MATLAB R2021a.

params = initialize_paper_parameters();
modelName = 'uav_landing_mpc_faa_model';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

add_block('simulink/Sources/Clock', [modelName '/Clock'], 'Position', [40 90 70 120]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [modelName '/MPC FAA Landing Step'], 'Position', [150 70 370 140]);
add_block('simulink/Sinks/To Workspace', [modelName '/landing_logs'], 'Position', [470 85 570 125]);

set_param([modelName '/MPC FAA Landing Step'], 'MATLABFcn', 'simulink_landing_step');
set_param([modelName '/landing_logs'], 'VariableName', 'simulink_landing_logs');
set_param([modelName '/landing_logs'], 'SaveFormat', 'Array');

add_line(modelName, 'Clock/1', 'MPC FAA Landing Step/1');
add_line(modelName, 'MPC FAA Landing Step/1', 'landing_logs/1');

set_param(modelName, 'StopTime', num2str(params.stateMachine.maxTime));
set_param(modelName, 'Solver', 'FixedStepDiscrete');
set_param(modelName, 'FixedStep', num2str(params.sim.dt));

annotationText = sprintf(['This Simulink harness mirrors the paper pipeline:\\n' ...
    'USV prediction -> mission state machine -> MPC-FAA trajectory generator -> linearized UAV plant.\\n' ...
    'The Interpreted MATLAB Function calls simulink_landing_step.m, which uses the same functions as run_main.m.']);
note = Simulink.Annotation(modelName, annotationText);
note.Position = [40 300 760 370];

save_system(modelName, fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
fprintf('Created Simulink model: %s\n', fullfile(fileparts(mfilename('fullpath')), [modelName '.slx']));
end
