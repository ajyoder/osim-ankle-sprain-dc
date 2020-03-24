
clear

import org.opensim.modeling.*;
import org.opensim.utils.*
addpath(genpath('.\common\'))

%% Create an instance of the MocoTrack tool.
track = MocoTrack();
track.setName("ankleSprain_track_torque");

%% Build model
modelProcessor = ModelProcessor("DeMers_nominal_base_4.0.osim");
% % % Add ground reaction external loads in lieu of a ground-contact model.
% modelProcessor.append(ModOpAddExternalLoads("grf_walk.xml"));
% % % Remove all the muscles in the model's ForceSet.
modelProcessor.append(ModOpRemoveMuscles());
% % % Add CoordinateActuators to all DOFs, ignores  pelvis which already 
% % % have residual CoordinateActuators
% % % (AY) TO DO: read from an Actuators that has joint specific settings? 
modelProcessor.append(ModOpAddReserves(250));
track.setModel(modelProcessor);

%%%%(AY) TO DO: add ground contact model 

%% Load experimental data for tracking problem
tableProcessor = TableProcessor('ankleSprainsExpData\Right_SLD_mean_states.sto');
% % %(AY) already filtered, also 6HZ may be too low for this fast an event?
% Also note it contains extra muscle state info
% tableProcessor.append(TabOpLowPassFilter(6)); 


%% Adjust tracking settings
track.setStatesReference(tableProcessor)
track.set_states_global_tracking_weight(10.0);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);

track.set_initial_time(0.0);
track.set_final_time(0.150);
%(AY) in example3DWalking, T=0.84/0.05=16.8, so try T=0.150/16.8=0.009 
track.set_mesh_interval(0.009); 

study = track.initialize();
problem = study.updProblem();

% Solve! The boolean argument indicates to visualize the solution.
solution = track.solve(true);

solution.write('ankleSprain_track_torque_solution.sto');

