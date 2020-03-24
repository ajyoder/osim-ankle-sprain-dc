function assistedSitToStand
% This file is for a workshop competition to design an assistive device for
% sit-to-stand that reduces control effort the most.
% evaluateDevice() returns the score for your device, which is the percent
% reduction in effort summed over 2 subjects, which have different mass
% properties.

% Competition rules:
% 1. Add any number of SpringGeneralizedForce components to the model.
% 2. The springs can be applied to the following coordinates:
%       hip_flexion_r
%       knee_angle_r
%       ankle_angle_r
% 3. Set Stiffness, RestLength, and Viscosity to any non-negative value.
% 4. The sum of all Stiffnesses must be less than 15.
% 5. Do not add any other types of components to the model.
% 6. Do not edit or remove any components already in the model.
%
% Information:
% 1. We started you off with two different device designs, stored in the
%    addSpringToKnee() and addSpringToAnkle() functions below.
% 2. To help with experimenting with different designs, create a separate
%    function for each design and change the argument to the evaluateDevice()
%    function below.
% 3. The unassisted solutions are cached as subject1_unassisted_solution.sto
%    and subject2_unassisted_solution.sto. If you want to re-run the unassisted
%    optimizations, delete these STO files or set cacheUnassisted to false.
% 4. To make Moco optimize the device parameters for you, do the following:
%       a. Create a MocoStudy using createStudy().
%       b. Add a MocoParameter to your problem representing
%       c. (optional) Set an initial guess for your parameter.
%          See createGuess() and setGuess() in MocoDirectCollocationSolver.
%       d. Solve the study returned from createStudy().
%       e. Get the parameter values out of the MocoSolution returned by solve().
%          See the documentation for MocoTrajectory.
%       f. Copy the parameter values into your device function, and evaluate
%          the optimized design.

import org.opensim.modeling.*;

global verbosity;
global visualize;
global cacheUnassisted;
createSubjectInfos();

% Use the verbosity variable to control console output (0 or 1).
verbosity = 1;
% Visualize the simulations after solving, and plot the solution trajectories.
visualize = 0;
% Avoid re-running the optimization for the unassisted cases.
cacheUnassisted = 0;

% Edit the argument to evaluateDevice() to any device function you create below.
% score = evaluateDevice(@addSpringToKnee);

%%%%% comment this out if you want to run the parameter optimization. 
score = evaluateDevice(@addSpringsToHipKneeAnkle);

% Use this space to perform a parameter optimization, if you wish.

% create study to do a parameter optimization
global subjectInfos;
subjects = [1,2];
% for subject = subjects
%     info = subjectInfos(subject)
% end
info1 = subjectInfos{1};

moco = createStudy(info1, @addSpringsToHipKneeAnkle);
problem = moco.updProblem();
model = problem.updModel();
model.buildSystem();

% need to have a value to optimize in the model -> needs to have the device
% set the bounds for the parameters:
% i have no idea what is realistic for the rest length and viscosity
% need to know their units
pk1 = MocoParameter('knee_stiffness','/forceset/knee_spring','stiffness',MocoBounds(0,12));
pk2 = MocoParameter('knee_restlength','/forceset/knee_spring','rest_length',MocoBounds(0,10));
pk3 = MocoParameter('knee_visco','/forceset/knee_spring','viscosity',MocoBounds(0,10));

pa1 = MocoParameter('ankle_stiffness','/forceset/ankle_spring','stiffness',MocoBounds(0,0.5));
pa2 = MocoParameter('ankle_restlength','/forceset/ankle_spring','rest_length',MocoBounds(0,10));
pa3 = MocoParameter('ankle_visco','/forceset/ankle_spring','viscosity',MocoBounds(0,10));

ph1 = MocoParameter('hip_stiffness','/forceset/hip_spring','stiffness',MocoBounds(0,2.5));
ph2 = MocoParameter('hip_restlength','/forceset/hip_spring','rest_length',MocoBounds(0,10));
ph3 = MocoParameter('hip_visco','/forceset/hip_spring','viscosity',MocoBounds(0,10));


problem.addParameter(pk1);
problem.addParameter(pk2);
problem.addParameter(pk3);

problem.addParameter(pa1);
problem.addParameter(pa2);
problem.addParameter(pa3);

problem.addParameter(ph1);
problem.addParameter(ph2);
problem.addParameter(ph3);


% somehow need to add the device to the model here and then optimize??
% name = addSpringToKnee(model, pk1);
testsolution = moco.solve()
test = testsolution.getParametersMat()


keyboard
end

% Edit these device functions or create your own!
function name = addSpringsToHipKneeAnkle(model)
name = ['knee_spring'];
import org.opensim.modeling.*;
% add the knee device
device1 = SpringGeneralizedForce('knee_angle_r');
device1.setName('knee_spring');
device1.setStiffness(11.9911);
device1.setRestLength(7.4351);
device1.setViscosity(9.9901);
model.addForce(device1);
% add the hip device
name = [name, 'hip_spring'];
device2 = SpringGeneralizedForce('hip_flexion_r');
device2.setName('hip_spring');
device2.setStiffness(0.0015);
device2.setRestLength(1.4063);
device2.setViscosity(9.9715);
model.addForce(device2);
% add the ankle device
name = [name, 'ankle_spring'];
device3 = SpringGeneralizedForce('ankle_angle_r');
device3.setName('ankle_spring');
device3.setStiffness(1.8170);
device3.setRestLength(0.0135);
device3.setViscosity(9.9888);
model.addForce(device3);
end


function name = addSpringToKnee(model)
name = 'knee_spring';
import org.opensim.modeling.*;
device = SpringGeneralizedForce('knee_angle_r');
device.setName('knee_spring');
device.setStiffness(14.9951);
device.setRestLength(4.9971);
device.setViscosity(4.9889);
model.addForce(device);
end

function name = addSpringToAnkle(model)
name = 'ankle_spring';
import org.opensim.modeling.*;
device = SpringGeneralizedForce('ankle_angle_r');
device.setName('ankle_spring');
device.setStiffness(10);
device.setRestLength(0);
device.setViscosity(5);
model.addForce(device);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DO NOT EDIT BELOW THIS LINE                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function createSubjectInfos()
% Create a global struct that specifies the mass properties of the 2 subjects.
global subjectInfos;
subjectInfos{1} = createSubjectInfo(1);
subjectInfos{2} = createSubjectInfo(2);
subjectInfos{2}.torso = 1.5;
end

function [moco] = createStudy(subjectInfo, addDeviceFunction)
% This function builds a MocoProblem for predicting a sit-to-stand motion given
% adjustments to a model (given a subject index; 1 or 2) and, optionally, a
% function for adding a device to a model.
global verbosity;
global subjectInfos;

import org.opensim.modeling.*;

moco = MocoStudy();

% Configure the problem.
problem = moco.updProblem();

% Set the model.
% subjectInfo = subjectInfos{subjectIndex};
ignoreActivationDynamics = true;
model = getMuscleDrivenModel(ignoreActivationDynamics, subjectInfo);

% Add the device to the model.
if nargin > 1
    name = addDeviceFunction(model);
    problem.setName([subjectInfo.name '_assisted_' name])
else
    problem.setName([subjectInfo.name '_unassisted'])
end
problem.setModel(model);

% Set variable bounds.
problem.setTimeBounds(0, 1);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-2, 0.5], -2, 0);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-2, 0], -2, 0);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', ...
    [-0.5, 0.7], -0.5, 0);
problem.setStateInfoPattern('/jointset/.*/speed', [], 0, 0);

% Set the cost.
problem.addCost(MocoControlCost('myeffort'));

% Configure the solver.
solver = moco.initCasADiSolver();
solver.set_dynamics_mode('implicit');
solver.set_num_mesh_points(25);
solver.set_optim_convergence_tolerance(1e-3);
solver.set_optim_constraint_tolerance(1e-3);
solver.set_optim_finite_difference_scheme('forward');
solver.set_parameters_require_initsystem(false);
if ~verbosity
    solver.set_verbosity(0);
    solver.set_optim_ipopt_print_level(1);
end

end

function [solution] = solve(subjectInfo, varargin)
% This function solves a MocoStudy created by createStudy() and may
% visualize the solution.
global visualize;

import org.opensim.modeling.*;

if nargin > 1
    moco = createStudy(subjectInfo, varargin{1});
else
    moco = createStudy(subjectInfo);
end

solution = moco.solve();

if nargin > 1
    solution.write([char(moco.getProblem().getName()) '_solution.sto']);
end

% outputPaths = StdVectorString();
% outputPaths.add('.*multiplier');
% outputTable = moco.analyze(solution, outputPaths);
% STOFileAdapter.write(outputTable, "assistedOutputs.sto");

if visualize
    moco.visualize(solution);
end

end


%% The remainder of the file contains utility functions.

function score = evaluateDevice(addDeviceFunction)
% This function runs unassisted and assisted optimizations on 2 subjects
% and prints the score for the device.

global visualize;
global cacheUnassisted;
global subjectInfos;

import org.opensim.modeling.*;

% Check the stiffness constraint.
if nargin > 1
    model = getMuscleDrivenModel(true, subjectInfos{1});
    name = addDeviceFunction(model);
    compList = model.getComponentsList();
    it = compList.begin();
    sumStiffness = 0;
    while ~it.equals(compList.end())
        if strcmp(it.getConcreteClassName(), 'SpringGeneralizedForce')
            object = model.getComponent(it.getAbsolutePathString());
            property = object.getPropertyByName('stiffness');
            stiffness = PropertyHelper.getValueDouble(property);
            sumStiffness = sumStiffness + stiffness;
        end
        it.next();
    end
    if sumStiffness > 15
        error('Sum of SpringGeneralizedForce stiffness must not exceed 15!');
    end
end

subjects = [1, 2];

percentChange = zeros(length(subjects), 1);

for subject = subjects
    info = subjectInfos{subject};
    str = sprintf('subject%i', subject);
    if cacheUnassisted && exist([str '_unassisted_solution.sto'], 'file')
        unassistedSolution = MocoTrajectory([str '_unassisted_solution.sto']);
        table = STOFileAdapter.read([str '_unassisted_solution.sto']);
        unassistedObjective = ...
            str2double(table.getTableMetaDataAsString('objective'));
    else
        unassistedSolution = solve(info);
        unassistedObjective = unassistedSolution.getObjective();
        unassistedSolution.write([str '_unassisted_solution.sto']);
    end

    assistedSolution = solve(info, addDeviceFunction);

    if visualize
        mocoPlotTrajectory(unassistedSolution, assistedSolution);
    end

    fprintf('Subject %i unassisted: %f\n', subject, unassistedObjective);
    fprintf('Subject %i assisted: %f\n', subject, assistedSolution.getObjective());

    percentChange(subject) = ...
            100.0 * (assistedSolution.getObjective() / unassistedObjective - 1);
end

score = sum(percentChange);
for subject = subjects
    fprintf('Subject %i percent change: %f\n', subject, percentChange(subject));
end
fprintf('Score (lower is better): %f\n', score);

end


function subjectInfo = createSubjectInfo(number)
subjectInfo.name = sprintf('subject%i', number);
subjectInfo.torso = 1;
subjectInfo.femur_r = 1;
subjectInfo.tibia_r = 1;
end

