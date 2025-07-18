%% Simulation fixture
% Variables that control simulation machinery, including rigid-body forward
% dynamics, disturbance and friction, and the (idealized) asymptotically
% stable attitude tracker
sim_.g = 9.80665;
sim_.grav = [0; 0; sim_.g];
sim_.k_thrust = 3e-6;
sim_.b_torque = 1e-7;

sim_.k_att = 1.0;
sim_.k_ang_vel = 1.0;
sim_.K_q = 100;

%% Initial Conditions
ics = JsonLoad('initialConditions.json');

%% Parameters
% Controller parameters and slung load transport system properties
params = JsonLoad('params.json');

params.derived = struct;
params.derived.sys_mass = params.uav_mass + params.pld_mass;
params.derived.uav_weight = params.uav_mass * sim_.grav;
params.derived.pld_weight = params.pld_mass * sim_.grav;
params.derived.sys_weight = params.derived.sys_mass * sim_.grav;
params.derived.cable_len_sq = params.cable_len.^2;

% params.mission.path = [ics.uav_pos, params.mission.path.'];

params.derived.inv_uav_inertia = inv(params.uav_inertia);
params.derived.uav_att_eul_0_rad = num2cell(ics.uav_att_eul(end:-1:1) ./ 57.3);
params.derived.uav_att_quat_0 = angle2quat(params.derived.uav_att_eul_0_rad{:});