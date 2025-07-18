%% Run simulation and plot results
% Directory 'util' is added using callback functions from simulation.slx
% Simulation settings
model = 'dynamicInversionControl';
in = Simulink.SimulationInput(model);
in = in.setModelParameter('SimulationMode', 'accelerator');
in = in.setModelParameter('StopTime', '20');
in = in.setModelParameter('SolverType', 'Fixed-step');
in = in.setModelParameter('FixedStep', '0.01');
in = in.setModelParameter('SolverName', 'euler');

% Simulate and record results
soln = sim(in);
tout = soln.tout;
yout = soln.yout;

json = struct('tout', struct('size', size(tout), 'value', tout));
res = struct('tout', tout);

for it = reshape(cellfun(@string, yout.getElementNames), 1, [])
    val = squeeze(yout.get(it).Values.Data);

    % Convert dataset to regular struct
    res.(it) = val;

    % Convert dataset to json, each data item consisting of a size and
    % 1D data array
    sz = size(val);

    if isrow(val)
        sz = sz(end:-1:1);
    end

    json.(it) = struct('size', sz, 'value', reshape(val, [], 1));
end

JsonDump(sprintf('%s/test_controller.json', 'results'), json);

%% Plot
pld_pos = squeeze(yout.get('pld_pos').Values.Data);
uav_pos = squeeze(yout.get('uav_pos').Values.Data);

figure()
plot3(params.mission.uav_pos(1),params.mission.uav_pos(2),...
      params.mission.uav_pos(3),'dr');
hold on; grid on;
title('Slung-load UAV Trajectory');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
plot3(uav_pos(1,:),uav_pos(2,:),uav_pos(3,:),'b-','linewidth',2)
plot3(pld_pos(1,:),pld_pos(2,:),pld_pos(3,:),'k--','linewidth',1)
legend('goal','UAV','Payload')