options.floating = 'quat';
options.use_new_kinsol = true;
urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'FallingBrick.urdf');
rbm = RigidBodyManipulator(urdf, options);
v = rbm.constructVisualizer();
I = diag([1;6;3]);
m = 1;
N = 20;
M = 1000;
dt = 1/N;
r0 = [0; 0; 1];
z0 = rpy2quat([pi/3; 0; 0]);
w0 = 0*[0; 1; 0];
v0 = [0; 0; 0];
w_fixed_array = 0*rand(3, N);
z_fixed_array = repmat(z0, 1, N);
M_fixed_array = zeros(3, N, 8);
F_fixed_array = zeros(3, N, 8);
lam = 0.5;
tol = 1e-6;

total_solvertime = 0;
total_time = tic;
for i = 1:M
  prog = StateAlternatingDynamicsProgram(I, m, N, dt, z_fixed_array, w_fixed_array, F_fixed_array, M_fixed_array, true);
  prog.contact_pts = rbm.getTerrainContactPoints().pts;
  prog = prog.addRegion([0, 0, -1], 0, [], [], [], []);
  prog = prog.addRegion([], [], [0, 0, 1], 0, [0; 0; 1], 1);
  prog = prog.addDefaultConstraintsAndCosts();
  %prog = prog.addOrientationConstraint(1, [1; 0; 0; 0]);
  prog = prog.addOrientationConstraint(1, z0);
  prog = prog.addPositionConstraint(1, r0, r0);
  prog = prog.addAngularVelocityConstraint(1, w0, w0);
  prog = prog.addVelocityConstraint(1, v0, v0);
  [prog, solvertime, objval] = prog.solve();
  delta_norm = sum((w_fixed_array(:) - prog.vars.w.value(:)).^2) ...
                + sum((z_fixed_array(:) - prog.vars.z.value(:)).^2);
  total_solvertime = total_solvertime + solvertime;
  w_fixed_array = (1-lam)*w_fixed_array + lam*prog.vars.w.value;
  z_fixed_array = (1-lam)*z_fixed_array + lam*prog.vars.z.value;
  for j = 1:size(prog.contact_pts,2)
    F_fixed_array(:, :, j) = (1-lam)*F_fixed_array(:, :, j) + lam*prog.vars.(sprintf('F%d',j)).value;
    M_fixed_array(:, :, j) = (1-lam)*M_fixed_array(:, :, j) + lam*prog.vars.(sprintf('M%d',j)).value;
  end
  fprintf('Objective Value: %f\tNorm squared of delta: %f\n', objval, delta_norm);
  t = cumsum([0, repmat(prog.dt, [1, prog.N-1])]);
  position_traj = PPTrajectory(foh(t, [prog.vars.r.value; prog.vars.z.value]));
  position_traj = position_traj.setOutputFrame(rbm.getPositionFrame());
  v.playback(position_traj)
  if objval < tol && delta_norm < tol, break; end
  %   if delta_norm < tol, break; end
end
toc(total_time);
%%
v.draw(0, position_traj.eval(t(end)))
%%
% options.floating = true;
% rbm = RigidBodyManipulator(urdf, options);
% v = rbm.constructVisualizer();
% t = cumsum(repmat(prog.dt, [1, prog.N]));
% rpy_value = zeros(3, size(prog.vars.z.value, 2));
% for i = 1:size(rpy_value, 2)
%   rpy_value(:, i) = quat2rpy(prog.vars.z.value(:, i));
% end
% position_traj = PPTrajectory(foh(t, [prog.vars.r.value; rpy_value]));
% % position_traj = PPTrajectory(foh(t, [prog.vars.r.value; prog.vars.z.value]));
% position_traj = position_traj.setOutputFrame(rbm.getPositionFrame());
