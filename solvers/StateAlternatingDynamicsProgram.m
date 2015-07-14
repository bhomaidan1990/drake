classdef StateAlternatingDynamicsProgram < MixedIntegerConvexProgram
  properties
    I
    m
    N = 2
    position_max = 1e3
    velocity_max = 1e3
    force_max = 1e2
    dt = 1e-3
    w_fixed_array
    regions
  end

  methods
    function obj = StateAlternatingDynamicsProgram(I, m, N, dt, w_fixed_array,has_symbolic)
      obj = obj@MixedIntegerConvexProgram(has_symbolic);
      obj.N = N;
      obj.dt = dt;
      obj.I = I;
      obj.m = m;
      obj.w_fixed_array = w_fixed_array;
      obj = obj.addPositionVariables();
      obj = obj.addVelocityVariables();
      obj = obj.addTranslationalDynamicConstraints(false);
      obj = obj.addRotationalDynamicConstraints(false);
      if obj.has_symbolic
        obj = obj.addSymbolicCost(sum((obj.vars.w.symb(:) - obj.w_fixed_array(:)).^2));
      else
        Q = zeros(obj.nv);
        c = zeros(obj.nv, 1);
        Q(obj.vars.w.i(:), obj.vars.w.i(:)) = eye(numel(obj.w_fixed_array));
        c(obj.vars.w.i(:)) = -2*obj.w_fixed_array(:);
        obj = obj.addCost(Q, c, sum(obj.w_fixed_array(:).^2));
      end
    end

    function obj = addTranslationalDynamicConstraints(obj, use_symbolic)
      if nargin < 2, use_symbolic = true; end
      h = obj.dt;
      I = obj.I; %#ok
      m = obj.m; %#ok
      g = [0; 0; -9.81];
      if obj.has_symbolic && use_symbolic
        for n = 1:(obj.N-1)
          r = obj.vars.r.symb(:, n);
          v = obj.vars.v.symb(:, n);

          r_next = obj.vars.r.symb(:, n+1);
          v_next = obj.vars.v.symb(:, n+1);

          F = m*g; %#ok
          F_next = m*g; %#ok
          v_mid = v + h/(2*m)*F; %#ok
          r_next_desired = r + h*v_mid;
          v_next_desired = v_mid + h/(2*m)*F_next; %#ok
          obj = obj.addSymbolicConstraints(r_next == r_next_desired);
          obj = obj.addSymbolicConstraints(v_next == v_next_desired);
        end
      else
        for n = 1:(obj.N-1)
          F = m*g; %#ok
          F_next = m*g; %#ok
          Aeq = zeros(6, obj.nv);
          beq = zeros(6, 1);

          % Position
          % r_next - r - h*v = h^2/(2*m)*F
          beq(1:3) = h^2/(2*m)*F; %#ok
          Aeq(sub2ind(size(Aeq), 1:3, obj.vars.r.i(:,n+1)')) = 1;
          Aeq(sub2ind(size(Aeq), 1:3, obj.vars.r.i(:,n)')) = -1;
          Aeq(sub2ind(size(Aeq), 1:3, obj.vars.v.i(:,n)')) = -h;

          % Velocity
          % v_next - v = h/(2*m)*(F + F_next)
          Aeq(sub2ind(size(Aeq), 4:6, obj.vars.v.i(:,n+1)')) = 1;
          Aeq(sub2ind(size(Aeq), 4:6, obj.vars.v.i(:,n)')) = -1;
          beq(sub2ind(size(Aeq), 4:6)) = h/(2*m)*(F+F_next); %#ok

          obj = obj.addLinearConstraints([], [], Aeq, beq);
        end
      end
    end

    function obj = addRotationalDynamicConstraints(obj, use_symbolic)
      if nargin < 2, use_symbolic = true; end
      h = obj.dt;
      I = obj.I; %#ok
      m = obj.m; %#ok
      g = [0; 0; -9.81];
      if obj.has_symbolic && use_symbolic
        for n = 1:(obj.N-1)
          z = obj.vars.z.symb(:, n);
          w = obj.vars.w.symb(:, n);

          z_next = obj.vars.z.symb(:, n+1);
          w_next = obj.vars.w.symb(:, n+1);
          Y_next = I*w_next; %#ok

          w_fixed = obj.w_fixed_array(:, n);
          w_next_fixed = obj.w_fixed_array(:, n+1);
          w_mid_fixed = 0.5*(w_fixed + w_next_fixed);
          A = expmap2quat(-h/2*w_mid_fixed);
          B = expmap2quat(h/2*w_mid_fixed);
          T = zeros(3, 1);
          T_next = zeros(3, 1);
          Y_next_desired = quatRotateVec(A, ...
            quatRotateVec(A, I*w + h/2*T) ...
            + h/2*quatRotateVec(B, T_next)); %#ok
          z_next_desired = quatProduct(z, expmap2quat(h*w_mid_fixed));
          obj = obj.addSymbolicConstraints(z_next == z_next_desired);
          obj = obj.addSymbolicConstraints(Y_next == Y_next_desired);
        end
      else
        for n = 1:(obj.N-1)
          T = zeros(3, 1);
          T_next = zeros(3, 1);
          w_fixed = obj.w_fixed_array(:, n);
          w_next_fixed = obj.w_fixed_array(:, n+1);
          w_mid_fixed = 0.5*(w_fixed + w_next_fixed);

          Aeq = zeros(7, obj.nv);
          beq = zeros(7, 1);

          % quatProduct(z, expmap(h*w_mid_fixed)) - z_next= 0
          deltaZ = expmap2quat(h*w_mid_fixed);
          Aeq(1:4, obj.vars.z.i(:,n+1)) = -eye(4);
          Aeq(1, obj.vars.z.i(1,n)) = deltaZ(1);
          Aeq(1, obj.vars.z.i(2:4,n)) = -deltaZ(2:4);
          Aeq(2, obj.vars.z.i(1:3,n)) = deltaZ([2,1,4]);
          Aeq(2, obj.vars.z.i(4,n)) = -deltaZ(3);
          Aeq(3, obj.vars.z.i([1,3,4],n)) = deltaZ([3,1,2]);
          Aeq(3, obj.vars.z.i(2,n)) = -deltaZ(4);
          Aeq(4, obj.vars.z.i([1,2,4],n)) = deltaZ([4,3,1]);
          Aeq(4, obj.vars.z.i(3,n)) = -deltaZ(2);

          % R(A)'*I*w_next - R(A)*I*w = h/2*(R(A)*T + R(B)*T_next)
          RA = quat2rotmat(expmap2quat(-h/2*w_mid_fixed));
          RB = quat2rotmat(expmap2quat(h/2*w_mid_fixed));
          Aeq(5:7, obj.vars.w.i(:,n+1)) = RA'*I; %#ok
          Aeq(5:7, obj.vars.w.i(:,n)) = -RA*I; %#ok
          beq(5:7) = h/2*(RA*T + RB*T_next);
          obj = obj.addLinearConstraints([], [], Aeq, beq);
        end
      end
    end

    function obj = addRegion(obj, Ar, br, Ar_eq, br_eq, normal, mu)
    end
    
    function angular_momentum = extractAngularMomentum(obj)
      w = obj.vars.w.value;
      z = obj.vars.z.value;
      angular_momentum = 0*w;
      for n = 1:obj.N
        angular_momentum(:, n) = quatRotateVec(z(:,n), obj.I*w(:,n));
      end
    end
    
    function obj = addPositionConstraint(obj, time_index, lb, ub)
      obj.vars.r.lb(:, time_index) = lb;
      obj.vars.r.ub(:, time_index) = ub;
    end
    
    function obj = addOrientationConstraint(obj, time_index, val)
      obj.vars.z.lb(:, time_index) = val;
      obj.vars.z.ub(:, time_index) = val;
    end
    
    function obj = addVelocityConstraint(obj, time_index, lb, ub)
      obj.vars.v.lb(:, time_index) = lb;
      obj.vars.v.ub(:, time_index) = ub;
    end
    
    function obj = addAngularVelocityConstraint(obj, time_index, lb, ub)
      obj.vars.w.lb(:, time_index) = lb;
      obj.vars.w.ub(:, time_index) = ub;
    end
    
    function obj = addPositionVariables(obj)
      obj = obj.addVariable('r', 'C', ...
        [3, obj.N], -obj.position_max, obj.position_max);
      obj = obj.addVariable('z', 'C', [4, obj.N], -1, 1);
    end

    function obj = addVelocityVariables(obj)
      obj = obj.addVariable('v', 'C', ...
        [3,obj.N], -obj.velocity_max, obj.velocity_max);
      obj = obj.addVariable('w', 'C', ...
        [3,obj.N], -obj.velocity_max, obj.velocity_max);
    end
  end
end
