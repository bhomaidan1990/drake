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
  end

  methods
    function obj = StateAlternatingDynamicsProgram(I, m, N, dt, w_fixed_array)
      obj = obj@MixedIntegerConvexProgram(true);
      obj.N = N;
      obj.dt = dt;
      obj.I = I;
      obj.m = m;
      obj.w_fixed_array = w_fixed_array;
      obj = obj.addPositionVariables();
      obj = obj.addVelocityVariables();
      obj = obj.addDynamicConstraints();
      obj = obj.addSymbolicCost(sum((obj.vars.w.symb(:) - obj.w_fixed_array(:)).^2));
    end


    function obj = addDynamicConstraints(obj)
      h = obj.dt;
      I = obj.I; %#ok
      m = obj.m; %#ok
      g = [0; 0; -9.81];
      for n = 1:(obj.N-1)
        r = obj.vars.r.symb(:, n);
        z = obj.vars.z.symb(:, n);
        v = obj.vars.v.symb(:, n);
        w = obj.vars.w.symb(:, n);

        r_next = obj.vars.r.symb(:, n+1);
        z_next = obj.vars.z.symb(:, n+1);
        w_next = obj.vars.w.symb(:, n+1);
        v_next = obj.vars.v.symb(:, n+1);
        Y_next = I*w_next; %#ok

        w_fixed = obj.w_fixed_array(:, n);
        w_next_fixed = obj.w_fixed_array(:, n+1);
        w_mid_fixed = 0.5*(w_fixed + w_next_fixed);
        A = expmap2quat(-h/2*w_mid_fixed);
        B = expmap2quat(h/2*w_mid_fixed);
        T = zeros(3, 1);
        T_next = zeros(3, 1);
        F = m*g; %#ok
        F_next = m*g; %#ok
        Y_next_desired = quatRotateVec(A, ...
                            quatRotateVec(A, I*w + h/2*T) ...
                            + h/2*quatRotateVec(B, T_next)); %#ok
        z_next_desired = quatProduct(z, expmap2quat(h*w_mid_fixed));
        v_mid = v + h/(2*m)*F; %#ok
        r_next_desired = r + h*v_mid;
        v_next_desired = v_mid + h/(2*m)*F_next; %#ok
        obj = obj.addSymbolicConstraints(r_next == r_next_desired);
        obj = obj.addSymbolicConstraints(v_next == v_next_desired);
        obj = obj.addSymbolicConstraints(z_next == z_next_desired);
        obj = obj.addSymbolicConstraints(Y_next == Y_next_desired);
      end
    end
    
    function obj = addPositionConstraint(obj, time_index, lb, ub)
      r = obj.vars.r.symb(:, time_index);
      obj = obj.addSymbolicConstraints([lb <= r <= ub]); %#ok
    end
    
    function obj = addOrientationConstraint(obj, time_index, val)
      z = obj.vars.z.symb(:, time_index);
      obj = obj.addSymbolicConstraints(z == val);
    end
    
    function obj = addPositionVariables(obj)
      obj = obj.addVariable('r', 'C', ...
        [3, obj.N], -obj.position_max, obj.position_max);
      obj = obj.addVariable('z', 'C', [4, obj.N], 0, 1);
    end

    function obj = addVelocityVariables(obj)
      obj = obj.addVariable('v', 'C', ...
        [3,obj.N], -obj.velocity_max, obj.velocity_max);
      obj = obj.addVariable('w', 'C', ...
        [3,obj.N], -obj.velocity_max, obj.velocity_max);
    end
  end
end
