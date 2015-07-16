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
    z_fixed_array
    F_fixed_array
    M_fixed_array
    regions = struct('A', {}, 'b', {}, 'Aeq', {}, 'beq', {}, 'normal', {}, 'mu', {});
    contact_pts
  end

  methods
    function obj = StateAlternatingDynamicsProgram(I, m, N, dt, z_fixed_array, w_fixed_array, F_fixed_array, M_fixed_array, has_symbolic)
      obj = obj@MixedIntegerConvexProgram(has_symbolic);
      obj.N = N;
      obj.dt = dt;
      obj.I = I;
      obj.m = m;
      obj.w_fixed_array = w_fixed_array;
      obj.z_fixed_array = z_fixed_array;
      obj.F_fixed_array = F_fixed_array;
      obj.M_fixed_array = M_fixed_array;
      obj = obj.addPositionVariables();
      obj = obj.addVelocityVariables();
    end
    
    function obj = addDefaultConstraintsAndCosts(obj)
      
      for i = 1:size(obj.contact_pts, 2)
        % Add binary variables for contact region assignment
        obj = obj.addVariable(sprintf('R%d',i), 'B', [numel(obj.regions), obj.N], 0, 1);
        
        % Add continuous variables for contact forces and moments
        obj = obj.addVariable(sprintf('F%d',i), 'C', [3, obj.N], ...
          -obj.force_max, obj.force_max);
        
        obj = obj.addVariable(sprintf('M%d',i), 'C', [3, obj.N], ...
          -obj.force_max, obj.force_max);
      end
      obj = obj.addTranslationalDynamicConstraints(true);
      obj = obj.addRotationalDynamicConstraints(true);
      obj = obj.addContactPointConstraints(true);
      obj = obj.addContactForceConstraints(true);
      if obj.has_symbolic
        obj = obj.addSymbolicCost(sum((obj.vars.w.symb(:) - obj.w_fixed_array(:)).^2));
        obj = obj.addSymbolicCost(sum((obj.vars.z.symb(:) - obj.z_fixed_array(:)).^2));
        for i = 1:size(obj.contact_pts,2)
          obj = obj.addSymbolicCost(sum(sum((obj.vars.(sprintf('F%d',i)).symb - obj.F_fixed_array(:,:,i)).^2)));
          obj = obj.addSymbolicCost(sum(sum((obj.vars.(sprintf('M%d',i)).symb - obj.M_fixed_array(:,:,i)).^2)));
        end
      else
        Q = zeros(obj.nv);
        c = zeros(obj.nv, 1);
        Q(obj.vars.w.i(:), obj.vars.w.i(:)) = eye(numel(obj.w_fixed_array));
        c(obj.vars.w.i(:)) = -2*obj.w_fixed_array(:);
        Q(obj.vars.z.i(:), obj.vars.z.i(:)) = eye(numel(obj.z_fixed_array));
        c(obj.vars.z.i(:)) = -2*obj.z_fixed_array(:);
        alpha = sum(obj.w_fixed_array(:).^2) + sum(obj.z_fixed_array(:).^2);
        obj = obj.addCost(Q, c, alpha);
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
          for i = 1:size(obj.contact_pts,2)
            Fname = sprintf('F%d', i);
            F = F + obj.vars.(Fname).symb(:, n);
            F_next = F_next + obj.vars.(Fname).symb(:, n+1);
          end
          
          v_mid = v + h/(2*m)*F; %#ok
          r_next_desired = r + h*v_mid;
          v_next_desired = v_mid + h/(2*m)*F_next; %#ok
          obj = obj.addSymbolicConstraints(r_next == r_next_desired);
          obj = obj.addSymbolicConstraints(v_next == v_next_desired);
%           obj = obj.addSymbolicCost(norm(r_next - r_next_desired));
%           obj = obj.addSymbolicCost(norm(v_next - v_next_desired));
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
          
          z_fixed = obj.z_fixed_array(:, n);
          z_next_fixed = obj.z_fixed_array(:, n+1);
          w_fixed = obj.w_fixed_array(:, n);
          w_next_fixed = obj.w_fixed_array(:, n+1);
          w_mid_fixed = 0.5*(w_fixed + w_next_fixed);
          A = expmap2quat(-h/2*w_mid_fixed);
          B = expmap2quat(h/2*w_mid_fixed);
          T = zeros(3, 1);
          T_next = zeros(3, 1);
          for i = 1:size(obj.contact_pts,2)
            M_fixed = obj.M_fixed_array(:, :, i);
            F_fixed = obj.F_fixed_array(:, :, i);
            Fname = sprintf('F%d', i);
            Mname = sprintf('M%d', i); 
            T = T + quatRotateVec(quatConjugate(z_fixed), obj.vars.(Mname).symb(:, n)) ...
                  + cross(obj.contact_pts(:, i), quatRotateVec(quatConjugate(z_fixed), obj.vars.(Fname).symb(:, n)));
            T_next = T_next + quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Mname).symb(:, n+1)) ...
                  + cross(obj.contact_pts(:, i), quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Fname).symb(:, n+1)));
            T = T + 0.5*quatRotateVec(quatConjugate(z_fixed), obj.vars.(Mname).symb(:, n)) + 0.5*obj.quatRotateVec2(quatConjugate(z), quatConjugate(z_fixed), M_fixed(:, n)) ...
                  + cross(obj.contact_pts(:, i), 0.5*quatRotateVec(quatConjugate(z_fixed), obj.vars.(Fname).symb(:, n)) + 0.5*obj.quatRotateVec2(quatConjugate(z), quatConjugate(z_fixed), F_fixed(:, n)));
            T_next = T_next + 0.5*quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Mname).symb(:, n+1)) + 0.5*obj.quatRotateVec2(quatConjugate(z_next), quatConjugate(z_next_fixed), M_fixed(:, n+1)) ...
                  + cross(obj.contact_pts(:, i), 0.5*quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Fname).symb(:, n+1)) + 0.5*obj.quatRotateVec2(quatConjugate(z_next), quatConjugate(z_next_fixed), F_fixed(:, n+1)));
          end
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
          z_fixed = obj.z_fixed_array(:, n);
          z_next_fixed = obj.z_fixed_array(:, n+1);
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

    function obj = addContactPointConstraints(obj, use_symbolic)
      if use_symbolic
        r = obj.vars.r.symb;
        z = obj.vars.z.symb;
        z_fixed = obj.z_fixed_array;
        for i = 1:size(obj.contact_pts, 2)
          R = obj.vars.(sprintf('R%d',i)).symb;
          obj = obj.addSymbolicConstraints(sum(R,1) == 1);
          for n = 1:obj.N
            p = obj.quatRotateVec2(z(:,n), z_fixed(:,n), obj.contact_pts(:,i)) + r(:,n);
            for j = 1:numel(obj.regions)
              if ~isempty(obj.regions(j).A)
                obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                  obj.regions(j).A*p <= obj.regions(j).b));
              end
              if ~isempty(obj.regions(j).Aeq)
                obj = obj.addSymbolicConstraints(implies(R(j, n), ...
                  obj.regions(j).Aeq*p == obj.regions(j).beq));
              end
            end
          end
        end
      else
        error('not implemented')
      end
    end
    
    function obj = addContactForceConstraints(obj, use_symbolic)
      if use_symbolic
        r = obj.vars.r.symb;
        z = obj.vars.z.symb;            
        v = obj.vars.v.symb;            
        w = obj.vars.w.symb;

        z_fixed = obj.z_fixed_array;
        for i = 1:size(obj.contact_pts, 2)
          R = obj.vars.(sprintf('R%d',i)).symb;
          for n = 1:obj.N-1
            p = obj.quatRotateVec2(z(:,n), z_fixed(:,n), obj.contact_pts(:,i)) + r(:,n);
            p_next = obj.quatRotateVec2(z(:,n+1), z_fixed(:,n+1), obj.contact_pts(:,i)) + r(:,n+1);
            pd = v(:,n) + cross(w(:,n), obj.contact_pts(:,i));
            F = obj.vars.(sprintf('F%d', i)).symb(:, n);
            M = obj.vars.(sprintf('M%d', i)).symb(:, n);
            for j = 1:numel(obj.regions)
              normal = obj.regions(j).normal;
              mu = obj.regions(j).mu;
              if isempty(normal)
                obj = obj.addSymbolicConstraints(implies(R(j,n+1), F == 0));
                obj = obj.addSymbolicConstraints(implies(R(j,n+1), M == 0));
              else
                F_normal = dot(F, normal)*normal;
                F_tan = F - F_normal;
                pd_normal = dot(pd, normal)*normal;
                pd_tan = pd - pd_normal;
                % HACK
                obj = obj.addSymbolicConstraints(cone(F_tan, mu*dot(F, obj.regions(j).normal)));
                obj = obj.addSymbolicConstraints(implies(R(j,n+1), M == 0));
                % END_HACK
%                 obj = obj.addSymbolicConstraints(implies(R(j,n) + R(j,n+1) == 2, pd == 0));
                obj = obj.addSymbolicConstraints(implies(R(j,n) + R(j,n+1) == 2, p == p_next));
%                 obj = obj.addSymbolicConstraints(implies(R(j,n), p == p_next));
%                 obj = obj.addSymbolicConstraints(implies(R(j,n), pd_tan == 0));
              end
            end
          end
        end
      else
        error('not implemented')
      end
    end
    
    function obj = addRegion(obj, A, b, Aeq, beq, normal, mu)
      j = numel(obj.regions)+1;
      if ~isempty(A)
        obj.regions(j).A = A;
      end
      if ~isempty(b)
        obj.regions(j).b = b;
      end
      if ~isempty(Aeq)
        obj.regions(j).Aeq = Aeq;
      end
      if ~isempty(beq)
        obj.regions(j).beq = beq;
      end
      if ~isempty(normal)
        obj.regions(j).normal = normal;
      end
      if ~isempty(mu)
        obj.regions(j).mu = mu;
      end
    end
    
    function obj = addContactPoint(obj, p)
      obj.contact_pts = [obj.contact_pts, p];
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
  
  methods (Static)
    function v_rotated = quatRotateVec2(q1, q2, v)
      rotmat = [1 - 2*q1(3)*q2(3) - 2*q1(4)*q2(4), 2*q1(2)*q2(3) - 2*q1(4)*q2(1),     2*q1(2)*q2(4) + 2*q1(3)*q2(1); ...
        2*q2(2)*q1(3)+2*q2(4)*q1(1),       1 - 2*q1(2)*q2(2) - 2*q1(4)*q2(4), 2*q1(3)*q2(4) - 2*q1(2)*q2(1); ...
        2*q2(2)*q1(4) - 2*q2(3)*q1(1),     2*q2(3)*q1(4) + 2*q2(2)*q1(1)      1 - 2*q1(2)*q2(2) - 2*q1(3)*q2(3)];
      v_rotated = rotmat*v;
    end
  end
end
