classdef FlockingSim < handle
    properties
        N, r, lambda, dt, X, V
    end
    methods
        function obj = FlockingSim(N, r, lambda, dt)
            obj.N = N; obj.r = r; obj.lambda = lambda; obj.dt = dt;
            obj.X = rand(N, 2) * r * 2; 
            obj.V = randn(N, 2) * 0.5;
        end
        function psi_val = get_psi(obj, dist)
            psi_val = zeros(size(dist));
            idx = dist < obj.r;
            psi_val(idx) = (obj.r - dist(idx)) / obj.r; 
        end
        function update(obj)
            new_V = obj.V;
            for i = 1:obj.N
                diff_X = obj.X - obj.X(i, :);
                dist = sqrt(sum(diff_X.^2, 2));
                neighbor_idx = dist < obj.r;
                Ni_t = sum(neighbor_idx);
                if Ni_t > 0
                    psi_values = obj.get_psi(dist(neighbor_idx));
                    v_diff = obj.V(neighbor_idx, :) - obj.V(i, :);
                    interaction = sum(psi_values .* v_diff, 1);
                    dvdt = (obj.lambda / Ni_t) * interaction;
                    new_V(i, :) = obj.V(i, :) + dvdt * obj.dt;
                end
            end
            obj.V = new_V;
            obj.X = obj.X + obj.V * obj.dt;
        end
    end
end