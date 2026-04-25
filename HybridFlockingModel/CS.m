function u_align = CS(agents)
    %% Params
    % idx         = 1; % Agent ID
    lamda       = 1; % Alignment Strength
    beta        = 0.25; % Communication Decay rate
    N           = size(agents, 2); % The number of Agents
    u_align     = zeros(3,N);
    
    %% Agent_i Information
    % pos_i       = agents(1:3, idx); % Position
    % vel_i       = agents(4:6, idx); % Velocity
    
    %% Skip calculation if there are fewer than two agents.
    if N < 2
        return;
    end
    
    %% Algorithm
    for idx = 1 : N
        % ith Position
        pos_i   = agents(1:3, idx); % Position
        % ith Velocity
        V       = agents(4,idx);
        gam_i     = agents(5,idx);
        psi_i     = agents(6,idx);
        xi_dot   = V*cos(gam_i)*cos(psi_i);
        yi_dot   = V*cos(gam_i)*sin(psi_i);
        zi_dot   = -V*sin(gam_i);
        vel_i   = [xi_dot yi_dot zi_dot]'; % Velocity
        
        sum_acc     = zeros(3,1);

        for j = 1 : N
            if j ==idx
                continue ;
            end
            % jth Velocity
            pos_j   = agents(1:3, j);
            % jth Velocity
            V       = agents(4,j);
            gam_j     = agents(5,j);
            psi_j     = agents(6,j);
            xj_dot   = V*cos(gam_j)*cos(psi_j);
            yj_dot   = V*cos(gam_j)*sin(psi_j);
            zj_dot   = -V*sin(gam_j);
            vel_j   = [xj_dot yj_dot zj_dot]'; % Velocity
            % euclideon norm
            r_ij    = norm(pos_i - pos_j);
            % communication decay rate
            com     = 1 / (1+r_ij^2)^beta;
            vel     = vel_j - vel_i;

            sum_acc = sum_acc + com*vel;    
        end
        u_align(:,idx) =  lamda/N*sum_acc;
    end
    

end