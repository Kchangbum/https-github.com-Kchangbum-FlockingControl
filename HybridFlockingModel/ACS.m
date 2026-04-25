function u_acs = ACS(agents)

    %% Agent parameters
    % idx     = 1;
    N       = size(agents,2);
    % pos_i   = agents(1:3, idx);
    % vel_i   = agents(4:6, idx);

    %% damping parameters
    K1      = 0.32; % Bonding gain 1
    % damping = zeros(3,N);

    %% spring parameters
    R       = 75;
    K2      = 0.05; % Bonding gain 2
    % spring  = zeros(3,N);
    u_acs = zeros(3,N);

    %% CS model algorithm
    lamda       = 1; % Alignment Strength
    beta        = 0.25; % Communication Decay rate

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

        damping = zeros(3,1);
        spring  = zeros(3,1); 
        cs      = zeros(3,1);

        for j = 1 : N
            if j == idx, continue; end
            % jth Velocity
            pos_j   = agents(1:3, j);
            % jth Velocity
            V       = agents(4,j);
            gam_j   = agents(5,j);
            psi_j   = agents(6,j);
            xj_dot  = V*cos(gam_j)*cos(psi_j);
            yj_dot  = V*cos(gam_j)*sin(psi_j);
            zj_dot  = -V*sin(gam_j);
            vel_j   = [xj_dot yj_dot zj_dot]'; % Velocity

            pos_ij  = pos_i - pos_j;
            pos_ji  = pos_j - pos_i;
            vel_ij  = vel_i - vel_j;

            r_ij    = norm(pos_ij) + 0.1;
    
            if r_ij < 1e-3, r_ij = 1e-3; end

            % communication decay rate
            com     = 1 / (1+r_ij^2)^beta;
            vel     = vel_j - vel_i;

            cs = cs + com*vel; 
            
            % Inner Product
            dot_val = dot(vel_ij, pos_ij);
            % Damping term
            damping = damping + (K1 / (2 * r_ij^2)) * dot_val * (pos_j - pos_i);
            
            % Spring term
            spring  = spring +  (K2 / (2 * r_ij)) * (r_ij - 2*R) * (pos_j - pos_i);
        end
        u_acs(:,idx) = lamda/N*cs + damping + spring;
    end


end