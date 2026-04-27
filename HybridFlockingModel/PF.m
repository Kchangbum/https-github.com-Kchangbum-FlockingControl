function u_pf = PF(agents)

    %% params
    % idx = 1;
    N = size(agents,2);
    alp = 5*sqrt(2); % Potential Strength
    % Bell Function Constant (a, b, c)
    a   = 5*sqrt(2); 
    b   = 4;
    c   = 0;
    u_pf = zeros(3,N);

    %% Algorithm
    for idx = 1: N
        pos_i = agents(1:3, idx);
        sum = zeros(3,1);

        for j = 1: N
            if idx==j 
                continue; 
            end
            pos_j = agents(1:3, j);
            pos_ij = pos_i - pos_j;
            norm_ij = norm(pos_ij);
    
            val = (norm_ij - c)/a;
            denominator = 1 + (abs(val))^(2*b);
            if norm_ij > 0.001
                sum = sum + (alp/denominator)*(pos_ij/norm_ij);
            end
        end
    u_pf(:,idx) = sum;
    end
end