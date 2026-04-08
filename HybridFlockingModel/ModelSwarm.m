function state_dot = ModelSwarm(u_act, state, spec)
    % ut: [3 x 20] (phi, L, T 명령 행렬)
    % state: [6 x 20] (현재 20대의 상태 행렬)
    % spec_matrix: [2 x 20] (20대의 질량, 중력 행렬)

    N = size(state, 2); % 에이전트 수 자동 인식 (20대)
    state_dot = zeros(6, N); % 미분값 저장용 행렬 예약

    % 1. i번째 드론의 파라미터와 입력만 쏙 뽑기
    m = spec(1);
    g = spec(2);
    for i = 1:N
        
        phi = u_act(1, i); L = u_act(2, i); T = u_act(3, i);
        V   = state(4, i); gam = state(5, i); psi = state(6, i);

        % 2. 기존에 만드신 1대용 물리 법칙 적용
        x_dot = V*cos(gam)*cos(psi);
        y_dot = V*cos(gam)*sin(psi);
        z_dot = -V*sin(gam);
        
        VT_dot  = (T)/m - g*sin(gam);
        gam_dot = (L*cos(phi) - m*g*cos(gam)) / (m*V + 1e-6);
        psi_dot = (L*sin(phi)) / (m*V*cos(gam) + 1e-6);

        % 3. i번째 열에 결과 저장
        state_dot(:, i) = [x_dot; y_dot; z_dot; VT_dot; gam_dot; psi_dot];
    end
end