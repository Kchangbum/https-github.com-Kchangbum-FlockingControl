function act_cmd = ActualOutput(u_virtual, state, spec)
    % 1. 에이전트 수 인식
    N = size(state, 2);
    
    % 2. [중요] 출력 행렬 초기화
    % MATLAB에게 최대 100대까지는 감당할 수 있다고 '상한선'을 알려줍니다.
    % 시뮬레이션 환경에 따라 20, 50, 100 등 적절한 숫자를 넣으세요.
    act_cmd = zeros(3, N); 
    % 만약 spec이 [2x20]이라면 spec(1, i)로 써야 합니다.
    m = spec(1); 
    g = spec(2);
    D = 0;
    for i = 1:N
        %% Virtual Input (i번째 열 추출)
        ux = u_virtual(1, i);
        uy = u_virtual(2, i);
        uz = u_virtual(3, i);
        
        %% state (i번째 열 추출)
        gam = state(5, i); % flight path angle
        psi = state(6, i); % Heading angle
        
        %% spec (각 드론의 질량/중력 - 행렬일 경우 i 추가)

        
        %% Actual Output 계산
        % 공통 분모 및 뱅크각 계산
        horz_acc = ux * cos(psi) + uy * sin(psi);
        den_phi  = (g + uz) * cos(gam) - horz_acc * sin(gam);
        
        % 분모 0 방지 (안정성)
        if abs(den_phi) < 1e-6, den_phi = sign(den_phi) * 1e-6; end
        
        phi = atan2(uy * cos(psi) - ux * sin(psi), den_phi);
        
        % Lift (양력)
        L = m * (den_phi / max(cos(phi), 0.1));
        
        % Thrust (추력)
        T = m * ((g + uz) * sin(gam) + horz_acc * cos(gam)) + D;
        
        %% [물리적 제한] 주석 해제 추천 (시뮬레이션 폭주 방지)
        % phi = max(min(phi, pi/3), -pi/3); 
        % L   = max(L, 0); 
        % T   = max(T, 0); 
        
        % 세로 벡터(3x1)로 i번째 열에 저장
        act_cmd(:, i) = [phi; L; T]; 
    end
end