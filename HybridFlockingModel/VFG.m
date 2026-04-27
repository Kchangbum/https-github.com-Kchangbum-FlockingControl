function [VFG,psi_wrapped ,des_psi, psi_err] = VFG(agents)
    N = size(agents, 2);
    VFG = zeros(3,N);
    xt = 0; yt = 0; zt = -100; % Target Position (NED 기준, -100m는 상공)
    Vd = 20;                   % Desired Speed
    rd = 500;                  % Desired Orbit Radius

    for i = 1 : N
        %% 1. State Input (Inertial Frame)
        x = agents(1,i);
        y = agents(2,i);
        z = agents(3,i);
        V = agents(4,i);      % Airspeed (or Groundspeed)
        gam = agents(5,i);    % Flight path angle
        psi = agents(6,i);    % Heading angle
        % psi = atan2(sin(psi), cos(psi));
        % des_psi = psi;
        
        %% 2. Desired Velocity Vector Field (Equation 7)
        xr = x - xt;
        yr = y - yt;
        r = sqrt(xr^2 + yr^2);
        
        % 분모 0 방지 및 수치 안정성 확보
        r_safe = max(r, 0.01);
        
        % Matrix A &amp; B formulation (Equation 7)
        % 논문의 수식: [xr(r^2-rd^2) + yr(2r rd); yr(r^2-rd^2) - xr(2r rd)]
        % A = [ (r_safe^2 - rd^2),   (2 * r_safe * rd); 
        %      -(2 * r_safe * rd),   (r_safe^2 - rd^2) ];
        % B = [xr; yr];
        
        xy_dot = [ 
            (r_safe^2 - rd^2)*xr + (2*r_safe*rd)*yr ;
            (r_safe^2 - rd^2)*yr - (2*r_safe*rd)*xr
            ];
        
        % xy_dot_des = [dot_x_di; dot_y_di]
        xy_dot_des = -(Vd / (r_safe * (r_safe^2 + rd^2))) .* xy_dot;
        
        %% 4. Desired Heading Angle (Equation 8)
        des_psi = atan2(xy_dot_des(2), xy_dot_des(1));
        psi_wrapped = atan2(sin(psi), cos(psi));
        %% 5. Control Commands in Horizontally Body-fixed Frame (Equation 5)
        
        
        % U2: Heading tracking (Lateral) - Angle Wrap-around 처리 필수
        % psi_err = des_psi - psi;
        % psi_err = atan2(sin(des_psi - psi), cos(des_psi - psi));
        
        % 목표와 현재 각도의 차이 구하기
        psi_err = des_psi - psi_wrapped;
        % [중요] Reddit 로직: 180도(pi)를 넘어가면 반대 방향 최단거리로 보정
        if psi_err > pi
            psi_err = psi_err - 2*pi;
        elseif psi_err < -pi
            psi_err = psi_err + 2*pi;
        end
        
        
        U1 = Vd - V * cos(gam); % U1: Speed tracking (Longitudinal)
        U2 = V * psi_err;
        U3 = zt - z; % U3: Altitude tracking (Vertical)
        U = [U1; U2; U3];
        
        %% 6. Transform to Inertial Frame (Equation 6)
        % R_psi rotation matrix
        tran = [
            cos(psi) -sin(psi) 0;
            sin(psi)  cos(psi) 0;
            0         0        1
            ];
        
        
        % 가속도(또는 속도 변화율) 명령 생성
        VFG(:,i) = tran * U;
    end
end