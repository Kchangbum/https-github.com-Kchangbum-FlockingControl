function all_obs = get_observation_normalized(all_states)
%% get_observation_normalized — 벡터화 관측 함수
%  입력
%   all_states  : [5 x N]  각 열 = (x, y, V, psi, phi)  (5대 → N=5)
%  출력
%   all_obs     : [10 x N] 정규화된 관측, 각 열이 한 에이전트의 obs_i
%
%  관측 채널 (각 열당 10개)
%   1) (x_i - x_c)/R_form           : 군집 중점 기준 상대 X
%   2) (y_i - y_c)/R_form           : 군집 중점 기준 상대 Y
%   3) (V_i - V_target)/V_target    : 속도 오차
%   4) sin(psi_i)                   : 헤딩 sin
%   5) cos(psi_i)                   : 헤딩 cos
%   6) (|p_c| - R_orbit)/R_orbit    : 군집 중점 궤도 반경 오차
%   7) sin(psi_des - psi_i)         : 접선 방향 헤딩 오차 sin
%   8) cos(psi_des - psi_i)         : 접선 방향 헤딩 오차 cos
%   9) Δx_nearest / R_form          : 가장 가까운 이웃 상대 X
%  10) d_min / R_safe               : 가장 가까운 이웃까지 거리(정규화)
%
%  Note: Simulink MATLAB Function 블록에 그대로 넣을 수 있도록
%        파라미터는 함수 내부에서 상수 선언 (학습 스크립트와 동기화 필요)
%% ------------------------------------------------------------------------

    %% --- 정규화 상수 (flocking_learning.m 과 동기화) -----------------
    R_orbit  = 500;
    V_target = 20;
    R_form   = 75;
    R_safe   = 30;

    %% --- 사이즈 ----------------------------------------------------
    N       = size(all_states, 2);
    all_obs = zeros(10, N);

    pos = all_states(1:2, :);              % 2 x N
    pc  = mean(pos, 2);                    % 2 x 1, 군집 중점

    rc       = norm(pc);
    e_radial = (rc - R_orbit) / R_orbit;
    th_c     = atan2(pc(2), pc(1));
    psi_des  = th_c + pi/2;                % 접선 방향

    %% --- 에이전트별 관측 계산 ---------------------------------------
    for i = 1:N
        p_i   = pos(:, i);
        V_i   = all_states(3, i);
        psi_i = all_states(4, i);

        % 1,2) 중점 기준 상대 위치
        drel  = (p_i - pc) / R_form;

        % 3) 속도 오차
        dV    = (V_i - V_target) / V_target;

        % 4,5) 헤딩 sin/cos
        sP = sin(psi_i);
        cP = cos(psi_i);

        % 7,8) 접선 방향과 본인 헤딩 차이 (wrap-around 안전)
        dpsi = psi_des - psi_i;
        dpsi = atan2(sin(dpsi), cos(dpsi));
        sE   = sin(dpsi);
        cE   = cos(dpsi);

        % 9,10) 가장 가까운 이웃
        d_min   = inf;
        rel_min = [0; 0];
        for j = 1:N
            if j == i, continue; end
            rel = pos(:, j) - p_i;
            d   = norm(rel);
            if d < d_min
                d_min   = d;
                rel_min = rel;
            end
        end
        if isinf(d_min)
            d_min   = 10 * R_safe;
            rel_min = [0; 0];
        end

        rel_min_n = rel_min / R_form;
        d_min_n   = d_min   / R_safe;

        obs_i = [drel(1); drel(2); dV; sP; cP; e_radial; sE; cE; rel_min_n(1); d_min_n];

        % 안전 클램프
        obs_i = max(min(obs_i, 10), -10);
        all_obs(:, i) = obs_i;
    end
end
