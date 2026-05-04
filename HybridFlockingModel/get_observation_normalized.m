function all_obs = get_observation_normalized(all_states)
%% get_observation_normalized — 14차원 벡터화 관측 (k=2 nearest neighbors)
%
%  입력
%   all_states  : [5 x N]  각 열 = (x, y, V, psi, phi)
%  출력
%   all_obs     : [14 x N] 정규화된 관측
%
%  관측 채널 (각 열당 14개)
%   1)  drel_x    = (x_i - x_c)/R_form
%   2)  drel_y    = (y_i - y_c)/R_form
%   3)  dV        = (V_i - V_target)/V_target
%   4)  sin(psi_i)
%   5)  cos(psi_i)
%   6)  e_radial  = (|p_c| - R_orbit)/R_orbit
%   7)  sin(psi_des - psi_i)
%   8)  cos(psi_des - psi_i)
%   9)  Nearest 1: rel_x / R_form
%  10)  Nearest 1: rel_y / R_form
%  11)  Nearest 1: dist  / R_safe
%  12)  Nearest 2: rel_x / R_form
%  13)  Nearest 2: rel_y / R_form
%  14)  Nearest 2: dist  / R_safe
%
%  k=2 이웃 정보를 추가한 이유:
%   - 1개만 보면 "내 옆 친구가 다가오면 멀어짐" → cascade
%   - 2개 보면 "옆 친구가 다가올 때 또 다른 친구가 어디 있는지" 인식
%   - 정책이 "어느 방향으로 비키는 게 안전한지" 학습 가능
%% ------------------------------------------------------------------------

    %% 정규화 상수 (flocking_learning.m 과 동기화)
    R_orbit  = 500;
    V_target = 20;
    R_form   = 75;
    R_safe   = 30;

    N       = size(all_states, 2);
    all_obs = zeros(14, N);

    pos = all_states(1:2, :);
    pc  = mean(pos, 2);

    rc       = norm(pc);
    e_radial = (rc - R_orbit) / R_orbit;
    th_c     = atan2(pc(2), pc(1));
    psi_des  = th_c + pi/2;

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

        % 9-14) 2명의 가장 가까운 이웃
        dists = inf(1, N);
        rels  = zeros(2, N);
        for j = 1:N
            if j == i, continue; end
            rel        = pos(:, j) - p_i;
            dists(j)   = norm(rel);
            rels(:, j) = rel;
        end

        % 거리 오름차순 정렬
        [sorted_d, sorted_idx] = sort(dists);

        % Nearest 1
        if sorted_d(1) < inf
            d1 = sorted_d(1);
            r1 = rels(:, sorted_idx(1));
        else
            d1 = 10 * R_safe;
            r1 = [0; 0];
        end

        % Nearest 2
        if length(sorted_d) >= 2 && sorted_d(2) < inf
            d2 = sorted_d(2);
            r2 = rels(:, sorted_idx(2));
        else
            d2 = 10 * R_safe;
            r2 = [0; 0];
        end

        obs_i = [
            drel(1); drel(2); dV; sP; cP; e_radial; sE; cE;
            r1(1)/R_form; r1(2)/R_form; d1/R_safe;
            r2(1)/R_form; r2(2)/R_form; d2/R_safe
            ];

        % 안전 클램프
        obs_i = max(min(obs_i, 10), -10);
        all_obs(:, i) = obs_i;
    end
end
