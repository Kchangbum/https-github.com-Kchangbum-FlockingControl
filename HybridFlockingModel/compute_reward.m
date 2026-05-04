function r_i = compute_reward(state, action, agent_idx, ...
                              R_orbit, V_target, R_form, R_safe, R_collision, R_swarm_max)
%% compute_reward — Hybrid Flocking 보상 함수 (음수 누적형, 우상향)
%
%  목표 학습 곡선:
%     초기(나쁜 정책)   → episode 총합 ≈ -10
%     학습 후(좋은 정책) → episode 총합 ≈   0
%
%  스케일 도출:
%     Episode 길이 = Tf/Ts = 200/0.1 = 2000 step 
%     초기 평균 cost ≈ 1.0 (정규화 가중합 기준)
%     scale = 10 / (2000 × 1.0) = 0.005
%
%  설계 원칙:
%   - 모든 항: 완벽 → 0,  나쁨 → 음수 (cost = 정규화 오차의 제곱)
%   - 충돌 등 위험 추가 페널티는 작게 → 그래프 폭락 방지
%   - per-step 보상은 [-0.1, 0]으로 캡
%
%% ------------------------------------------------------------------------
    coder.extrinsic('disp');

    N   = size(state, 2);
    pos = state(1:2, :);                 % 2 x N
    p_i = pos(:, agent_idx);
    V_i = state(3, agent_idx);

    % --- 군집 중점 ---
    pc  = mean(pos, 2);

    %% (A) 군집 중점 경로 오차 (정규화 제곱) ────────────────────────────────
    rc_norm  = norm(pc);
    e_radial = (rc_norm - R_orbit) / R_orbit;       % 0이면 완벽
    e_path   = e_radial^2;

    %% (B) 응집 오차 — R_form 바깥일 때만 부과 ─────────────────────────────
    %   에이전트가 자유롭게 움직일 수 있는 영역(R_form) 안에서는 비용 0
    d_ic   = norm(p_i - pc);
    excess = max(0, d_ic - R_form);
    e_coh  = (excess / R_form)^2;

    %% (C) 충돌 위험 — R_safe 안일 때만 부과 ────────────────────────────────
    %   d_min ≥ R_safe → cost 0 (안전)
    %   d_min < R_safe → cost가 거리 비례로 부드럽게 증가
    d_min = inf;
    for j = 1:N
        if j == agent_idx, continue; end
        d_ij = norm(pos(:, j) - p_i);
        if d_ij < d_min, d_min = d_ij; end
    end
    if isinf(d_min), d_min = 1e6; end
    danger = max(0, R_safe - d_min);
    e_safe = (danger / R_safe)^2;

    %% (D) 속도 오차 (정규화 제곱) ──────────────────────────────────────────
    e_vel = ((V_i - V_target) / V_target)^2;

    %% --- 가중합 (모두 ≥ 0, 단위: 정규화 제곱오차) -----------------------
    w_path = 1.0;     % 군집 중점 경로 추종이 최우선
    w_coh  = 0.6;     % 자유 비행 영역 — 강제하지 않음
    w_safe = 0.8;     % 충돌 회피 — 핵심 안전
    w_vel  = 0.4;     % 속도 추종 — 보조

    cost = w_path*e_path + w_coh*e_coh + w_safe*e_safe + w_vel*e_vel;

    %% --- 음수 보상 ------------------------------------------------------
    %   scale = 0.005 → 초기 episode 총합 ≈ -10
    scale = 0.005;
    r_i = -scale * cost;

    %% --- 위험 상황 추가 페널티 (작게, 그래프 폭락 방지) ─────────────────
    %  충돌 임박/발생: tanh 형태 부드러운 추가 음수
    if d_min < R_collision
        r_i = r_i - 0.05 * (1 + (R_collision - d_min)/max(R_collision,1));
    end
    %  군집 산개: 약한 추가 음수
    if d_ic > R_swarm_max
        r_i = r_i - 0.02;
    end

    %% --- 안전 클램프 ----------------------------------------------------
    if isnan(r_i) || isinf(r_i)
        r_i = -0.1;
    end
    r_i = max(min(r_i, 0), -0.1);   % per-step 보상은 [-0.1, 0]
end
