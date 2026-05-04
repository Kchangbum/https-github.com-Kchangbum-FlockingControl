function [all_rewards, rewards_detail] = get_reward_normalized(all_obs)
%% get_reward_normalized — 벡터화 보상 함수 (음수 누적형, 우상향)
%
%  입력
%   all_obs        : [10 x N]  get_observation_normalized 출력
%
%  출력
%   all_rewards    : [1  x N]  각 에이전트의 총 보상
%   rewards_detail : [5  x N]  세부 항목 [path; align; flocking; col; surv]
%
%  목표 학습 곡선
%     초기(나쁜 정책)   → episode 총합 ≈ -10
%     학습 후(좋은 정책) → episode 총합 ≈   0
%     (Episode 길이: Tf/Ts = 200/0.1 = 2000 step)
%
%  관측에서 직접 디코드 (정규화된 값을 그대로 활용)
%     obs(1,2) = (drel_x, drel_y)        = (p_i - p_c)/R_form
%     obs(6)   = e_radial                = (|p_c| - R_orbit)/R_orbit
%     obs(7,8) = sin/cos(dpsi)           : 접선 방향 헤딩 오차
%     obs(10)  = d_min / R_safe          : 최근접 이웃 거리(정규화)
%% ------------------------------------------------------------------------

    N              = size(all_obs, 2);
    all_rewards    = zeros(1, N);
    rewards_detail = zeros(5, N);

    %% --- 채널별 스케일 (gradient 신호 강화 버전) -----------------------
    %  이전 학습이 너무 빨리 평탄화되어 정책 붕괴 → 스케일을 5배 키워
    %  per-step penalty 크기를 ↑ 시켜 critic Q-value의 식별력 확보
    %  새 episode 합계: bad ≈ -50, good ≈ 0  (절대값보다 gradient 강도가 중요)
    s_path  = 0.0125;     % 군집 중점 경로 추종 (5×)
    s_align = 0.0075;     % 헤딩 정렬 (접선 방향) (5×)
    s_flock = 0.0100;     % 군집 응집 (R_form 바깥에서만) (5×)
    s_col   = 0.0150;     % 충돌 회피 (R_safe 안에서만) (5×)
    surv    = 0.0015;     % survival bonus (5×, 전체 ≈ +3)

    for i = 1:N
        obs_i    = all_obs(:, i);
        drel_x   = obs_i(1);
        drel_y   = obs_i(2);
        e_radial = obs_i(6);
        sE       = obs_i(7);     % sin(heading error)
        d_min_n  = obs_i(10);    % d_min / R_safe

        %% (1) Path error penalty ─ 군집 중점 궤도 오차
        %     e_radial² (정규화) → 항상 ≥ 0, 곱하면 ≤ 0
        path_pen = -s_path * e_radial^2;

        %% (2) Align error penalty ─ 접선 방향 정렬
        %     sin²(dpsi): 헤딩 오차 90° → 1 (최대), 0° → 0
        align_pen = -s_align * sE^2;

        %% (3) Flocking penalty ─ R_form 바깥일 때만 부과
        %     d_norm > 1 이면 자유 비행 영역(R_form) 이탈
        d_norm = sqrt(drel_x^2 + drel_y^2);
        excess = max(0, d_norm - 1);
        flock_pen = -s_flock * excess^2;

        %% (4) Collision penalty ─ R_safe 안일 때만 부과
        %     d_min_n < 1 이면 가까움; (1 - d_min_n)² 로 부드러운 증가
        danger  = max(0, 1 - d_min_n);
        col_pen = -s_col * danger^2;

        %% (5) Survival bonus ─ 매 스텝 작은 양수
        surv_bonus = surv;

        %% --- 총합 ---
        total = path_pen + align_pen + flock_pen + col_pen + surv_bonus;

        %% --- 안전 클램프 (수치 폭주/폭락 방지) ---
        if isnan(total) || isinf(total)
            total = -0.25;
        end
        total = max(min(total, 0.005), -0.5);  % per-step [-0.5, +0.005]

        rewards_detail(:, i) = [path_pen; align_pen; flock_pen; col_pen; surv_bonus];
        all_rewards(i)       = total;
    end
end
