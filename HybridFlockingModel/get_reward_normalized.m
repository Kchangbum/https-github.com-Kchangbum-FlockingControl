function [all_rewards, rewards_detail] = get_reward_normalized(all_obs)
%% get_reward_normalized — 음수 누적형 + Team Reward (협응 학습)
%
%  입력
%   all_obs        : [14 x N]  get_observation_normalized 출력
%
%  출력
%   all_rewards    : [1 x N]   각 에이전트 최종 보상 (개별 + 팀 mix)
%   rewards_detail : [5 x N]   세부 항목 [path; align; flock; col; surv]
%
%  보상 설계
%   r_i_individual = w_p·path_pen + w_a·align_pen + w_f·flock_pen + w_c·col_pen + surv
%   r_i_total      = 0.7 × r_i_individual + 0.3 × mean(r_individual)
%
%  Team mixing 30%의 효과:
%   - 한 명이 다른 명을 R_form 밖으로 밀어내면 자기 보상도 깎임
%   - 자연스러운 협력 압력 → cascade 방지
%   - 30% 비율은 너무 강하지 않게 (개별 학습은 70%로 유지)
%
%  관측 채널 매핑 (14차원에서 추출)
%   obs(1,2) = drel_x, drel_y     (정규화된 R_form 기준)
%   obs(6)   = e_radial           (정규화된 R_orbit 기준)
%   obs(7)   = sin(heading error)
%   obs(11)  = nearest 1 dist / R_safe
%% ------------------------------------------------------------------------

    N              = size(all_obs, 2);
    all_rewards    = zeros(1, N);
    rewards_detail = zeros(5, N);

    %% 채널별 스케일 (-10 → 0 우상향 디스플레이용)
    %  1배 스케일로 환원: episode 총합이 [-10, 0] 범위에 들어옴
    %  surv = 0 → 보상은 항상 ≤ 0, 정책 좋아지면 0에 점근
    s_path  = 0.0025;
    s_align = 0.0015;
    s_flock = 0.0020;
    s_col   = 0.0030;
    surv    = 0.0;       % 양수 항 제거 → 0을 절대 안 넘음

    %% Team Mixing 비율
    alpha_indiv = 0.7;     % 개별 보상 비중
    alpha_team  = 0.3;     % 팀 평균 보상 비중

    %% 1단계: 각 에이전트의 개별 보상 계산
    individual = zeros(1, N);
    detail     = zeros(5, N);

    for i = 1:N
        obs_i    = all_obs(:, i);
        drel_x   = obs_i(1);
        drel_y   = obs_i(2);
        e_radial = obs_i(6);
        sE       = obs_i(7);
        d_min_n  = obs_i(11);    % nearest 1 distance / R_safe

        % (1) Path error penalty
        path_pen = -s_path * e_radial^2;

        % (2) Align error penalty (heading vs path tangent)
        align_pen = -s_align * sE^2;

        % (3) Flocking penalty (R_form 바깥)
        d_norm    = sqrt(drel_x^2 + drel_y^2);
        excess    = max(0, d_norm - 1);
        flock_pen = -s_flock * excess^2;

        % (4) Collision penalty (R_safe 안)
        danger  = max(0, 1 - d_min_n);
        col_pen = -s_col * danger^2;

        % (5) Survival bonus
        surv_bonus = surv;

        total = path_pen + align_pen + flock_pen + col_pen + surv_bonus;

        % 안전 클램프
        if isnan(total) || isinf(total)
            total = -0.05;
        end
        total = max(min(total, 0), -0.05);    % per-step [-0.05, 0] (0 절대 안 넘음)

        detail(:, i)  = [path_pen; align_pen; flock_pen; col_pen; surv_bonus];
        individual(i) = total;
    end

    %% 2단계: Team Reward Mixing
    team_mean = mean(individual);

    for i = 1:N
        all_rewards(i)       = alpha_indiv * individual(i) + alpha_team * team_mean;
        rewards_detail(:, i) = detail(:, i);    % 디테일은 개별 페널티 그대로 표시
    end
end
