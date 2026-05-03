function obs_i = compute_observation(state, agent_idx, R_orbit, V_target, R_form, R_safe)
%% compute_observation — Hybrid Flocking 관측 함수 (에이전트별)
%  Simulink의 MATLAB Function 블록에 그대로 붙여넣어 사용.
%  입력
%   state        : [5 x N]  (x, y, V, psi, phi)
%   agent_idx    : 본 에이전트 인덱스 (1..N)
%   R_orbit, V_target, R_form, R_safe : 학습 스크립트의 정규화 파라미터
%
%  반환
%   obs_i        : [10 x 1]  정규화된 관측 벡터
%
%  설계
%   - 모든 값은 정규화되어 ~[-3, 3] 범위에 들어옴 → 학습 안정성 ↑
%   - 헤딩은 sin/cos 분해 → 각도 wrap-around 문제 제거
%   - 가장 가까운 이웃 정보 → 충돌 회피 학습에 핵심 신호
%% ------------------------------------------------------------------------
    N    = size(state, 2);
    pos  = state(1:2, :);
    p_i  = pos(:, agent_idx);
    V_i  = state(3, agent_idx);
    psi_i= state(4, agent_idx);

    % 군집 중점
    pc   = mean(pos, 2);

    %  1,2) 중점 기준 상대 위치 (정규화)
    drel = (p_i - pc) / R_form;

    %  3) 속도 오차
    dV   = (V_i - V_target) / V_target;

    %  4,5) 헤딩 sin/cos
    sP = sin(psi_i);
    cP = cos(psi_i);

    %  6) 군집 중점의 궤도 반경 오차
    rc        = norm(pc);
    e_radial  = (rc - R_orbit) / R_orbit;

    %  7,8) 접선 방향과 본인 헤딩의 차이 (sin/cos 분해)
    th_c    = atan2(pc(2), pc(1));
    psi_des = th_c + pi/2;
    dpsi    = psi_des - psi_i;
    %   wrap to [-pi, pi]
    dpsi    = atan2(sin(dpsi), cos(dpsi));
    sE = sin(dpsi);
    cE = cos(dpsi);

    %  9,10) 가장 가까운 이웃 정보
    d_min   = inf;
    rel_min = [0; 0];
    for j = 1:N
        if j == agent_idx, continue; end
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
    d_min_n   = d_min / R_safe;

    obs_i = [drel(1); drel(2); dV; sP; cP; e_radial; sE; cE; rel_min_n(1); d_min_n];

    % 안전 클램프 (수치 폭주 방지)
    obs_i = max(min(obs_i, 10), -10);
end
