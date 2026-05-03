function isdone = check_termination(all_states)
%% check_termination — 벡터화 종료 조건 함수
%  입력
%   all_states : [5 x N]  (x, y, V, psi, phi)
%  출력
%   isdone     : [1 x N]  1 = 해당 에이전트 종료, 0 = 계속
%
%  종료 조건 (어떤 에이전트라도 만족 시 전체 episode 종료)
%   1) 위치가 작업 영역(5km) 밖으로 이탈
%   2) 상태에 NaN/Inf 발생 (수치 폭주)
%   3) 속도가 비정상 범위 (V < 1 m/s 또는 V > 100 m/s)
%
%  Note: 충돌은 종료 조건에 포함하지 않음
%        (보상 함수의 col_pen 으로 연속적으로 학습 신호 부여)
%        → 충돌마다 episode가 끝나면 학습 데이터가 너무 적어짐
%% ------------------------------------------------------------------------

    N      = size(all_states, 2);
    isdone = false(1, N);

    %% 1) 위치 이탈
    pos = all_states(1:2, :);
    out_of_bounds = false;
    for i = 1:N
        if norm(pos(:, i)) > 5000
            out_of_bounds = true;
        end
    end

    %% 2) 수치 폭주
    nan_state = any(any(isnan(all_states))) || any(any(isinf(all_states)));

    %% 3) 비정상 속도
    V    = all_states(3, :);
    bad_V = any(V < 1) || any(V > 100);

    %% 어떤 조건이든 만족 시 모든 에이전트 종료
    if out_of_bounds || nan_state || bad_V
        isdone(:) = true;
    end
end
