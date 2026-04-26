% 시뮬레이션 파라미터
N = 25; r = 5.0; lambda_val = 2.5; dt = 0.1;
sim = FlockingSim(N, r, lambda_val, dt);
% sim.X = rand(N, 2) * (r * 0.7); 
% sim.V = randn(N, 2) * 0.2;

% 그래픽 설정
fig = figure('Color', 'w', 'Position', [100, 100, 1000, 800]);
hold on; grid on;
colors = hsv(N);

% 에이전트들의 이전 위치를 저장할 변수
old_X = sim.X; 

% 화면 초기화 (에이전트 머리)
h_heads = scatter(sim.X(:,1), sim.X(:,2), 30, colors, 'filled', 'MarkerEdgeColor', 'k');
title('에이전트 전체 이동 경로 기록 (Full Trajectory Log)');

% 실시간 수렴도 표시
h_text = text(0.05, 0.95, '', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');

for s = 1:300
    % 1. 위치 업데이트 전 현재 위치 저장
    old_X = sim.X;
    
    % 2. 모델 업데이트 (논문 식 2.1 적용)
    sim.update();
    
    % 3. 경로 선 그리기 (이전 위치와 현재 위치를 선으로 연결)
    for i = 1:N
        line([old_X(i,1), sim.X(i,1)], [old_X(i,2), sim.X(i,2)], ...
             'Color', [colors(i,:), 0.5], 'LineWidth', 1);
    end
    
    % 4. 에이전트 머리 위치 갱신 (가장 위로 올리기 위해 uistack 사용 가능하나 여기선 간단히 업데이트)
    set(h_heads, 'XData', sim.X(:,1), 'YData', sim.X(:,2));
    
    % 5. 카메라 추적 (전체 경로가 잘 보이도록 범위를 넓게 잡거나 중심 추적)
    avg_pos = mean(sim.X);
    view_range = 15;
    axis([avg_pos(1)-view_range, avg_pos(1)+view_range, ...
          avg_pos(2)-view_range, avg_pos(2)+view_range]);
    
    % 정보 업데이트
    v_var = sum(var(sim.V));
    set(h_text, 'String', sprintf('Step: %d | 속도 분산: %.8f', s, v_var));
    
    drawnow limitrate;
    
    % 거의 수렴하면 종료
    % if v_var < 1e-12, break; end
end