%% 
close all; clc;

%%
% 1. 데이터 추출 (기존 인덱스 유지)
x1 = out.Agent1.Data(:, 1);
y1 = out.Agent1.Data(:, 2);
x2 = out.Agent2.Data(:, 1);
y2 = out.Agent2.Data(:, 2);
x3 = out.Agent3.Data(:, 1);
y3 = out.Agent3.Data(:, 2);
x4 = out.Agent4.Data(:, 1);
y4 = out.Agent4.Data(:, 2);
x5 = out.Agent5.Data(:, 1);
y5 = out.Agent5.Data(:, 2);
x6 = out.Agent6.Data(:, 1);
y6 = out.Agent6.Data(:, 2);

vx1 = out.DotAgent1.Data(:, 1); 
vy1 = out.DotAgent1.Data(:, 2); 
vx2 = out.DotAgent2.Data(:, 1); 
vy2 = out.DotAgent2.Data(:, 2); 
vx3 = out.DotAgent3.Data(:, 1); 
vy3 = out.DotAgent3.Data(:, 2);
vx4 = out.DotAgent4.Data(:, 1); 
vy4 = out.DotAgent4.Data(:, 2);
vx5 = out.DotAgent5.Data(:, 1); 
vy5 = out.DotAgent5.Data(:, 2);
vx6 = out.DotAgent6.Data(:, 1); 
vy6 = out.DotAgent6.Data(:, 2);

Vt1 = out.Agent1.Data(:, 4);
Vt2 = out.Agent2.Data(:, 4);
Vt3 = out.Agent3.Data(:, 4);
Vt4 = out.Agent4.Data(:, 4);
Vt5 = out.Agent5.Data(:, 4);
Vt6 = out.Agent6.Data(:, 4);

% 시간 데이터 추출 (시뮬레이션 시간 축 설정)
t = out.DotAgent2.Time; 

% 이론적 파라미터 설정 (R = 75)
R_val           = 75;
Bound_2R = 2 * R_val; % 150m
Bound_4R = 4 * R_val; % 300m
N = 2;

% %% Figure 1:
% % 2. 그래프 생성
% figure('Color', 'w', 'Name', 'UAV Mission Overview');
% hold on;
% 
% % --- 궤적 그리기 ---
% p1 = plot(x1, y1, 'LineWidth', 1.5, 'DisplayName', 'UAV 1', 'color', [0 0 1]);
% p2 = plot(x2, y2, 'LineWidth', 1.5, 'DisplayName', 'UAV 2', 'color', [0 1 0]);
% p3 = plot(x3, y3, 'LineWidth', 1.5, 'DisplayName', 'UAV 3', 'color', [1 0 0]);
% p4 = plot(x4, y4, 'LineWidth', 1.5, 'DisplayName', 'UAV 4', 'color', [0 1 1]);
% p5 = plot(x5, y5, 'LineWidth', 1.5, 'DisplayName', 'UAV 5', 'color', [1 0 1]);
% p6 = plot(x6, y6, 'LineWidth', 1.5, 'DisplayName', 'UAV 6', 'color', [1 1 0]);
% 
% % --- 시작점(Start) 표시: 동그라미(o) ---
% plot(x1(1), y1(1), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'HandleVisibility', 'off', 'color', [0 0 1]);
% plot(x2(1), y2(1), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'HandleVisibility', 'off', 'color', [0 1 0]);
% plot(x3(1), y3(1), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'HandleVisibility', 'off', 'color', [1 0 0]);
% plot(x4(1), y4(1), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'HandleVisibility', 'off', 'color', [0 1 1]);
% plot(x5(1), y5(1), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'HandleVisibility', 'off', 'color', [1 0 1]);
% plot(x6(1), y6(1), 'o', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'HandleVisibility', 'off', 'color', [1 1 0]);
% text(x1(1)+20, y1(1)+20, 'Start', 'FontSize', 9, 'FontWeight', 'bold');
% 
% % --- 끝점(End) 표시: 엑스(x) ---
% plot(x1(end), y1(end), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off', 'color', [0 0 1]);
% plot(x2(end), y2(end), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off', 'color', [0 1 0]);
% plot(x3(end), y3(end), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off', 'color', [1 0 0]);
% plot(x4(end), y4(end), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off', 'color', [0 1 1]);
% plot(x5(end), y5(end), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off', 'color', [1 0 1]);
% plot(x6(end), y6(end), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off', 'color', [1 1 0]);
% text(x1(end)+20, y1(end)-20, 'End', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
% 
% % % --- 가이드라인 (목표 300m 원) ---
% th = linspace(0, 2*pi, 100);
% plot(500*cos(th), 500*sin(th), 'k--', 'LineWidth', 1, 'DisplayName', 'Target Orbit');
% 
% % 3. 축 범위 및 미관 설정
% axis equal; % 1:1 비율 유지 (중요!)
% grid on;
% grid minor; % 보조 격자선 추가
% 
% % 4. 라벨 및 범례
% xlabel('X Position (North) [m]');
% ylabel('Y Position (East) [m]');
% % title('UAV Orbit Guidance Results');
% legend('Location', 'northeastoutside'); % 범례를 그래프 밖으로 빼서 넉넉하게 보기
% 
% %% Figure 2:
% % 2. 그래프 생성 (성분별 비교)
% figure('Color', 'w', 'Name', 'Position Components Comparison');
% 
% % --- (Subplot 1) X-Velocity (North) 비교 ---
% subplot(2, 1, 1);
% plot(t, x1, 'LineWidth', 1.5, 'DisplayName', 'UAV 1 x', 'color', [0 0 1]);
% hold on;
% plot(t, x2, 'LineWidth', 1.5, 'DisplayName', 'UAV 2 x', 'color', [0 1 0]);
% plot(t, x3, 'LineWidth', 1.5, 'DisplayName', 'UAV 3 x', 'color', [1 0 0]);
% plot(t, x4, 'LineWidth', 1.5, 'DisplayName', 'UAV 4 x', 'color', [0 1 1]);
% plot(t, x5, 'LineWidth', 1.5, 'DisplayName', 'UAV 5 x', 'color', [1 0 1]);
% plot(t, x6, 'LineWidth', 1.5, 'DisplayName', 'UAV 6 x', 'color', [1 1 0]);
% grid on; grid minor;
% ylabel('x (North) [m]');
% title('Position');
% legend('Location', 'best');
% 
% % --- (Subplot 2) Y-Velocity (East) 비교 ---
% subplot(2, 1, 2);
% plot(t, y1, 'LineWidth', 1.5, 'DisplayName', 'UAV 1 y', 'color', [0 0 1]);
% hold on;
% plot(t, y2, 'LineWidth', 1.5, 'DisplayName', 'UAV 2 y', 'color', [0 0 1]);
% plot(t, y3, 'LineWidth', 1.5, 'DisplayName', 'UAV 3 y', 'color', [1 0 0]);
% plot(t, y4, 'LineWidth', 1.5, 'DisplayName', 'UAV 4 y', 'color', [0 1 1]);
% plot(t, y5, 'LineWidth', 1.5, 'DisplayName', 'UAV 5 y', 'color', [1 0 1]);
% plot(t, y6, 'LineWidth', 1.5, 'DisplayName', 'UAV 6 y', 'color', [1 1 0]);
% grid on; grid minor;
% xlabel('Time [s]');
% ylabel('y (East) [m]');
% legend('Location', 'best');
% 
% %% Figure 3:
% % 2. 그래프 생성 (성분별 비교) Vel.ver
% figure('Color', 'w', 'Name', 'Velocity Components Comparison');
% 
% % --- (Subplot 1) X-Velocity (North) 비교 ---
% subplot(3, 1, 1);
% plot(t, vx1, 'LineWidth', 1.5, 'DisplayName', 'UAV 1 vx', 'color', [0 0 1]);
% hold on;
% plot(t, vx2, 'LineWidth', 1.5, 'DisplayName', 'UAV 2 vx', 'color', [0 1 0]);
% plot(t, vx3, 'LineWidth', 1.5, 'DisplayName', 'UAV 3 vx', 'color', [1 0 0]);
% plot(t, vx4, 'LineWidth', 1.5, 'DisplayName', 'UAV 4 vx', 'color', [0 1 1]);
% plot(t, vx5, 'LineWidth', 1.5, 'DisplayName', 'UAV 5 vx', 'color', [1 0 1]);
% plot(t, vx6, 'LineWidth', 1.5, 'DisplayName', 'UAV 6 vx', 'color', [1 1 0]);
% grid on; grid minor;
% ylabel('vx (North) [m/s]');
% title('Velocity');
% legend('Location', 'best');
% 
% % --- (Subplot 2) Y-Velocity (East) 비교 ---
% subplot(3, 1, 2);
% plot(t, vy1, 'LineWidth', 1.5, 'DisplayName', 'UAV 1 vy', 'color', [0 0 1]);
% hold on;
% plot(t, vy2, 'LineWidth', 1.5, 'DisplayName', 'UAV 2 vy', 'color', [0 0 1]);
% plot(t, vy3, 'LineWidth', 1.5, 'DisplayName', 'UAV 3 vy', 'color', [1 0 0]);
% plot(t, vy4, 'LineWidth', 1.5, 'DisplayName', 'UAV 4 vy', 'color', [0 1 1]);
% plot(t, vy5, 'LineWidth', 1.5, 'DisplayName', 'UAV 5 vy', 'color', [1 0 1]);
% plot(t, vy6, 'LineWidth', 1.5, 'DisplayName', 'UAV 6 vy', 'color', [1 1 0]);
% grid on; grid minor;
% xlabel('Time [s]');
% ylabel('vy (East) [m/s]');
% legend('Location', 'best');
% 
% % --- (Subplot 3) Velocity (East) 비교 ---
% subplot(3, 1, 3);
% plot(t, Vt1, 'LineWidth', 1.5, 'DisplayName', 'UAV 1 VT', 'color', [0 0 1]);
% hold on;
% plot(t, Vt2, 'LineWidth', 1.5, 'DisplayName', 'UAV 2 VT', 'color', [0 0 1]);
% plot(t, Vt3, 'LineWidth', 1.5, 'DisplayName', 'UAV 3 VT', 'color', [1 0 0]);
% plot(t, Vt4, 'LineWidth', 1.5, 'DisplayName', 'UAV 4 VT', 'color', [0 1 1]);
% plot(t, Vt5, 'LineWidth', 1.5, 'DisplayName', 'UAV 5 VT', 'color', [1 0 1]);
% plot(t, Vt6, 'LineWidth', 1.5, 'DisplayName', 'UAV 6 VT', 'color', [1 1 0]);
% grid on; grid minor;
% xlabel('Time [s]');
% ylabel('Total Velocity [m/s]');
% legend('Location', 'best');
% 

%%
%% Figure 4 & 5 Expansion for 4 Agents
P1 = [x1, y1]; 
P2 = [x2, y2];
P3 = [x3, y3];
P4 = [x4, y4];

% 2. 군집 중심(Center of Mass) 및 각 에이전트의 거리 계산
% xc, yc, zc를 포함한 중심점 벡터 [Time x 3]
Pc = (P1 + P2 + P3 + P4) / 4;

% ||pi - pc||: 중심으로부터의 3차원 유클리드 거리
dist_1_c = sqrt(sum((P1 - Pc).^2, 2));
dist_2_c = sqrt(sum((P2 - Pc).^2, 2));
dist_3_c = sqrt(sum((P3 - Pc).^2, 2));
dist_4_c = sqrt(sum((P4 - Pc).^2, 2));

% 3. ||pi - pj||: 기체 간의 모든 상대 거리 (6개 조합, 3D)
dist_12 = sqrt(sum((P1 - P2).^2, 2));
dist_13 = sqrt(sum((P1 - P3).^2, 2));
dist_14 = sqrt(sum((P1 - P4).^2, 2));
dist_23 = sqrt(sum((P2 - P3).^2, 2));
dist_24 = sqrt(sum((P2 - P4).^2, 2));
dist_34 = sqrt(sum((P3 - P4).^2, 2));
%% Figure 4:
figure('Color', 'w', 'Name', 'Bound Verification: 4 Agents to Center');
hold on;
plot(t, dist_1_c, 'b-',  'LineWidth', 1.5, 'DisplayName', 'Agent 1');
plot(t, dist_2_c, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Agent 2');
plot(t, dist_3_c, 'g-.', 'LineWidth', 1.5, 'DisplayName', 'Agent 3');
plot(t, dist_4_c, 'm:',  'LineWidth', 1.5, 'DisplayName', 'Agent 4');

% 상한선 (2R)
yline(Bound_2R, 'r--', ['Upper Bound (2R = ', num2str(Bound_2R), 'm)'], ...
    'LineWidth', 2, 'LabelHorizontalAlignment', 'right');

grid on; grid minor;
xlabel('Time [s]'); ylabel('Distance to Center [m]');
title('Verification of ||p_i - p_c|| \leq 2R (4 Agents)');
legend('Location', 'northeastoutside'); % 범례가 그래프를 가리지 않게 밖으로 뺌
%% Figure 5: 기체 간 거리 검증 (||pi - pj|| <= 4R)
figure('Color', 'w', 'Name', 'Bound Verification: Inter-agent (6 Pairs)');
hold on;
% 모든 쌍의 거리 플롯
p12 = plot(t, dist_12, 'DisplayName', 'Pair 1-2');
p13 = plot(t, dist_13, 'DisplayName', 'Pair 1-3');
p14 = plot(t, dist_14, 'DisplayName', 'Pair 1-4');
p23 = plot(t, dist_23, 'DisplayName', 'Pair 2-3');
p24 = plot(t, dist_24, 'DisplayName', 'Pair 2-4');
p34 = plot(t, dist_34, 'DisplayName', 'Pair 3-4');

% 상한선 (4R) 및 목표 거리 (2R)
yline(Bound_4R, 'r--', 'Upper Bound (4R)', 'LineWidth', 2);
yline(150, 'b:', 'Desired Distance (2R)', 'LineWidth', 1.5);

grid on; grid minor;
xlabel('Time [s]'); ylabel('Inter-agent Distance [m]');
title('Verification of ||p_i - p_j|| \leq 4R (All Pairs)');
legend('Location', 'northeastoutside');

%% Figure 6: Snapshots with Agent & Formation Legend (DisplayName 활용)
figure('Color', 'w', 'Name', 'Comprehensive Swarm Analysis');
hold on;

% 1. 데이터 준비 및 수렴 조건 (R_max <= 75m)
all_x = [x1, x2, x3, x4, x5, x6]; 
all_y = [y1, y2, y3, y4, y5, y6];
centroids_x = mean(all_x, 2);
centroids_y = mean(all_y, 2);
dist_from_c = sqrt((all_x - centroids_x).^2 + (all_y - centroids_y).^2);
[R_max_val, farthest_uav_idx] = max(dist_from_c, [], 2);

% 수렴 시점 찾기
conv_indices = find(R_max_val <= 150);
if isempty(conv_indices)
    sample_indices = round(linspace(1, length(t), 5)); 
else
    % 수렴 이후 5개 스냅샷 선택
    sample_indices = round(linspace(conv_indices(1), length(t), 5));
end

% --- 시각화 설정 ---
colors = [0 0 1; 0 1 0; 1 0 0; 0 1 1; 1 0 1; 0.5 0.5 0.5]; % 파, 녹, 빨, 청록, 자홍, 회색
th_plt = linspace(0, 2*pi, 100);

% 배경 궤적 및 목표 궤도 (범례 제외)
for k = 1:6
    plot(all_x(:,k), all_y(:,k), ':', 'Color', colors(k,:), 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(500*cos(th_plt), 500*sin(th_plt), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target Orbit (500m)');

% 2. [추가] 각 에이전트의 색상 라벨링 (더미 플롯 활용)
% 그래프 밖 영역에 아주 작은 점을 찍어 범례에만 나타나게 함
for k = 1:6
    plot(NaN, NaN, 'o', 'MarkerFaceColor', colors(k,:), 'MarkerEdgeColor', 'k', ...
        'MarkerSize', 6, 'DisplayName', sprintf('UAV %d', k));
end

% 3. 스냅샷 루프
snapshot_count = 1;
% 스냅샷 원에 사용할 별도의 컬러맵 (에이전트 색상과 구별)
snap_colors = lines(length(sample_indices)); 

for i = sample_indices
    cur_x = all_x(i, :);
    cur_y = all_y(i, :);
    c_x = centroids_x(i);
    c_y = centroids_y(i);
    idx_far = farthest_uav_idx(i);
    val_far = R_max_val(i);
    
    % [중요] 범례에 들어갈 스냅샷 정보 생성
    % 예: "#1: UAV3 (D:148.5m)"
    lgd_name = sprintf('#%d: UAV%d (D:%.1fm)', snapshot_count, idx_far, val_far*2);
    
    % [원 그리기] 이 원의 DisplayName이 범례에 표시됩니다.
    plot(c_x + val_far*cos(th_plt), c_y + val_far*sin(th_plt), ...
        'LineWidth', 1.5, 'Color', snap_colors(snapshot_count,:), 'DisplayName', lgd_name); 
    
    % [중심에 순서 표시] (그래프 내부 가독성)
    text(c_x, c_y, num2str(snapshot_count), 'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'w', 'EdgeColor', 'k');

    % 기체 간 연결 및 마커 (범례 제외)
    % plot([cur_x, cur_x(1)], [cur_y, cur_y(1)], 'k-', 'Color', [0.7 0.7 0.7], 'HandleVisibility', 'off');
    for k = 1:6
        plot(cur_x(k), cur_y(k), 'o', 'MarkerFaceColor', colors(k,:), 'MarkerEdgeColor', 'k', ...
            'MarkerSize', 5, 'HandleVisibility', 'off');
    end
    
    snapshot_count = snapshot_count + 1;
end

% 4. 마무리 설정
axis equal; grid on;
xlabel('North [m]'); ylabel('East [m]');
title('Swarm Analysis: Agent Labels & Formation Convergence Info');

% 범례를 그래프 바깥 우측에 배치
legend('Location', 'northeastoutside', 'FontSize', 9, 'TextColor', 'k');
