%% 1. 데이터 로드 및 차원 재정렬 (5 x N x Time -> Time x 5 x N)
close all; clc;

% out 데이터 구조에서 데이터 추출
if ~exist('out', 'var')
    error('워크스페이스에 시뮬레이션 결과(out)가 없습니다.');
end

% [5 x N x Time] 데이터 가져오기 (상태: x, y, V, psi, phi)
raw_data = out.Agent1.Data; 
t = out.Agent1.Time;
N = size(raw_data, 2); 
num_steps = length(t);

% 차원 교환: [Time x 5 x N]
% Index: 1:x, 2:y, 3:V, 4:psi, 5:phi(bank)
data_perm = permute(raw_data, [3 1 2]); 
 
% 에이전트별 데이터 분리 [Time x N]
pos_x   = squeeze(data_perm(:, 1, :)); % North (x)
pos_y   = squeeze(data_perm(:, 2, :)); % East (y)
vel_mag = squeeze(data_perm(:, 3, :)); % Speed (V)
psi     = squeeze(data_perm(:, 4, :)); % Heading (psi)
phi     = squeeze(data_perm(:, 5, :)); % Bank Angle (phi)

% 시각화 파라미터 및 색상 설정
R_val = 75;
Bound_2R = 2 * R_val;
Bound_4R = 4 * R_val;
colors = hsv(N); 

%% Figure 1: 2D 궤적 및 목표 궤도 (Orbit)
figure('Color', 'w', 'Name', '2D Swarm Trajectory');
hold on;
th = linspace(0, 2*pi, 100);
plot(500*cos(th), 500*sin(th), 'k--', 'LineWidth', 2, 'DisplayName', 'Target Orbit (500m)');

for i = 1:N
    plot(pos_x(:, i), pos_y(:, i), 'Color', [colors(i,:), 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(pos_x(1, i), pos_y(1, i), 'o', 'MarkerSize', 4, 'MarkerFaceColor', colors(i,:), 'HandleVisibility', 'off');
    plot(pos_x(end, i), pos_y(end, i), 'x', 'MarkerSize', 6, 'Color', colors(i,:), 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

axis equal; grid on; grid minor;
xlabel('North (x) [m]'); ylabel('East (y) [m]');
title(sprintf('Swarm 2D Trajectory (%d Agents)', N));

%% Figure 2: 상태 변수 수렴 확인 (Speed & Bank Angle)
figure('Color', 'w', 'Name', 'State Convergence');

% --- Speed Tracking ---
subplot(2, 1, 1); hold on;
plot(t, vel_mag, 'LineWidth', 0.5, 'Color', [0.7 0.7 0.7]); 
plot(t, mean(vel_mag, 2), 'k', 'LineWidth', 1.5, 'DisplayName', 'Mean Speed'); 
yline(20, 'r--', 'Target (20m/s)', 'LineWidth', 2);
grid on; ylabel('Speed (V) [m/s]'); title('Velocity Tracking');
legend('Location', 'best');

% --- Bank Angle (phi) Tracking ---
subplot(2, 1, 2); hold on;
plot(t, rad2deg(phi), 'LineWidth', 0.5, 'Color', [0.7 0.7 0.7]);
plot(t, rad2deg(mean(phi, 2)), 'b', 'LineWidth', 1.5, 'DisplayName', 'Mean Phi');
grid on; xlabel('Time [s]'); ylabel('Bank Angle (\phi) [deg]'); title('Bank Angle Dynamics');

%% Figure 3: 군집 응집도 및 거리 검증 (Cohesion)
figure('Color', 'w', 'Name', 'Cohesion Analysis');

% --- Subplot 1: 중심으로부터의 거리 (2R) ---
subplot(2, 1, 1); hold on;
mean_x = mean(pos_x, 2); mean_y = mean(pos_y, 2);
dist_to_center = sqrt((pos_x - mean_x).^2 + (pos_y - mean_y).^2);
plot(t, dist_to_center, 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
plot(t, max(dist_to_center, [], 2), 'r', 'LineWidth', 1, 'DisplayName', 'Max Dist');
yline(Bound_2R, 'r--', 'Limit (2R)', 'LineWidth', 2);
grid on; ylabel('Dist to Center [m]');
title('Verification: ||p_i - p_c|| \leq 2R');

% --- Subplot 2: 기체 간 최소/최대 거리 (4R) ---
subplot(2, 1, 2); hold on;
sample_step = 20;
s_idx = 1:sample_step:num_steps;
min_pair = zeros(length(s_idx), 1);
max_pair = zeros(length(s_idx), 1);

for k = 1:length(s_idx)
    dists = pdist([pos_x(s_idx(k), :)', pos_y(s_idx(k), :)']);
    min_pair(k) = min(dists);
    max_pair(k) = max(dists);
end

fill([t(s_idx); flipud(t(s_idx))], [min_pair; flipud(max_pair)], 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(t(s_idx), max_pair, 'r', 'LineWidth', 1, 'DisplayName', 'Max Inter-agent');
plot(t(s_idx), min_pair, 'g', 'LineWidth', 1, 'DisplayName', 'Min Inter-agent');
yline(Bound_4R, 'k--', 'Upper Limit (4R)', 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('Inter-agent Dist [m]');
title('Verification: ||p_i - p_j|| \leq 4R');
legend('Location', 'best');

%% Figure 4: Swarm Snapshot & 4R Bound Verification
figure('Color', 'w', 'Name', 'Swarm Snapshots');
hold on;
th_circle = linspace(0, 2*pi, 100);
num_snaps = 5; 
snap_indices = round(linspace(1, num_steps, num_snaps)); 
snap_colors = lines(num_snaps); 

plot(500*cos(th_circle), 500*sin(th_circle), 'k:', 'LineWidth', 1, 'DisplayName', 'Target Orbit');

for s = 1:num_snaps
    idx = snap_indices(s);
    cur_x = pos_x(idx, :); cur_y = pos_y(idx, :);
    c_x = mean(cur_x); c_y = mean(cur_y);
    
    dists = pdist([cur_x', cur_y']);
    max_d = max(dists); 
    
    plot(cur_x, cur_y, '.', 'Color', snap_colors(s,:), 'MarkerSize', 8, 'HandleVisibility', 'off');
    if s > 1
        plot(c_x + (max_d/2)*cos(th_circle), c_y + (max_d/2)*sin(th_circle), ...
            '--', 'Color', snap_colors(s,:), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('t=%.1fs (MaxD=%.1fm)', t(idx), max_d));
    text(c_x, c_y, num2str(s-1), 'Color', snap_colors(s,:), 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    end
end

axis equal; grid on;
xlabel('North (x) [m]'); ylabel('East (y) [m]');
title('Swarm Snapshots: Orbit Entry & Formation Verification');
legend('Location', 'northeastoutside');