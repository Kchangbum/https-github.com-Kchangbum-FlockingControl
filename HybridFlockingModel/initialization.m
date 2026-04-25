close all; clear; clc;

% 에이전트 및 환경 설정
m = 15.0;
g = 9.806;
N = 20.0;
min_dist = 100; % [추가] 에이전트 간 최소 보장 거리 (100m)

% 초기화용 빈 배열
x0 = zeros(1, N);
y0 = zeros(1, N);

% 1. 위치 생성 (Collision-free placement)
for i = 1:N
    valid = false;
    while ~valid
        % 1.1. 반지름(R) 및 각도(Theta) 랜덤 설정
        r_temp = 1000 + (1500 - 1000) * rand();
        theta_temp = rand() * 2 * pi;
        
        x_temp = r_temp * cos(theta_temp);
        y_temp = r_temp * sin(theta_temp);
        
        % 1.2. 기존에 배치된 드론들과의 거리 체크
        if i == 1
            valid = true; % 첫 번째 드론은 무조건 통과
        else
            distances = sqrt((x0(1:i-1) - x_temp).^2 + (y0(1:i-1) - y_temp).^2);
            if all(distances >= min_dist)
                valid = true; % 모든 드론과 100m 이상 떨어짐
            end
        end
    end
    x0(i) = x_temp;
    y0(i) = y_temp;
end

% 1.3. 나머지 상태 설정
z0   = ones(1, N) * -100;
V0   = ones(1, N) *20; % [팁] 미세한 속도 차이를 주면 수치적으로 더 안정됨
gam0 = zeros(1, N);

% 2. 헤딩(psi) 설정: 시작부터 목표 원(R=500)의 접선 방향을 바라보게 유도
% (그냥 0으로 두면 급격한 회전 때문에 터질 확률이 높습니다)
theta_pos = atan2(y0, x0);
psi0 = theta_pos + pi/2;

% 3. 하나의 행렬로 묶기
state0 = [x0; y0; z0; V0; gam0; psi0];

state1 = [x0 y0 V0 psi0 0]'; 

% 시각화 확인
% figure('Color', 'w');
% quiver(x0, y0, V0.*cos(psi0), V0.*sin(psi0), 0.5, 'LineWidth', 1.5);
% hold on;
% viscircles([0,0], 500, 'Color', 'r', 'LineStyle', '--'); % 목표 궤도
% grid on; axis equal;
% xlabel('North (x) [m]'); ylabel('East (y) [m]');
% title(['Initial Configuration (N=' num2str(N) ', Min Dist=' num2str(min_dist) 'm)']);