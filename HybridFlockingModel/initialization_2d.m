clc;
r2d = 180/pi;
d2r = pi/180;

% 에이전트 및 환경 설정
m = 5.0;
g = 9.806;
N = 5;
spec = [m; g];
min_dist = 100; % 에이전트 간 최소 이격 거리

% 1. 위치 생성 (1사분면, 반경 1500m 이상)
x0 = zeros(1, N);
y0 = zeros(1, N);

for i = 1:N
    valid = false;
    while ~valid
        % 거리: 1500m ~ 2500m 사이 (너무 좁으면 20대가 안 들어갈 수 있어 여유를 줌)
        r_temp = 500 + 600 * rand(); 
        
        % 각도: 0 ~ pi/2 (1사분면)
        theta_temp = rand() * (pi/2); 
        
        x_temp = r_temp * cos(theta_temp);
        y_temp = r_temp * sin(theta_temp);
        
        if i == 1
            valid = true;
        else
            % 기존 에이전트들과의 거리 체크 (충돌 방지)
            distances = sqrt((x0(1:i-1) - x_temp).^2 + (y0(1:i-1) - y_temp).^2);
            if all(distances >= min_dist)
                valid = true;
            end
        end
    end
    x0(i) = x_temp;
    y0(i) = y_temp;
end

% 2. 나머지 상태 설정
V0 = ones(1, N) * 20; % 초기 속도
theta_pos = atan2(y0, x0);
psi0 = theta_pos + pi/2; % 원 궤적을 그리며 시작하도록 접선 방향 헤딩 설정
bank0 = zeros(1, N);

% 3. 최종 초기 상태 행렬 (5 x N)
state0 = [x0; y0; V0; psi0; bank0];

% 시각화 확인 (제대로 배치됐는지 확인용)
% figure;
% plot(x0, y0, 'ro', 'MarkerSize', 8, 'LineWidth', 2); hold on;
% xlabel('X Position (m)'); ylabel('Y Position (m)');
% title('Initial Multi-UAV Placement (1st Quadrant, R > 1500)');
% axis equal; grid on;