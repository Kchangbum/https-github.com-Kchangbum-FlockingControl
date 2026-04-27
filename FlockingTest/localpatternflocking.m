% Cucker-Smale 모델 기반 삼각형 패턴 및 경로 추적 시뮬레이션 (Alpha 오류 수정본)
clear; clc; close all;

%% 1. 매개변수 설정
N = 10;             % 개체 수
dt = 0.1;          % 시간 간격
T_end = 300;         % 총 시뮬레이션 시간
steps = T_end / dt;

kappa = 4.0;        % 결합 강도 (Alignment)
mu = 4.5;           % 구동력 강도 (Pattern Formation)

%% 2. 초기 상태 설정
pos = rand(N, 2) * 10;    % 초기 위치
vel = (rand(N, 2) - 0.5) * 4; % 초기 속도

% 삼각형 패턴 벡터 (V-shape)
e_left = [-0.7, -0.7]; e_left = e_left / norm(e_left);
e_right = [0.7, -0.7]; e_right = e_right / norm(e_right);

%% 3. 시각화 및 경로 추적 준비
figure('Color', 'w', 'Position', [100, 100, 900, 700]);
hold on; grid on; axis equal;
xlabel('Position X'); ylabel('Position Y');
title('Pattern Formation with Path Tracking (V-shape)');

% 경로를 그리기 위한 animatedline 객체 생성
colors = lines(N); 
for i = 1:N
    % 'Alpha' 속성을 제거하여 오류 해결
    path_line(i) = animatedline('Color', colors(i,:), 'LineWidth', 1);
end

% 현재 위치를 표시할 scatter 객체
h_plot = scatter(pos(:,1), pos(:,2), 60, colors, 'filled', 'MarkerEdgeColor', 'k');
% 리더(1번) 강조
h_leader = scatter(pos(1,1), pos(1,2), 100, 'r', 'filled', 'MarkerEdgeColor', 'k');

%% 4. 메인 루프
for t_step = 1:steps
    new_vel = vel;
    
    for i = 1:N
        % (1) Cucker-Smale Alignment
        alignment = [0, 0];
        for j = 1:N
            dist = norm(pos(j,:) - pos(i,:));
            psi = 1 / (1 + dist^2)^0.5; 
            alignment = alignment + (kappa/N) * psi * (vel(j,:) - vel(i,:));
        end
        
        % (2) Pattern Driving Force (삼각형 구동력)
        if i == 1
            F_i = [0, 0];
        else
            target_e = e_right;
            if mod(i, 2) == 0, target_e = e_left; end
            
            rel_pos = pos(i,:) - pos(1,:); 
            % 논문의 핵심 공식: F_i = <rel_pos, e>e - rel_pos
            F_i = dot(rel_pos, target_e) * target_e - rel_pos;
        end
        
        % (3) 가속도 계산
        accel = alignment + mu * F_i;
        new_vel(i,:) = vel(i,:) + accel * dt;
    end
    
    % 위치 업데이트
    pos = pos + new_vel * dt;
    vel = new_vel;
    
    % --- 경로 업데이트 ---
    for i = 1:N
        addpoints(path_line(i), pos(i,1), pos(i,2));
    end
    
    % --- 시각화 갱신 ---
    if mod(t_step, 4) == 0
        set(h_plot, 'XData', pos(:,1), 'YData', pos(:,2));
        set(h_leader, 'XData', pos(1,1), 'YData', pos(1,2));
        
        % 리더 추적 모드
        xlim([pos(1,1)-20, pos(1,1)+20]);
        ylim([pos(1,2)-20, pos(1,2)+20]);
        drawnow limitrate; 
    end
end