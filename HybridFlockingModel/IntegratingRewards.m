% 1. 경로 및 설정
clear; close all; clc;

% 1. 경로 및 설정
folders = {'8-4-3', '8-4-4'};
fileName = 'Agents2000.mat';
numAgents = 5;

all_EpisodeReward_Total = []; % 배경용 (전체 에이전트 평균 에피소드 보상)
agentAvgRewards = cell(1, numAgents); % 에이전트별 평균 보상

for i = 1:length(folders)
    fullPath = fullfile(pwd, folders{i}, fileName);
    
    if exist(fullPath, 'file')
        fprintf('[읽기 중] %s...\n', fullPath);
        data = load(fullPath);
        
        if isfield(data, 'savedAgentResult')
            res = data.savedAgentResult;
            
            % 배경용: 5개 에이전트의 에피소드 보상 평균 (1줄로 만들기)
            all_EpisodeReward_Total = [all_EpisodeReward_Total; mean(res.EpisodeReward, 2)];
            
            % 에이전트별: 개별 Average Reward 누적
            for a = 1:numAgents
                agentAvgRewards{a} = [agentAvgRewards{a}; res.AverageReward(:, a)];
            end
            fprintf('[성공] %d 에피소드 구간 통합 완료.\n', size(res.AverageReward, 1));
        end
    end
end

% 2. 시각화 (배경 노이즈 + 개별 추세선)
if ~isempty(all_EpisodeReward_Total)
    figure('Color', 'w', 'Position', [100, 100, 1000, 600]);
    hold on;
    
    % (1) 배경: 전체 에피소드 보상 (연한 회색, 투명도 조절)
    hExp = plot(all_EpisodeReward_Total, 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5);
    hExp.Annotation.LegendInformation.IconDisplayStyle = 'off'; % 범례에서 제외 (너무 지저분함)
    
    % (2) 전면: 에이전트별 Average Reward (서로 다른 색상)
    colors = lines(numAgents); 
    for a = 1:numAgents
        plot(agentAvgRewards{a}, 'Color', colors(a, :), 'LineWidth', 2, ...
            'DisplayName', sprintf('Agent %d (Avg)', a));
    end
    
    % (3) 전체 평균 추세선 (필요하다면 추가, 아니면 주석 처리)
    % totalAvg = mean(horzcat(agentAvgRewards{:}), 2);
    % plot(totalAvg, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Overall Swarm Avg');
    
    grid on;
    xlabel('Total Episodes', 'FontSize', 12);
    ylabel('Reward Score', 'FontSize', 12);
    title('Multi-Agent Training Performance Analysis', 'FontSize', 14);
    
    % 범례 설정 (에이전트별 선만 표시됨)
    legend('Location', 'northeastoutside');
    
    fprintf('=== 통합 시각화 완료! 그래프를 확인하세요. ===\n');
end