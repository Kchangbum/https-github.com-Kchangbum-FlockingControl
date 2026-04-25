clear; close all; clc;

load("figdata.mat");

des_water_level = out.fig_data.Data(:,1);
water_level = out.fig_data.Data(:,2);
flow = out.fig_data.Data(:,3);
reward = out.reward.Data(:,1);
t = out.fig_data.Time(:,1);

figure;

subplot(3,1,1);
plot(t, des_water_level, 'r', 'DisplayName', 'Desired Water Level'); hold on;
plot(t, water_level, 'b--', 'DisplayName', 'Actual Water Level');
legend show;
ylim([-20 20]);
ylabel('Water Level / Flow');
title('Water Level and Flow Over Time');

subplot(3,1,2);
plot(t, flow, 'b', 'DisplayName', 'Flow');

ylim([-20 20]);
legend show;
ylabel('flow');
title('Flow');

subplot(3,1,3);
plot(t, reward, 'b', 'DisplayName', 'Reward');
legend show;

ylim([-20 20]);
xlabel('Time');
ylabel('Reward');
title('Reward Over Time');

