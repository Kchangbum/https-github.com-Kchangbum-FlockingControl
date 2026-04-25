close all; clc;

x = out.Agent1.Data(:,1);
y = out.Agent1.Data(:,2);
z = out.Agent1.Data(:,3);
V = out.Agent1.Data(:,4);
gamma = out.Agent1.Data(:,5);
psi = out.Agent1.Data(:,6);
t= out.Agent1.Time;

fig1 = figure("Name",'Model state');
hold on;
subplot(3,2,1);
p1 = plot(t, x, 'LineWidth', 1.5, 'DisplayName', 'X', 'Color', [0 0 0]);
% grid on; grid minor;
grid off;
xlabel('Time (s)');
ylabel('X (m)');
leg1 = legend('X axis');
leg1.Location = 'north';
leg1.Orientation = 'horizontal';

subplot(3,2,2);
p2 = plot(t, y, 'LineWidth', 1.5, 'DisplayName', 'Y', 'Color', [0 0 0]);
% grid on; grid minor;
grid off;
xlabel('Time (s)');
ylabel('Y (m)');
leg1 = legend('Y axis');
leg1.Location = 'north';
leg1.Orientation = 'horizontal';

subplot(3,2,3);
p3 = plot(t, z, 'LineWidth', 1.5, 'DisplayName', 'Z', 'Color', [0 0 0]);
% grid on; grid minor;
grid off;
xlabel('Time (s)');
ylabel('Z (m)');
leg1 = legend('Z axis');
leg1.Location = 'north';
leg1.Orientation = 'horizontal';

subplot(3,2,4);
p4 = plot(t, V, 'LineWidth', 1.5, 'DisplayName', 'Total Velocity', 'Color', [0 0 0]);
% grid on; grid minor;
grid off;
xlabel('Time (s)');
ylabel('VT (m/s)');
leg1 = legend('Total Velovity axis');
leg1.Location = 'north';
leg1.Orientation = 'horizontal';

subplot(3,2,5);
p5 = plot(t, gamma, 'LineWidth', 1.5, 'DisplayName', '𝛾', 'Color', [0 0 0]);
% grid on; grid minor;
grid off;
xlabel('Time (s)');
ylabel('𝛾 (rad)');
leg1 = legend('𝛾 axis');
leg1.Location = 'north';
leg1.Orientation = 'horizontal';

subplot(3,2,6);
p6 = plot(t, psi, 'LineWidth', 1.5, 'DisplayName', '𝜓', 'Color', [0 0 0]);
% grid on; grid minor;
grid off;
xlabel('Time (s)');
ylabel('𝜓 (rad)');
leg1 = legend('𝜓 axis');
leg1.Location = 'north';
leg1.Orientation = 'horizontal';

sgtitle('Point Mass Model States', 'FontSize', 14, 'FontWeight', 'bold');