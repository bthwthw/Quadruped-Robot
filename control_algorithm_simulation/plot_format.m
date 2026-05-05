% ===== Lấy dữ liệu từ Scope =====
% Cập nhật đúng tên biến trong workspace của bạn là out.ScopeDataTH1
t = out.ScopeDataTH1.time;

% ===== Tách tín hiệu đồ thị trên =====
% Dữ liệu có kích thước [4 x 1 x N], dùng squeeze để lấy vector [N x 1] dọc theo chiều thứ 3
y_top1 = squeeze(out.ScopeDataTH1.signals(1).values(1, 1, :)); % q_ref1 (Vàng)
y_top2 = squeeze(out.ScopeDataTH1.signals(1).values(2, 1, :)); % q_ref2 (Xanh dương)
y_top3 = squeeze(out.ScopeDataTH1.signals(1).values(3, 1, :)); % q1 (Cam)
y_top4 = squeeze(out.ScopeDataTH1.signals(1).values(4, 1, :)); % q2 (Xanh lá)

% ===== Tách tín hiệu đồ thị dưới =====
% Dữ liệu có kích thước [2 x 1 x N]
y_bot1 = squeeze(out.ScopeDataTH1.signals(2).values(1, 1, :)); % SMC Controller:1 (Vàng)
y_bot2 = squeeze(out.ScopeDataTH1.signals(2).values(2, 1, :)); % SMC Controller:2 (Xanh dương)

% ===== LaTeX mặc định =====
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');

% ===== Figure =====
figure('Color','w');
set(gcf, 'Units','centimeters', 'Position',[5 5 18 16]);

% Mã màu tương đồng với Simulink Scope
c_yellow = '#EDB120';
c_blue   = '#0072BD';
c_orange = '#D95319';
c_green  = '#77AC30';

% ==========================================
% ===== Plot 1: Đồ thị phía trên =====
% ==========================================
subplot(2, 1, 1);
plot(t, y_top1, 'Color', c_yellow, 'LineWidth', 1.5); hold on;
plot(t, y_top2, 'Color', c_blue,   'LineWidth', 1.5);
plot(t, y_top3, 'Color', c_orange, 'LineWidth', 1.5);
plot(t, y_top4, 'Color', c_green,  'LineWidth', 1.5);
grid on;
grid minor;
xlim([0 10]); 
ylabel('Position [deg/rad]', 'FontSize', 12); 

% Legend cho đồ thị trên
legend({'$q_{ref1}$', '$q_{ref2}$', '$q_1$', '$q_2$'}, ...
    'Location', 'best', ...
    'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% ==========================================
% ===== Plot 2: Đồ thị phía dưới =====
% ==========================================
subplot(2, 1, 2);
plot(t, y_bot1, 'Color', c_yellow, 'LineWidth', 1.5); hold on;
plot(t, y_bot2, 'Color', c_blue,   'LineWidth', 1.5);
grid on;
grid minor;
xlim([0 10]);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Control Input', 'FontSize', 12);

% Legend cho đồ thị dưới
legend({'$\tau_1$', '$\tau_2$'}, ...
    'Location', 'best', ...
    'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);