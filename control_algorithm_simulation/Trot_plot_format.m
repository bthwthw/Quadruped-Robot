% ===== Lấy dữ liệu từ Scope =====
% Cập nhật đúng tên biến trong workspace của bạn (ví dụ: out.ScopeDataTH1)
t = out.ScopeDataTH1.time;

% ===== Tách tín hiệu đồ thị trên (Position) =====
% Dữ liệu có kích thước [8 x 1 x N], dùng squeeze để lấy vector [N x 1] dọc theo chiều thứ 3
y_top1 = squeeze(out.ScopeDataTH1.signals(1).values(1, 1, :)); % FL Hip
y_top2 = squeeze(out.ScopeDataTH1.signals(1).values(2, 1, :)); % FL Knee
y_top3 = squeeze(out.ScopeDataTH1.signals(1).values(3, 1, :)); % FR Hip
y_top4 = squeeze(out.ScopeDataTH1.signals(1).values(4, 1, :)); % FR Knee
y_top5 = squeeze(out.ScopeDataTH1.signals(1).values(5, 1, :)); % BL Hip
y_top6 = squeeze(out.ScopeDataTH1.signals(1).values(6, 1, :)); % BL Knee
y_top7 = squeeze(out.ScopeDataTH1.signals(1).values(7, 1, :)); % BR Hip
y_top8 = squeeze(out.ScopeDataTH1.signals(1).values(8, 1, :)); % BR Knee

% ===== Tách tín hiệu đồ thị dưới (Control Input) =====
% Dữ liệu có kích thước [8 x 1 x N]
y_bot1 = squeeze(out.ScopeDataTH1.signals(2).values(1, 1, :)); % FL tau1
y_bot2 = squeeze(out.ScopeDataTH1.signals(2).values(2, 1, :)); % FL tau2
y_bot3 = squeeze(out.ScopeDataTH1.signals(2).values(3, 1, :)); % FR tau1
y_bot4 = squeeze(out.ScopeDataTH1.signals(2).values(4, 1, :)); % FR tau2
y_bot5 = squeeze(out.ScopeDataTH1.signals(2).values(5, 1, :)); % BL tau1
y_bot6 = squeeze(out.ScopeDataTH1.signals(2).values(6, 1, :)); % BL tau2
y_bot7 = squeeze(out.ScopeDataTH1.signals(2).values(7, 1, :)); % BR tau1
y_bot8 = squeeze(out.ScopeDataTH1.signals(2).values(8, 1, :)); % BR tau2

% ===== LaTeX mặc định =====
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');

% ===== Figure =====
figure('Color','w');
set(gcf, 'Units','centimeters', 'Position',[5 5 20 16]); % Tăng chiều rộng để chừa chỗ cho legend

% Mã màu tương đồng với Simulink Scope (Bổ sung thêm 4 màu)
c_yellow = '#EDB120';
c_blue   = '#0072BD';
c_orange = '#D95319';
c_green  = '#77AC30';
c_purple = '#7E2F8E';
c_cyan   = '#4DBEEE';
c_maroon = '#A2142F';
c_black  = '#000000';

% ==========================================
% ===== Plot 1: Đồ thị phía trên =====
% ==========================================
subplot(2, 1, 1);
plot(t, y_top1, 'Color', c_yellow, 'LineWidth', 1.5); hold on;
plot(t, y_top2, 'Color', c_blue,   'LineWidth', 1.5);
plot(t, y_top3, 'Color', c_orange, 'LineWidth', 1.5);
plot(t, y_top4, 'Color', c_green,  'LineWidth', 1.5);
plot(t, y_top5, 'Color', c_purple, 'LineWidth', 1.5);
plot(t, y_top6, 'Color', c_cyan,   'LineWidth', 1.5);
plot(t, y_top7, 'Color', c_maroon, 'LineWidth', 1.5);
plot(t, y_top8, 'Color', c_black,  'LineWidth', 1.5);
grid on;
grid minor;

ylim([-2.1, 1.2]);
ylabel('Position [rad]', 'FontSize', 12); 

% Legend cho đồ thị trên (Đưa ra ngoài đồ thị)
legend({'$q_{FL\_Hip}$', '$q_{FL\_Knee}$', '$q_{FR\_Hip}$', '$q_{FR\_Knee}$', ...
        '$q_{BL\_Hip}$', '$q_{BL\_Knee}$', '$q_{BR\_Hip}$', '$q_{BR\_Knee}$'}, ...
    'Location', 'eastoutside', ...
    'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% ==========================================
% ===== Plot 2: Đồ thị phía dưới =====
% ==========================================
subplot(2, 1, 2);
plot(t, y_bot1, 'Color', c_yellow, 'LineWidth', 1.5); hold on;
plot(t, y_bot2, 'Color', c_blue,   'LineWidth', 1.5);
plot(t, y_bot3, 'Color', c_orange, 'LineWidth', 1.5);
plot(t, y_bot4, 'Color', c_green,  'LineWidth', 1.5);
plot(t, y_bot5, 'Color', c_purple, 'LineWidth', 1.5);
plot(t, y_bot6, 'Color', c_cyan,   'LineWidth', 1.5);
plot(t, y_bot7, 'Color', c_maroon, 'LineWidth', 1.5);
plot(t, y_bot8, 'Color', c_black,  'LineWidth', 1.5);
grid on;
grid minor;
xlim([0 10]);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Control Input [Nm]', 'FontSize', 12);

% Legend cho đồ thị dưới
legend({'$\tau_{FL\_Hip}$', '$\tau_{FL\_Knee}$', '$\tau_{FR\_Hip}$', '$\tau_{FR\_Knee}$', ...
        '$\tau_{BL\_Hip}$', '$\tau_{BL\_Knee}$', '$\tau_{BR\_Hip}$', '$\tau_{BR\_Knee}$'}, ...
    'Location', 'eastoutside', ...
    'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);