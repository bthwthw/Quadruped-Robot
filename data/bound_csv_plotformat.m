% =========================================================
% SCRIPT VẼ ĐỒ THỊ TỪ FILE CSV DÁNG ĐI WALK
% =========================================================

% Thiết lập font chữ chuẩn LaTeX cho biểu đồ
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');

% 1. Đọc dữ liệu từ file CSV
data = readtable('bound.csv');
t = data.timestamp_s;

% Mã màu phân biệt cho 4 chân (Thống nhất trong toàn bộ biểu đồ)
c_FL = '#EDB120'; % Vàng (Front-Left)
c_FR = '#0072BD'; % Xanh dương (Front-Right)
c_BL = '#D95319'; % Cam (Back-Left)
c_BR = '#77AC30'; % Xanh lá (Back-Right)

% =========================================================
% FIGURE 1: ĐÁP ỨNG VỊ TRÍ (POSITION TRACKING)
% =========================================================
figure('Name', 'Đáp ứng vị trí - bound Gait', 'Color', 'w', 'Units', 'centimeters', 'Position', [2 5 22 16]);

% --- Subplot 1: Khớp Háng (Hip Joints) ---
subplot(2, 1, 1);
hold on; grid on; grid minor;

% Vẽ tín hiệu tham chiếu (q_ref) - Nét đứt mờ
plot(t, data.FL_hip_q_ref, '--', 'Color', [hex2rgb(c_FL) 0.5], 'LineWidth', 1.2);
plot(t, data.FR_hip_q_ref, '--', 'Color', [hex2rgb(c_FR) 0.5], 'LineWidth', 1.2);
plot(t, data.BL_hip_q_ref, '--', 'Color', [hex2rgb(c_BL) 0.5], 'LineWidth', 1.2);
plot(t, data.BR_hip_q_ref, '--', 'Color', [hex2rgb(c_BR) 0.5], 'LineWidth', 1.2);

% Vẽ tín hiệu thực tế (q_real) - Nét liền đậm
h1 = plot(t, data.FL_hip_q_real, '-', 'Color', c_FL, 'LineWidth', 1.5);
h2 = plot(t, data.FR_hip_q_real, '-', 'Color', c_FR, 'LineWidth', 1.5);
h3 = plot(t, data.BL_hip_q_real, '-', 'Color', c_BL, 'LineWidth', 1.5);
h4 = plot(t, data.BR_hip_q_real, '-', 'Color', c_BR, 'LineWidth', 1.5);

xlim([0 5]);
ylabel('Hip Position [rad]', 'FontSize', 12);
title('Đáp ứng quỹ đạo khớp Háng (Hip) - bound Gait', 'FontSize', 13);
legend([h1, h2, h3, h4], {'$q_{FL\_Hip}$', '$q_{FR\_Hip}$', '$q_{BL\_Hip}$', '$q_{BR\_Hip}$'}, ...
    'Location', 'eastoutside', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% --- Subplot 2: Khớp Gối (Knee Joints) ---
subplot(2, 1, 2);
hold on; grid on; grid minor;

% Vẽ tín hiệu tham chiếu (q_ref) - Nét đứt mờ
plot(t, data.FL_knee_q_ref, '--', 'Color', [hex2rgb(c_FL) 0.5], 'LineWidth', 1.2);
plot(t, data.FR_knee_q_ref, '--', 'Color', [hex2rgb(c_FR) 0.5], 'LineWidth', 1.2);
plot(t, data.BL_knee_q_ref, '--', 'Color', [hex2rgb(c_BL) 0.5], 'LineWidth', 1.2);
plot(t, data.BR_knee_q_ref, '--', 'Color', [hex2rgb(c_BR) 0.5], 'LineWidth', 1.2);

% Vẽ tín hiệu thực tế (q_real) - Nét liền đậm
k1 = plot(t, data.FL_knee_q_real, '-', 'Color', c_FL, 'LineWidth', 1.5);
k2 = plot(t, data.FR_knee_q_real, '-', 'Color', c_FR, 'LineWidth', 1.5);
k3 = plot(t, data.BL_knee_q_real, '-', 'Color', c_BL, 'LineWidth', 1.5);
k4 = plot(t, data.BR_knee_q_real, '-', 'Color', c_BR, 'LineWidth', 1.5);

xlim([0 5]);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Knee Position [rad]', 'FontSize', 12);
title('Đáp ứng quỹ đạo khớp Gối (Knee) - bound Gait', 'FontSize', 13);
legend([k1, k2, k3, k4], {'$q_{FL\_Knee}$', '$q_{FR\_Knee}$', '$q_{BL\_Knee}$', '$q_{BR\_Knee}$'}, ...
    'Location', 'eastoutside', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% =========================================================
% FIGURE 2: SAI SỐ BÁM (TRACKING ERROR)
% =========================================================
figure('Name', 'Tracking Error - bound Gait', 'Color', 'w', 'Units', 'centimeters', 'Position', [6 8 22 16]);

% --- Subplot 1: Sai số Khớp Háng (Hip Error) ---
subplot(2, 1, 1);
hold on; grid on; grid minor;

e1 = plot(t, data.FL_hip_error, '-', 'Color', c_FL, 'LineWidth', 1.2);
e2 = plot(t, data.FR_hip_error, '-', 'Color', c_FR, 'LineWidth', 1.2);
e3 = plot(t, data.BL_hip_error, '-', 'Color', c_BL, 'LineWidth', 1.2);
e4 = plot(t, data.BR_hip_error, '-', 'Color', c_BR, 'LineWidth', 1.2);

xlim([0 5]);
ylabel('Hip Error [rad]', 'FontSize', 12);
title('Sai số bám quỹ đạo khớp Háng (Hip Error) - bound Gait', 'FontSize', 13);
legend([e1, e2, e3, e4], {'$e_{FL\_Hip}$', '$e_{FR\_Hip}$', '$e_{BL\_Hip}$', '$e_{BR\_Hip}$'}, ...
    'Location', 'eastoutside', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% --- Subplot 2: Sai số Khớp Gối (Knee Error) ---
subplot(2, 1, 2);
hold on; grid on; grid minor;

ek1 = plot(t, data.FL_knee_error, '-', 'Color', c_FL, 'LineWidth', 1.2);
ek2 = plot(t, data.FR_knee_error, '-', 'Color', c_FR, 'LineWidth', 1.2);
ek3 = plot(t, data.BL_knee_error, '-', 'Color', c_BL, 'LineWidth', 1.2);
ek4 = plot(t, data.BR_knee_error, '-', 'Color', c_BR, 'LineWidth', 1.2);

xlim([0 5]);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Knee Error [rad]', 'FontSize', 12);
title('Sai số bám quỹ đạo khớp Gối (Knee Error) - bound Gait', 'FontSize', 13);
legend([ek1, ek2, ek3, ek4], {'$e_{FL\_Knee}$', '$e_{FR\_Knee}$', '$e_{BL\_Knee}$', '$e_{BR\_Knee}$'}, ...
    'Location', 'eastoutside', 'FontSize', 12);
set(gca, 'FontSize', 12, 'LineWidth', 1.2);

% =========================================================
% Hàm phụ trợ chuyển đổi mã màu HEX sang RGB để làm mờ nét đứt
% =========================================================
function rgb = hex2rgb(hexStr)
    hexStr = strrep(hexStr, '#', '');
    r = hex2dec(hexStr(1:2)) / 255;
    g = hex2dec(hexStr(3:4)) / 255;
    b = hex2dec(hexStr(5:6)) / 255;
    rgb = [r g b];
end