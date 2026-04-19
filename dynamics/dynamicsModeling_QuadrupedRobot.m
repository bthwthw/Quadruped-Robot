% =========================================================================
% MÔ HÌNH ĐỘNG HỌC & ĐỘNG LỰC HỌC CHÂN ROBOT 2-DOF (PLANAR)
% =========================================================================
%% 1. KHAI BÁO BIẾN TRẠNG THÁI
syms q1 q2 dq1 dq2 real

q = [q1; q2];       % Vector vị trí góc
dq = [dq1; dq2];    % Vector vận tốc góc

%% 2. THÔNG SỐ HÌNH HỌC & KHỐI TÂM (CoM)
% Chiều dài (m)
L1 = 0.2;       
L2 = 0.2;       
Lc1 = 0.1;      
Lc2 = 0.1;      

% Khối lượng (kg) & Trọng trường
m1 = 1.0;       
m2 = 0.5;       
g = 9.81;       

% Mô-men quán tính (Tính theo khối trụ Cylinder - Trục quay Z)
r1 = 0.02;  h1 = 0.2;
r2 = 0.015; h2 = 0.2;
I1 = (1/12) * m1 * (3*r1^2 + h1^2); % ~0.003433 kg.m^2
I2 = (1/12) * m2 * (3*r2^2 + h2^2); % ~0.001695 kg.m^2

% =========================================================================
% PHẦN 1: ĐỘNG HỌC (KINEMATICS)
% =========================================================================

% 1. Phương trình vị trí khối tâm (Mặt phẳng Oxy)
r1_pos = [Lc1*cos(q1); 
          Lc1*sin(q1)];
      
r2_pos = [L1*cos(q1) + Lc2*cos(q1+q2); 
          L1*sin(q1) + Lc2*sin(q1+q2)];

% 2. Ma trận Jacobian Tịnh tiến (Jv) [Kích thước 2x2]
Jv1 = simplify(jacobian(r1_pos, q));
Jv2 = simplify(jacobian(r2_pos, q));

% 3. Vận tốc tịnh tiến
v1 = simplify(Jv1 * dq);
v2 = simplify(Jv2 * dq);

% 4. Ma trận Jacobian Quay (Jw) [Kích thước 1x2 - Chỉ quay quanh trục Z]
Jw1 = [1, 0];
Jw2 = [1, 1];

% 5. Vận tốc góc vô hướng
w1 = Jw1 * dq; % w1 = dq1
w2 = Jw2 * dq; % w2 = dq1 + dq2

% =========================================================================
% PHẦN B: ĐỘNG LỰC HỌC (DYNAMICS) - EULER-LAGRANGE
% =========================================================================

% 1. Ma trận Khối lượng / Quán tính M(q) [Kích thước 2x2]
M11 = m1*Lc1^2 + I1 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*cos(q2)) + I2;
M12 = m2*(Lc2^2 + L1*Lc2*cos(q2)) + I2;
M21 = M12; % Có tính đối xứng
M22 = m2*Lc2^2 + I2;

M = [M11, M12; 
     M21, M22];

% 2. Ma trận Coriolis và lực Ly tâm C(q, dq) [Kích thước 2x2]
h_coriolis = -m2*L1*Lc2*sin(q2);

C11 = h_coriolis * dq2;
C12 = h_coriolis * (dq1 + dq2);
C21 = -h_coriolis * dq1;
C22 = 0;

C = [C11, C12; 
     C21, C22];

% 3. Vector Trọng trường G(q) [Kích thước 2x1]
G1 = (m1*Lc1 + m2*L1)*g*cos(q1) + m2*Lc2*g*cos(q1+q2);
G2 = m2*Lc2*g*cos(q1+q2);

G = [G1; 
     G2];

% =========================================================================
% XUẤT KẾT QUẢ VÀ LƯU FILE BÀN GIAO
% =========================================================================

disp('---------------------------------------------------------');
disp('ĐÃ TÍNH TOÁN XONG MÔ HÌNH ĐỘNG HỌC & ĐỘNG LỰC HỌC!');
disp('Các ma trận cốt lõi: M, C, G đã sẵn sàng.');
disp('---------------------------------------------------------');

% Tự động lưu các biến quan trọng ra file .mat để Thành viên C sử dụng
save('MoHinh_ChanRobot_2DOF.mat', 'M', 'C', 'G', 'q1', 'q2', 'dq1', 'dq2');
