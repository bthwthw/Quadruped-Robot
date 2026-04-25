clc; clear; close all;

%% 1. THIẾT LẬP KẾT NỐI WEBOTS
webots_root = 'C:\Program Files\Webots'; 
    
% Cấp biến môi trường WEBOTS_HOME cho MATLAB
setenv('WEBOTS_HOME', webots_root);

% Bơm đường dẫn chứa file .dll vào biến PATH của Windows
path1 = fullfile(webots_root, 'lib', 'controller');
path2 = fullfile(webots_root, 'msys64', 'mingw64', 'bin');
setenv('PATH', [path1, ';', path2, ';', getenv('PATH')]);

% Thêm thư viện .m 
addpath(fullfile(webots_root, 'lib', 'controller', 'matlab'));

% Khởi động 
wb_robot_init();

fprintf('Đã thiết lập xong đường dẫn Webots!\n');

%% 2. KHAI BÁO THÔNG SỐ ĐỘNG LỰC HỌC (Cho SMC)
% Chiều dài (m)
L1 = 0.2;       
L2 = 0.2;       
Lc1 = 0.1;      
Lc2 = 0.1;      

% Khối lượng (kg)
m1 = 1.0;       
m2 = 0.5;       
g = 9.81;

% Mô-men quán tính
I1 = 0.003433; 
I2 = 0.001695; 

fprintf('Đã nạp xong thông số robot!\n');