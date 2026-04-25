function [methodinfo, structs, enuminfo, ThunkLibName] = webots_proto()
    % Khai báo cấu trúc thư viện rỗng
    ival = {cell(1,0)}; structs = []; enuminfo = []; 
    fcns = struct('name', ival, 'calltype', ival, 'LHS', ival, 'RHS', ival, 'alias', ival);

    % 1. Hàm khởi động Robot
    fcns(1).name = 'wb_robot_init'; 
    fcns(1).calltype = 'cdecl'; fcns(1).LHS = []; fcns(1).RHS = []; 

    % 2. Hàm bước thời gian
    fcns(2).name = 'wb_robot_step'; 
    fcns(2).calltype = 'cdecl'; fcns(2).LHS = 'int32'; fcns(2).RHS = {'int32'}; 

    % 3. Hàm lấy Tag của thiết bị
    fcns(3).name = 'wb_robot_get_device'; 
    fcns(3).calltype = 'cdecl'; fcns(3).LHS = 'uint16'; fcns(3).RHS = {'cstring'}; 

    % 4. Hàm truyền Momen xoắn (Torque)
    fcns(4).name = 'wb_motor_set_torque'; 
    fcns(4).calltype = 'cdecl'; fcns(4).LHS = []; fcns(4).RHS = {'uint16', 'double'}; 

    % 5. Hàm truyền Vị trí (Position)
    fcns(5).name = 'wb_motor_set_position'; 
    fcns(5).calltype = 'cdecl'; fcns(5).LHS = []; fcns(5).RHS = {'uint16', 'double'};

    % 6. Hàm kích hoạt Cảm biến
    fcns(6).name = 'wb_position_sensor_enable'; 
    fcns(6).calltype = 'cdecl'; fcns(6).LHS = []; fcns(6).RHS = {'uint16', 'int32'}; 

    % 7. Hàm đọc Cảm biến
    fcns(7).name = 'wb_position_sensor_get_value'; 
    fcns(7).calltype = 'cdecl'; fcns(7).LHS = 'double'; fcns(7).RHS = {'uint16'}; 

    methodinfo = fcns; ThunkLibName = '';
end