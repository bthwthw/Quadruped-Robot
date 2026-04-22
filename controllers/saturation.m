function u_sw = saturation(s, Phi)
u_sw = zeros(2,1); % Khởi tạo vector đầu ra 2 phần tử
    for i = 1:2
        if abs(s(i)) > Phi(i,i)
            u_sw(i) = sign(s(i));
        else
            u_sw(i) = s(i) / Phi(i,i);
        end
    end
end