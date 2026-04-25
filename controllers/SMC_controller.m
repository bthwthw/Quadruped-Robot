function tau = SMC_controller(q_ref, dq_ref, ddq_ref, q, dq, K_val, lambda_val)
    %#codegen    
    K = [K_val(1) 0; 0 K_val(2)]; % Ma trận hệ số mặt trượt
    lambda = [lambda_val(1) 0; 0 lambda_val(2)]; % Độ lợi điều khiển
    Phi = [0.1 0; 0 0.1];  % Bề dày lớp biên để chống rung 
    
    q1 = q(1); 
    q2 = q(2);
    dq1 = dq(1); 
    dq2 = dq(2);
    
    %% 2. Tính toán Ma trận Động lực học 
    M = [cos(q2)/50 + 0.040128, cos(q2)/100 + 0.0066948;
         cos(q2)/100 + 0.0066948, 0.0066948];
    
    C = [dq2*sin(q2)*(-0.01), sin(q2)*(dq1 + dq2)*(-0.01);
         (dq1*sin(q2))/100, 0];
    
    G = [cos(q1+q2)*0.4905 + cos(q1)*1.962;
         cos(q1+q2)*0.4905];
         
    %% 3. Thuật toán SMC
    e = q - q_ref;      % Sai số vị trí
    de = dq - dq_ref;   % Sai số vận tốc
    s = de + lambda * e;
    
    % Điều khiển tương đương 
    tau_eq = M * (ddq_ref - lambda * de) + C * dq + G;
    
    % Điều khiển bù 
    sat_s = saturation(s, Phi);
    tau_reach = -M * K * sat_s;
    
    %% 4. Đầu ra mô-men xoắn ([2x1])
    tau = tau_eq + tau_reach;
end

function sat = saturation(s, Phi)
    sat = zeros(2,1);
    for i = 1:2
        if s(i) > Phi(i,i)
            sat(i) = 1;
        elseif s(i) < -Phi(i,i)
            sat(i) = -1;
        else
            sat(i) = s(i) / Phi(i,i);
        end
    end
end