function u_sw = saturation(s, K, Phi)
% smc_saturation: Calculates the switching control law with chattering reduction.
%
% Inputs:
%   s   - Sliding surface value (Sai số mặt trượt)
%   K   - Switching gain (Hệ số khuếch đại đóng cắt, K > 0)
%   Phi - Boundary layer thickness (Bề dày lớp biên)
%
% Output:
%   u_sw - Switching control signal (Tín hiệu điều khiển thành phần)

    % Apply saturation function to reduce high-frequency chattering
    if abs(s) > Phi
        % Outside the boundary layer: Act like a normal sign() function
        u_sw = K * sign(s);
    else
        % Inside the boundary layer: Linear behavior to smooth out chattering
        u_sw = K * (s / Phi); 
    end

end