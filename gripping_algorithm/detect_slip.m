function slip_flg = detect_slip(R_jtc, touch_flg)

    % --- 현재 조인트 전류 ---
    joint0_curr = R_jtc(1);
    joint1_curr = R_jtc(2);
    joint2_curr = R_jtc(3);

    % --- 임계값 설정 ---
    % 슬립 시 전류 크기가 거의 0에 가까워짐
    near_zero_threshold = 2e-5;         % [A]
    slip_drop_threshold = -1.0e-4;      % [A] 전류 급감

    persistent prev_R_jtc
    persistent is_slipping

    if isempty(prev_R_jtc)
        prev_R_jtc = zeros(3,1);
    end 

    % --- 전류 변화량 계산 ---
    dI_joint0 = joint0_curr - prev_R_jtc(1);
    dI_joint1 = joint1_curr - prev_R_jtc(2);
    dI_joint2 = joint2_curr - prev_R_jtc(3);
    

    if touch_flg ==1 && dI < -1e-04
        if  abs(I) > slip_threshold
            slip_going_flg =1;
        else abs(I) < slip_complete_th
            slip_comp_flg =1;
        end
    end


    % --- 전류 업데이트 ---
    prev_R_jtc = R_jtc;
end