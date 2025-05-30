close all; clear all; clc;

% 시뮬레이션 시간: 10 step
n = 10;

% touch_flg 시퀀스
touch_flags = [0 0 0 1 1 1 1 1 1 1];

% R_jtc 시퀀스 (3 x 10 matrix)
R_jtc_series = [
    % joint0    joint1     joint2
    0     ,    0     ,    0     ;   % t = 1 (no touch)
    0     ,    0     ,    0     ;   % t = 2
    0     ,    0     ,    0     ;   % t = 3
    1.1e-4,  4.5e-5,  4.2e-5 ;   % t = 4 (touch begins)
    1.3e-4,  4.8e-5,  4.4e-5 ;   % t = 5
    1.2e-4,  5.0e-5,  4.5e-5 ;   % t = 6 (firmly grasped)
    1.3e-4,  4.9e-5,  3.5e-5 ;   % t = 7 (joint2 starts slipping)
    1.2e-4,  4.7e-5,  3.5e-5 ;   % t = 8
    1.1e-4,  4.5e-5,  3.5e-5 ;   % t = 9
    1.0e-4,  4.3e-5,  3.5e-5 ;   % t = 10
];

% 결과 저장
slip_flg = zeros(n,1);

for t = 1:n
    R_jtc = R_jtc_series(t, :);
    touch_flg = touch_flags(t);
    
    slip_flg(t) = detect_slip(R_jtc, touch_flg);
    
    fprintf("t = %d | touch = %d | slip = %d\n", t, touch_flg, slip_flg(t));
end

% 시각화
figure;
subplot(2,1,1);
plot(1:n, touch_flags, '-o'); ylabel('Touch Flag'); ylim([-0.1, 1.1]);
title('Touch Flag Over Time');

subplot(2,1,2);
plot(1:n, slip_flg, '-or'); ylabel('Slip Detected'); xlabel('Time Step');
ylim([-0.1, 1.1]);
title('Slip Detection Over Time');

function slip_flg = detect_slip(R_jtc, touch_flg)

    % --- 조인트 전류 ---
    joint0_curr = R_jtc(1);
    joint1_curr = R_jtc(2);
    joint2_curr = R_jtc(3);

    % --- 슬립 탐지 기준 임계값 ---
    joint0_curr_limit = 1e-4;
    joint1_curr_limit = 4e-5;
    joint2_curr_limit = 4e-5;
    slip_threshold = 3e-5;

    persistent prev_R_jtc
    persistent is_sliping

    if isempty(prev_R_jtc)
        prev_R_jtc = zeros(1,3);  % 초기화
        is_sliping = 0;
    end

    % 전류 변화량 계산
    dI_joint2 = R_jtc(3) - prev_R_jtc(3);
    slip_flg = 0;

    % 슬립 조건 판단
     if touch_flg == 1 && ...
        joint0_curr > joint0_curr_limit && ...
        joint1_curr > joint1_curr_limit && ...
        prev_R_jtc(3) > joint2_curr_limit 

        if  dI_joint2 < -0.8e-5

            is_sliping =1;
        end

        if is_sliping
            slip_flg = 1;
        else
            slip_flg = 0;
        end
     end
 
    % 이전 값 업데이트
    prev_R_jtc = R_jtc;

end
