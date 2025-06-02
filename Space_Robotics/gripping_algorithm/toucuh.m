clc; clear; close all;

T = 50;  % 총 시간 프레임 수
num_joints = 3;

%% 우리집 강아지는 복슬 강아지
% 파라미터 설정
speed_err_thresh = 8e-02;
current_spike_thresh = 1e-3;
small_change_thresh = 0.01;

% 초기화
Rv_cmd_data = repmat([0.1; 0.1; 0.1], 1, T);               % 일정한 명령 속도
R_Jd_data   = Rv_cmd_data + 0.01 * randn(3, T);            % 실제 속도 (노이즈 포함)
R_jts_data  = zeros(3, T);                                 % 전류 데이터

% 전류 급증 이벤트 삽입 (joint 2)
R_jts_data(2, 11:20) = 2e-3;

% 결과 저장용
touch_flags_log = zeros(3, T);
touch_system_log = zeros(1, T);

% 초기 상태 저장용
prev_cur = zeros(3, 1);
prev_cmd = [0.1; 0.1; 0.1];  % 첫 명령

for t = 1:T
    des_q_dot = Rv_cmd_data(:, t);
    mea_q_dot = R_Jd_data(:, t);
    mea_cur   = R_jts_data(:, t);

    % 조건 계산
    speed_drop     = abs(des_q_dot - mea_q_dot) > speed_err_thresh;
    current_spike  = abs(mea_cur - prev_cur) > current_spike_thresh;
    command_stable = abs(des_q_dot - prev_cmd) < small_change_thresh;

    % Touch 판단
    touch_flags = speed_drop & current_spike & command_stable;
    touch_system = any(touch_flags);

    % 저장
    touch_flags_log(:, t) = touch_flags;
    touch_system_log(t) = touch_system;

    % 상태 업데이트
    prev_cur = mea_cur;
    prev_cmd = des_q_dot;
end

% 시각화
figure;
subplot(4,1,1);
plot(Rv_cmd_data', '--'); title('Desired Velocity'); ylabel('rad/s');
legend('Joint 1','Joint 2','Joint 3');

subplot(4,1,2);
plot(R_Jd_data'); title('Measured Velocity'); ylabel('rad/s');

subplot(4,1,3);
plot(R_jts_data'); title('Motor Current'); ylabel('A');

subplot(4,1,4);
plot(touch_flags_log'); hold on;
plot(touch_system_log, 'k--', 'LineWidth', 2);
title('Touch Detected'); ylabel('Flag'); xlabel('Time');
legend('Joint 1','Joint 2','Joint 3','System Touch');
