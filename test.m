clear all;
close all;
clc;
% 검증 스크립트

% Ground truth joint angles
theta_true = deg2rad([20, 0, 0]);

% Target EE position via FK
[pos_true, p_total_init] = fk_3link_explicit(theta_true);
% pos_true = [0.35 0]';
% Run FABRIK using initial theta and pos_true as goal
tol = 1e-2;
max_iter = 100;

[theta_est, joint_pos] = fabrik_ik_custom_with_theta(theta_true, pos_true, tol, max_iter);

% Recompute EE position using estimated theta
[pos_est, p_total] = fk_3link_explicit(theta_est);

% Compare
fprintf('True EE pos:     [%.6f, %.6f]\n', pos_true(1), pos_true(2));
fprintf('Estimated EE pos:[%.6f, %.6f]\n', pos_est(1), pos_est(2));
fprintf('Position error:  %.6e\n', norm(pos_est - pos_true));

% Visualize
figure; hold on; axis equal;
plot(p_total_init(1,:), p_total_init(2,:), 'r-o', 'LineWidth', 2);
plot(p_total(1,:), p_total(2,:), 'b-o', 'LineWidth', 2);
plot(pos_true(1), pos_true(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X'); ylabel('Y'); title('FABRIK Verification');
legend('init', 'res');
grid on;
axis equal;

% --- Forward Kinematics 함수
function [pos, p_total] = fk_3link_explicit(theta)
    L0 = 0.04494;
    L1 = 0.08979;
    L2 = 0.1665;
    L3a = 0.02945;
    L3b = 0.1232;
    t_0C = atan2(L0, L1);
    t_2C = atan2(L3a, L3b);

    t0 = theta(1);
    t1 = theta(2);
    t2 = theta(3);
    t01 = t0 + t1;
    t012 = t01 + t2;

    R = @(theta) [cos(theta); sin(theta)];

    P0 = [0; 0];
    P1 = P0 + L0 * R(t0 + pi/2);
    P2 = P1 + L1 * R(t0);
    P3 = P2 + L2 * R(t01);
    P4 = P3 + L3a * R(t012 + pi/2);
    P5 = P4 + L3b * R(t012);
    pos = P4 + L3b * R(t012);  % EE 위치

    p_total = [P0, P1, P2, P3, P4, P5];

end

% --- FABRIK 함수 (각도 역산 포함)
function [theta_sol, joint_pos] = fabrik_ik_custom_with_theta(theta_init, target, tol, max_iter)
    % 링크 길이
    L0 = 0.04494;
    L1 = 0.08979;
    L2 = 0.1665;
    L3a = 0.02945;
    L3b = 0.1232;
    L = [L0, L1, L2, L3a, L3b];

    t_0C = atan2(L0, L1);
    t_2C = atan2(L3a, L3b);

    % 초기 관절 각
    % t0 = theta_init(1);
    % t1 = theta_init(2);
    % t2 = theta_init(3);
    t0 = 45*pi/180;
    t1 = 0*pi/180;
    t2 = 0*pi/180;

    t01 = t0 + t1;
    t012 = t01 + t2;
    R = @(theta) [cos(theta); sin(theta)];

    % 초기 조인트 위치
    P0 = [0; 0];
    P1 = P0 + L0 * R(t0 + pi/2);
    P2 = P1 + L1 * R(t0);
    P3 = P2 + L2 * R(t01);
    P4 = P3 + L3a * R(t012 + pi/2);
    P5 = P4 + L3b * R(t012);
    joint_pos = [P0, P1, P2, P3, P4, P5];

    % FABRIK
    for iter = 1:max_iter
        % Backward
        joint_pos(:,6) = target(:);
        for i = 5:-1:1
            r = norm(joint_pos(:,i+1) - joint_pos(:,i));
            lambda = L(i) / r;
            joint_pos(:,i) = (1 - lambda) * joint_pos(:,i+1) + lambda * joint_pos(:,i);
        end
        % Forward
        P0 = joint_pos(:,1); P1 = joint_pos(:,2);
        P2 = joint_pos(:,3); P3 = joint_pos(:,4);
        P4 = joint_pos(:,5); P5 = joint_pos(:,6);
        t0 = atan2(P2(2), P2(1)) - t_0C;
        temp = P3 - P2;
        t1 = atan2(temp(2), temp(1)) - t0;
        temp = P5 - P3;
        t2 = atan2(temp(2), temp(1)) - t_2C - t0 - t1;
        [pos, p_total] = fk_3link_explicit([t0,t1,t2]);

        P0 = p_total(:,1);        P1 = p_total(:,2);
        P2 = p_total(:,3);        P3 = p_total(:,4);
        P4 = p_total(:,5);        P5 = p_total(:,6);
        joint_pos(:,1) = P0;        joint_pos(:,2) = P1;
        joint_pos(:,3) = P2;        joint_pos(:,4) = P3;
        joint_pos(:,5) = P4;        joint_pos(:,6) = P5;
        
        for i = 2:6
            r = norm(joint_pos(:,i) - joint_pos(:,i-1));
            lambda = L(i-1) / r;
            joint_pos(:,i) = (1 - lambda) * joint_pos(:,i-1) + lambda * joint_pos(:,i);
        end
        P0 = joint_pos(:,1); P1 = joint_pos(:,2);
        P2 = joint_pos(:,3); P3 = joint_pos(:,4);
        P4 = joint_pos(:,5); P5 = joint_pos(:,6);
        
        t0 = atan2(P2(2), P2(1)) - t_0C;
        temp = P3 - P2;
        t1 = atan2(temp(2), temp(1)) - t0;
        temp = P5 - P3;
        t2 = atan2(temp(2), temp(1)) - t_2C - t0 - t1;
        [pos, p_total] = fk_3link_explicit([t0,t1,t2]);

        P0 = p_total(:,1);        P1 = p_total(:,2);
        P2 = p_total(:,3);        P3 = p_total(:,4);
        P4 = p_total(:,5);        P5 = p_total(:,6);
        joint_pos(:,1) = P0;        joint_pos(:,2) = P1;
        joint_pos(:,3) = P2;        joint_pos(:,4) = P3;
        joint_pos(:,5) = P4;        joint_pos(:,6) = P5;
        iter
        if norm(joint_pos(:,6) - target(:)) < tol
            break;
        end

    end

    % ✅ 각도 역추산 (fk_3link_explicit에 맞춰 정확히)
    P0 = joint_pos(:,1); P1 = joint_pos(:,2);
    P2 = joint_pos(:,3); P3 = joint_pos(:,4);
    P4 = joint_pos(:,5); P5 = joint_pos(:,6);
    
    t0 = atan2(P2(2), P2(1)) - t_0C;
    temp = P3 - P2;
    t1 = atan2(temp(2), temp(1)) - t0;
    temp = P5 - P3;
    t2 = atan2(temp(2), temp(1)) - t_2C - t0 - t1;

    theta_sol = [t0, t1, t2];
end

