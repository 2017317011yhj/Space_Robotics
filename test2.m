clear all;
close all;
clc;

theta = [-50; 20; -40]*pi/180;  % 조인트 명령각 (rad)

a1 = sqrt(0.04494^2 + 0.08979^2);
a2 = 0.1665;
a3 = sqrt(0.02945^2 + 0.1232^2);
% fk_3link([0 0 0],a1,a2,a3)
segfwgesrgrsedgggggggggggggggggggggggggggggggggg
theta1_offset = -atan2(0.04494,0.08979);
theta2_offset =  atan2(0.04494,0.08979);
theta3_offset = -atan2(0.02945, 0.1232);
t_offset = [theta1_offset, theta2_offset, theta3_offset]';

[ee_pos, res0] = fk_3link_explicit(theta);
% ee_pos = [0.2 0.2]';
% solutions = ik_3link_phi_free(ee_pos(1), ee_pos(2), a1, a2, a3);
% [solutions,ee_pos_mod] = ik_3link_no_phi(ee_pos(1), ee_pos(2), a1, a2, a3);
[solutions, pos_list, err, iter_fin] = fabrik_3link(ee_pos, 0.0001, 100);

[res1, ee_pos1] = fk_3link(solutions,a1,a2,a3);
% temp_ = [solutions(1), solutions(1)+solutions(2), solutions(1)+solutions(2)+solutions(3)]';
[pos, p_total] = fk_3link_explicit(solutions + t_offset);
% [res2, ee_pos2] = fk_3link(solutions(2,:)',a1,a2,a3);
err1 = norm(ee_pos - ee_pos1)
iter_fin


fprintf("End-Effector Position:\n");
figure; hold on; axis equal;
plot(res1(1,:), res1(2,:), 'r-o', 'LineWidth', 2);
plot(p_total(1,:), p_total(2,:), 'b-o', 'LineWidth', 2);
plot(ee_pos(1), ee_pos(2), 'mx', 'MarkerSize',10);
xlabel('X'); ylabel('Y'); title('FABRIK Verification');
legend('simple', 'real');
grid on;
axis equal;
% [pos, p_total] = fk_3link_explicit(theta);
% pos

function [positions, ee_pose] = fk_3link(theta, a1, a2, a3)
    % theta: [theta1, theta2, theta3] (rad)
    % a1, a2, a3: 링크 길이
    % positions: [P0; P1; P2; P3] (각 조인트 위치)
    % ee_pose: [x, y, phi] (EE 위치 + 자세)

    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);

    % 누적 각도
    t1 = theta1;
    t2 = theta1 + theta2;
    t3 = t2 + theta3;

    % 각 조인트 위치 계산
    P0 = [0; 0];
    P1 = P0 + a1 * [cos(t1); sin(t1)];
    P2 = P1 + a2 * [cos(t2); sin(t2)];
    P3 = P2 + a3 * [cos(t3); sin(t3)];

    % 자세
    phi = t3;

    % 출력
    positions = [P0, P1, P2, P3];  % 각 열이 각 위치
    ee_pose = [P3(1), P3(2)]'; % EE 위치 및 방향
end

function [solution, ee_mod] = ik_3link_no_phi(px, py, a1, a2, a3)
    % px, py: 목표 위치 (엔드이펙터)
    % a1, a2, a3: 링크 길이

    % 링크3을 위치만 고려하여 병합 (2링크 모델로 전환)
    L = a1 + a2 + a3;

    % 거리 계산
    r_squared = px^2 + py^2;
    r = sqrt(r_squared);

    if r > (a1 + L)
        % error('목표 위치가 작업 공간을 벗어났습니다.');
        temp = [px;py]/norm([px,py]);
        px = temp(1)*(a1+a2+a3);
        py = temp(2)*(a1+a2+a3);
        r_squared = px^2 + py^2;
        ee_mod = [px,py]';

    
    else
        ee_mod = [px,py]';
    end
    % L = a1 + a2 + a3;


    % 출력
    fprintf('엘보우 다운:\nθ1 = %.3f, θ2 = %.3f, θ3 = %.3f\n', theta1_down, theta2_down, theta3_down);
    fprintf('엘보우 업:\n  θ1 = %.3f, θ2 = %.3f, θ3 = %.3f\n', theta1_up, theta2_up, theta3_up);

    % 반환
    theta1 = [theta1_down, theta1_up]';
    theta2 = [theta2_down, theta2_up]';
    theta3 = [theta3_down, theta3_up]';

    solution = [theta1,theta2,theta3];
end

function [q_list, pos_list, err, iter_fin] = fabrik_3link(target, tol, max_iter)
% target: [x; y] 목표 위치
% L: [L1, L2, L3] 링크 길이
% tol: 허용 오차 (예: 1e-4)
% max_iter: 반복 한계 (예: 100)

L = ...
[sqrt(0.04494^2 + 0.08979^2);
 0.1665;
 sqrt(0.02945^2 + 0.1232^2)];


[positions, ee_pose] = fk_3link([-50,0,0]'*pi/180, L(1), L(2), L(3));

% 초기 관절 위치 (x, y) 초기화
pos_list = zeros(2, 4); % P0, P1, P2, P3
pos_list(:,1) = [0; 0];                    % Base
pos_list(:,2) = positions(:,1) + [L(1); 0]; % P1
pos_list(:,3) = positions(:,2) + [L(2); 0]; % P2
pos_list(:,4) = positions(:,3) + [L(3); 0]; % EE (초기화)

% Base 저장
base = pos_list(:,1);

% 목표 거리
total_length = sum(L);
if norm(target - base) > total_length
    % error("목표가 작업공간을 벗어났습니다.");
    target = total_length*(target/norm(target));
end

iter_fin = 0;

for iter = 1:max_iter
    % 1. FORWARD REACHING
    pos_list(:,4) = target; % 끝단 고정
    for i = 3:-1:1
        r = norm(pos_list(:,i+1) - pos_list(:,i));
        lambda = L(i) / r;
        pos_list(:,i) = (1 - lambda) * pos_list(:,i+1) + lambda * pos_list(:,i);
    end

    % 2. BACKWARD REACHING
    pos_list(:,1) = base; % base 고정
    for i = 1:3
        r = norm(pos_list(:,i+1) - pos_list(:,i));
        lambda = L(i) / r;
        pos_list(:,i+1) = (1 - lambda) * pos_list(:,i) + lambda * pos_list(:,i+1);
    end

    % 3. 수렴 확인
    err = norm(pos_list(:,end) - target);
    if err < tol
        break;
    end
    iter_fin = iter;
end

% P0 = joint_pos(:,1); P1 = joint_pos(:,2);
P2 = pos_list(:,2); P3 = pos_list(:,3);
% P4 = joint_pos(:,5); 
P5 = pos_list(:,4);

t0 = atan2(P2(2), P2(1));
temp = P3 - P2;
t1 = atan2(temp(2), temp(1)) - t0;
temp = P5 - P3;
t2 = atan2(temp(2), temp(1)) - t0 - t1;

q_list = [t0,t1,t2]';

end

function [pos, res] = fk_3link_with_offsets(theta)
    % 링크 길이 [m]

    L1 = sqrt(0.04494^2 + 0.08979^2);
    L2 = 0.1665;
    L3 = sqrt(0.02945^2 + 0.1232^2);

    % 조인트 오프셋 [rad]
    theta1_offset = atan2(0.04494,0.08979);
    theta2_offset = 0;
    theta3_offset = atan2(0.02945, 0.1232);

    % 입력 각도 (조인트 명령값)
    t1 = theta(1) + theta1_offset;
    t2 = theta(2) + theta2_offset;
    t3 = theta(3) + theta3_offset;

    % 누적 각도
    t12 = t1 + t2;
    t123 = t12 + t3;

    % 포지션 계산
    P0 = [0 0]';
    P1 = L1 * [cos(t1); sin(t1)];
    P2 = P1 + L2 * [cos(t2); sin(t2)];
    P3 = P2 + L3 * [cos(t3); sin(t3)];
    res = [P0, P1, P2, P3];
    % EE 위치 반환
    pos = P3;
end

function solutions = ik_3link_phi_free(px, py, a1, a2, a3)
    % φ 생략: θ3 = 0으로 고정
    % Wrist 위치
    wx = px - a3;  % θ3 = 0 → cos(0) = 1
    wy = py;
    a = 0.08979 + 0.1665 + 0.1232;
    b = 0.04494 + 0.02945;    
    % 거리 기반 D 계산
    % D = (wx^2 + wy^2 - a1^2 - a2^2) / (2 * a1 * a2);
    D_limit = sqrt(a^2 + b^2);
    D = sqrt(px*px + py*py);
    % 실현 가능 여부 확인
    if abs(D) > D_limit
        error('목표 위치가 작업 범위를 벗어났습니다.');
    end

    theta2_set = [atan2(sqrt(1 - D^2), D), atan2(-sqrt(1 - D^2), D)];
    solutions = zeros(2, 3);

    for i = 1:2
        theta2 = theta2_set(i);
        k1 = a1 + a2 * cos(theta2);
        k2 = a2 * sin(theta2);
        theta1 = atan2(wy, wx) - atan2(k2, k1);
        theta3 = 0;  % 고정

        solutions(i,:) = [wrapToPi(theta1), wrapToPi(theta2), theta3];
    end
end

function [pos, p_total] = fk_3link_explicit(theta)
    L0 = 0.04494;
    L1 = 0.08979;
    L2 = 0.1665;
    L3a = 0.02945;
    L3b = 0.1232;

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