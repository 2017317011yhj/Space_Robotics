clear all;
close all;
clc;


Simulation_Time = 200;
Sensing_delay = 0.01;
dt = 0.01;

% % % % % RW % % % % % % % % % 
RW_Init_Speed = 500; %[RPM]
RW_Mass = 10; %[g]

% % % % Manipulator % % % % % % %
Link1_Mass = 300;%gram
Link2_Mass = 200;
Link3_Mass = 100;

U_ARM_joint0_init_angle = -60; %[deg]
D_ARM_joint0_init_angle = -60; %[deg]
R_ARM_joint0_init_angle = -60; %[deg]
L_ARM_joint0_init_angle = -60; %[deg]
U_ARM_joint1_init_angle = 0; %[deg]
D_ARM_joint1_init_angle = 0; %[deg]
R_ARM_joint1_init_angle = 0; %[deg]
L_ARM_joint1_init_angle = 0; %[deg]
U_ARM_joint2_init_angle = 0; %[deg]
D_ARM_joint2_init_angle = 0; %[deg]
R_ARM_joint2_init_angle = 0; %[deg]
L_ARM_joint2_init_angle = 0; %[deg]

U_ARM_joint0_init_angle_rate = 0; %[deg/sec]
D_ARM_joint0_init_angle_rate = 0; %[deg/sec]
R_ARM_joint0_init_angle_rate = 0; %[deg/sec]
L_ARM_joint0_init_angle_rate = 0; %[deg/sec]
U_ARM_joint1_init_angle_rate = 0; %[deg/sec]
D_ARM_joint1_init_angle_rate = 0; %[deg/sec]
R_ARM_joint1_init_angle_rate = 0; %[deg/sec]
L_ARM_joint1_init_angle_rate = 0; %[deg/sec]
U_ARM_joint2_init_angle_rate = 0; %[deg/sec]
D_ARM_joint2_init_angle_rate = 0; %[deg/sec]
R_ARM_joint2_init_angle_rate = 0; %[deg/sec]
L_ARM_joint2_init_angle_rate = 0; %[deg/sec]

% Joint0_down_limit = -170;%[deg]
% Joint0_upper_limit = 170;%[deg]
% Joint1_down_limit = -110;%[deg]
% Joint1_upper_limit = 110;%[deg]
% Joint2_down_limit = -85;%[deg]
% Joint2_upper_limit = 85;%[deg]
Joint0_down_limit = -180;%[deg]
Joint0_upper_limit = 180;%[deg]
Joint1_down_limit = -180;%[deg]
Joint1_upper_limit = 180;%[deg]
Joint2_down_limit = -180;%[deg]
Joint2_upper_limit = 180;%[deg]

% % % arm2target contact % % %
arm2target_stiffness = 10;%N/m
arm2target_damping = 8;%N/(m/s)
arm2target_TRW = 1e-4;%m

arm2target_static_friction = 0.6;
arm2target_dynamic_friction = 0.2;
arm2target_velocity = 1e-4;%m/s

% % % body2target contact % % %
body2target_stiffness = 1000;%N/m
body2target_damping = 100;%N/(m/s)
body2target_TRW = 1e-4;%m

body2target_static_friction = 0.6;
body2target_dynamic_friction = 0.5;
body2target_velocity = 1e-4;%m/s

% % % sernsor2target contact % % %
sensor2target_stiffness = 1000;%N/m
sensor2target_damping = 100;%N/(m/s)
sensor2target_TRW = 1e-4;%m

sensor2target_static_friction = 0.6;
sensor2target_dynamic_friction = 0.5;
sensor2target_velocity = 1e-4;%m/s

% % % % % % Attitude Control Gain % % % % % % %
RW_P = 0.4;
RW_D = 0.2;
RW_I = 0;

flag = 2;

if flag == 1
    % joint_angle_control_mode Gain Parameter
    % % % % joint control init % % %
    % % Right Joint PD Gain
    R_joint0_P = 0.02;
    R_Joint0_D = 0.05;
    R_Joint0_I = 0;
    R_joint1_P = 0.01;
    R_Joint1_D = 0;
    R_Joint1_I = 0.00;
    R_joint2_P = 0.02;
    R_Joint2_D = 0;
    R_Joint2_I = 0;

    % % Left Joint PD Gain
    L_joint0_P = 0.001;
    L_Joint0_D = 0.01;
    L_Joint0_I = 0.001;
    L_joint1_P = 0.001;
    L_Joint1_D = 0.001;
    L_Joint1_I = 0.002;
    L_joint2_P = 0.001;
    L_Joint2_D = 0.00;
    L_Joint2_I = 0.001;
elseif flag == 2
    % joint_velocity_control_mode Gain Parameter
    % % % % joint control init % % %
    % % Right Joint PD Gain
    R_ANG0_P = 0.02;
    R_ANG1_P = 0.002;
    R_ANG2_P = 0.004;

    R_ANG0_I = 0.000000;
    R_ANG1_I = 0.000000;
    R_ANG2_I = 0.000000;    

    R_ANG0_D = 0.05;
    R_ANG1_D = 0.01;
    R_ANG2_D = 0.001;    
% % % % % % % % % % % % 
    R_VEL0_P = 0.06;
    R_VEL1_P = 0.01;
    R_VEL2_P = 0.0005;

    R_VEL0_I = 0.01;
    R_VEL1_I = 0.1;
    R_VEL2_I = 0.02;

    R_VEL0_D = 0.02;
    R_VEL1_D = 0.001;
    R_VEL2_D = 0.5;
% % % % % % % % % % % % % % % % % % % % % % % % % % % % 
    % % Left Joint PD Gain
    L_ANG0_P = R_ANG0_P;
    L_ANG1_P = R_ANG1_P;
    L_ANG2_P = R_ANG2_P;

    L_ANG0_I = R_ANG0_I;
    L_ANG1_I = R_ANG1_I;
    L_ANG2_I = R_ANG2_I;

    L_ANG0_D = R_ANG0_D;
    L_ANG1_D = R_ANG1_D;
    L_ANG2_D = R_ANG2_D;
% % % % % % % % % % % % % 
    L_VEL0_P = R_VEL0_P;
    L_VEL1_P = R_VEL1_P;
    L_VEL2_P = R_VEL2_P;
    L_VEL0_I = R_VEL0_I;
    L_VEL1_I = R_VEL1_I;
    L_VEL2_I = R_VEL2_I;
    L_VEL0_D = R_VEL0_D;
    L_VEL1_D = R_VEL1_D;
    L_VEL2_D = R_VEL2_D;

else
end
% % % % % % 
% vals = [1, 2, 3];
% [A, B, C] = ndgrid(vals, vals, vals);
% combos = [A(:), B(:), C(:)];

target_init_rate_list = perms([1, 2, 3]*0);
target_init_attitude_list = perms([1, 2, 3]*0);

target_init_attitude = [0 0 0];
target_init_rate = [0 0 0];
target_init_posX = 0.5;
target_init_posY = 0.0;
target_init_posZ = 0.00;
target_init_velX = 0;
target_init_velY = 0;
target_init_velZ = 0;

% sim('main.slx',Simulation_Time);

% target_init_rate_list = perms([1, 2, 3]*0);
% target_init_attitude_list = perms([1, 2, 3]*0);
% 
% target_init_attitude = [0 0 0];
% target_init_rate = [0 0 0];
% target_init_posX = 0.5;
% target_init_posY = 0.0;
% target_init_posZ = 0.00;
% target_init_velX = 0;
% target_init_velY = 0;
% target_init_velZ = 0;
