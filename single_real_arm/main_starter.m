close all;
clc;

% % % % Service Body
S_Body_Mass = 5;

S_init_pos = [0 0 0]';
S_init_vel = [0 0 0]';

S_init_att = [0 0 0]';%[deg]
S_init_rate = [0 0 0]';%[deg/sec]
% % % % Service ARM

L_Joint_Offset = [
    -0.464048464137005
     0.464048464137005
    -0.234639156053821]';%[rad]

Joint_Motor_Mass = 23;%[g]
Link1_Mass = 11;%[g]
Link2_Mass = 15.6;%[g]
Link3_Mass = 11;%[g]

% [0 -90 -180] : Stowing Mode
% [45 0 0] : Pre-Grasp Mode
L_Joint0_angle0 = 45;%[deg]
L_Joint0_angle1 = 0;%[deg] 
L_Joint0_angle2 = 0;%[deg]
L_Joint0_rate0 = 0;%[deg/sec]
L_Joint0_rate1 = 0;%[deg/sec]
L_Joint0_rate2 = 0;%[deg/sec]

R_Joint0_angle0 = 45;%[deg]
R_Joint0_angle1 = 0;%[deg] 
R_Joint0_angle2 = 0;%[deg]
R_Joint0_rate0 = 0;%[deg/sec]
R_Joint0_rate1 = 0;%[deg/sec]
R_Joint0_rate2 = 0;%[deg/sec]

U_Joint0_angle0 = 45;%[deg]
U_Joint0_angle1 = 0;%[deg] 
U_Joint0_angle2 = 0;%[deg]
U_Joint0_rate0 = 0;%[deg/sec]
U_Joint0_rate1 = 0;%[deg/sec]
U_Joint0_rate2 = 0;%[deg/sec]

D_Joint0_angle0 = 45;%[deg]
D_Joint0_angle1 = 0;%[deg] 
D_Joint0_angle2 = 0;%[deg]
D_Joint0_rate0 = 0;%[deg/sec]
D_Joint0_rate1 = 0;%[deg/sec]
D_Joint0_rate2 = 0;%[deg/sec]

Motor_transfer_Num = [100];
Motor_transfer_Den = [1 100];

Torq_mea_transfer_Num = [5];
Torq_mea_transfer_Den = [1 5];

PID_dt = 0.01;%[sec]
ARM_T_PID = [100 0 0];
ARM_V_PID = [10 0 5]*0.1;
ARM_P_PID = [10 0 20]*0.1;

% % % Contact Parameter
% Body to Target
B2T_Siffness = 1000;%N/m
B2T_Damping = 500;%N/(m/s)
B2T_TRW = 0.001;%m
B2T_SF = 0.8;
B2T_DF = 0.6;
B2T_CV = 1e-3;%[m/s]

% LINK1 to TARGET
L12T_Siffness = 1000;%N/m
L12T_Damping = 500;%N/(m/s)
L12T_TRW = 0.01;%m
L2T_SF = 0.8;
L2T_DF = 0.6;
L2T_CV = 1e-3;%[m/s]
% LINK2 to TARGET
L22T_Siffness = 1000;%N/m
L22T_Damping = 500;%N/(m/s)
L22T_TRW = 0.001;%m
% LINK3 to TARGET
L32T_Siffness = 1000;%N/m
L32T_Damping = 10;%N/(m/s)
L32T_TRW = 0.4;%m

% LINK1 to Body
L12B_Stiffness = 1000;
L12B_Damping = 100;
L12B_TRW = 0.001;
L2B_SF = 0.8;
L2B_DF = 0.6;
L2B_CV = 1e-3;
% LINK2 to Body
L22B_Stiffness = 1000;
L22B_Damping = 1;
L22B_TRW = 0.001;
% LINK3 to Body
L32B_Stiffness = 1000;
L32B_Damping = 1;
L32B_TRW = 0.001;

% % % % % % RW & & & & & & & & & & & & &
RW_Mass = 100;%[g]
RW_Init_Speed = 1000;%[RPM]

RW_Att_P = 0.1;
RW_Att_I = 0.2;
RW_Att_D = 0.5;

% % % % % Body Contact Sensor
sensor2target_stiffness = 1000;
sensor2target_damping = 100;
sensor2target_TRW = 0.001;
sensor2target_static_friction = 1.0;
sensor2target_dynamic_friction = 0.9;
sensor2target_velocity=1e-4;

% % % % Target Body
T_Body_Mass = 4;

T_init_pos = [1 0 0]';
T_init_vel = [0 0 0]';

T_init_att = [0 0 0]';%[deg] ZYX
T_init_rate = [0 0 0]';%[deg/sec]

