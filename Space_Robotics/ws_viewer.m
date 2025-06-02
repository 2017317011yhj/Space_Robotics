close all;
clc;

EE_Pos = out.EE_Pos;

EE_Pos_X = EE_Pos(:,1);
EE_Pos_Y = EE_Pos(:,2);

figure;
plot(EE_Pos_X, EE_Pos_Y,'r.');
axis equal;
grid on;
