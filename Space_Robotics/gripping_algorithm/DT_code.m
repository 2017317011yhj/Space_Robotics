clear all;
close all;
clc;

% 예시 입력
empty_flag1 = 1;  touch_flag1 = 0;
empty_flag2 = 1;  touch_flag2 = 0;
empty_flag3 = 1;  touch_flag3 = 1;
empty_flag4 = 1;  touch_flag4 = 1;

a = state_DT(empty_flag1, touch_flag1, empty_flag2, touch_flag2, ...
             empty_flag3, touch_flag3, empty_flag4, touch_flag4);


function system_state = state_DT(empty_flag1, touch_flag1, ...
                                 empty_flag2, touch_flag2, ...
                                 empty_flag3, touch_flag3, ...
                                 empty_flag4, touch_flag4)
%% State def
S1 =1; % Empty
S2 =2; % Overload
S3 =3; % Slip
S4 =4; % Grasped

empty_flags = [empty_flag1, empty_flag2, empty_flag3, empty_flag4];
touch_flags = [touch_flag1, touch_flag2, touch_flag3, touch_flag4];

%% Empty
if empty_flag1 && empty_flag2 && empty_flag3 && empty_flag4
    system_state = 1;
else
    system_state = 0;
end

%% Overload


%% Slip
% %%DT
% if empty_system_flg == 1
%     system_state = 1;  % Empty 상태
% elseif touch_system_flg == 1
%     system_state = 2;  % Touch 상태
% else
%     system_state = 0;  % 기본 상태
% end
system_state = system_state
end

;