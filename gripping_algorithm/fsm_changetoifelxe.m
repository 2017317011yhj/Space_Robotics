clear all; close all; clc;

R_t = [0; 0; 0];
R_q = [0; 0; 0];
grip_cmd = [1 1 1];
% elapsedTime = 14;
% 
% 
% current_state = mlhdlc_fsm_mealy(R_t, R_q, grip_cmd, elapsedTime);

for t = 0:1:30
    elapsedTime = t;
    state = mlhdlc_fsm_mealy(R_t, R_q, grip_cmd, elapsedTime);
    pause(0.1); 
end


function Z = mlhdlc_fsm_mealy(R_t, ...
    R_q, grip_cmd, elapsedTime)

% define states
S1 = 1; %% pre-grasping
S2 = 2; %% stay
S3 = 3; %% closing (grasping)
S4 = 4; %% touch
S5 = 5; %% empty
Body_contact=1;

persistent current_state
persistent flag

if isempty(current_state)
    current_state = S1;   
    timer_flg = 0;
    flag = 1;
    current_state = 1;
end

if current_state == S1;
     fprintf ("state S1\n")
        
    R_condition_1 = (R_q(1) >= -45-2 && R_q(1) <= -45+2);
    R_condition_2 = (R_q(2) >= -5 && R_q(2) <= 5);
    R_condition_3 = (R_q(3) >= -5 && R_q(3) <= 5);
    
    R_condition = R_condition_1 && R_condition_2 && R_condition_3;

    % condition = R_condition;
    condition = 1;

    Rv_cmd = [-45 0 0]'*pi/180;
    control_mode = [1 1 1]';
    
    if (condition)
        if trackTime(elapsedTime, condition,1) >= 10
            current_state = 2;
            fprintf ("S1 to S2 change complete\n")
            trackTime(elapsedTime, 0,1);
        end
    else
            current_state = 1;
            fprintf ("S1 stay")

    end
   
end

if current_state == S2
     fprintf ("state S2\n")

     if trackTime(elapsedTime, 1, 2) >= 10
        current_state = 3;
        fprintf("S2 going\n")
        durationTime_S2 = elapsedTime
        trackTime(elapsedTime, 0, 2);
     else
        current_state = 2;
        Rv_cmd = [0 0 0]'*pi/180;
        control_mode = [2 2 2]';
        fprintf("S2 stay\n")
     end
end

if current_state == S3;
     fprintf ("state S3\n")
       Rv_cmd = [0.5 2 1.5]'*1*pi/180;
       control_mode = [2 2 2]';

    if Body_contact == 1
       grasped_time = trackTime(elapsedTime, 1,3);

        if grasped_time>=1
            current_state = 2;
            trackTime(elapsedTime, 0,3);
        end
    else
        trackTime(elapsedTime, 0,3);
    end
end

 Z = current_state

end



function [temp] = trackTime(elapsedTime, timer_flg, id)
    persistent isTiming startTime totalDuration
    temp = 0;

    if isempty(isTiming)
        isTiming = false(10,1);  % 최대 10개의 타이머 ID 지원
        startTime = zeros(10,1);
        totalDuration = zeros(10,1);
    end

    fprintf('[DEBUG] [T%d] flag = %d, isTiming = %d, startTime = %.2f, elapsedTime = %.2f\n', ...
        id, timer_flg, isTiming(id), startTime(id), elapsedTime);

    if timer_flg == 1 && ~isTiming(id)
        startTime(id) = elapsedTime;
        isTiming(id) = true;
        fprintf('[DEBUG] [T%d] Timer started at %.2f초\n', id, startTime(id));

    elseif timer_flg == 1 && isTiming(id)
        currentDuration = elapsedTime - startTime(id);
        temp = currentDuration;
        fprintf('[DEBUG] [T%d] Timer running: %.2f초 경과\n', id, currentDuration);

    elseif timer_flg == 0 && isTiming(id)
        totalDuration(id) = elapsedTime - startTime(id);
        fprintf('[DEBUG] [T%d] Timer stopped at %.2f초 (총 경과 %.2f초)\n', ...
            id, elapsedTime, totalDuration(id));
        isTiming(id) = false;
        startTime(id) = 0;
        totalDuration(id) = 0;
    end
end

