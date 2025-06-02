close all; clear all; clc;

R_t =1;
cmd =1;
elapsedTime = 15;

AA = casex(R_t, cmd, elapsedTime);


function A = casex(R_t, cmd, elapsedTime)

S1 =0;
S2 =1;


persistent current_state
persistent flag

if isempty(current_state)
    current_state = S1;   
    timer_flg = 0;
    flag = 1;
    current_state = 0;
end

switch (current_state)

    % R_t 가 1, 즉 타이머 플래그가 1이고 타이머가 10초 이상이되면 case S2로 넘어감

    case S1 

    if (R_t)
        if trackTime(elapsedTime, R_t) >=10
            current_state = 1;
            fprintf("state changed 0 to 1")
            trackTime(elapsedTime,0);
        end
    else
        current_state = 0;
        fprintf ("state 0")
    end

    case S2
        if cmd ==1;
            current_state =1;
        end
end

A =current_state
end



function [temp] = trackTime(elapsedTime, timer_flag)

    persistent isTiming startTime totalDuration
    temp =0;

    if isempty(isTiming)
        isTiming =false;
        startTime =0;
        totalDuration =0;
    end

    if timer_flag ==1 && ~isTiming
        startTime =elapsedTime;
        isTiming =true;
        fprintf("Timer start")

    elseif timer_flag ==1 && isTiming
           currentDuration = elapsedTime-startTime;
           temp = currentDuration;
           fprintf("onTimer")

    elseif timer_flag ==0 && isTiming
            totalDuration = elapsedTime - startTime;
            fprintf("Timer stop")
            isTiming = false;
            startTime =0;
            totalDuration =0;
    end
end