function [speed_drop_flg, touch_flg, q_touch_flg, speed_drop_vec] = detectTouch3(touch_start, R_jtc, R_Jd, Rv_cmd)

    % --- 파라미터 설정 ---

    speed_err_thresh      = 1e-2;
    current_min_thresh    = 0.2e-3;
    current_max_thresh    = 2.5e-3;

    % --- 출력 초기화 ---
    speed_drop_flg  = 0;
    touch_flg       = 0;
    q_touch_flg     = 0;
    speed_drop_vec  = zeros(3,1);  % double 타입 고정

    % --- 전류 추출 ---
    R_qt0 = R_jtc(1);
    R_qt1 = R_jtc(2);
    R_qt2 = R_jtc(3);

    q0_touch_flg = 0;
    q1_touch_flg = 0;
    q2_touch_flg = 0;

    % --- 속도 차이 계산 ---
    raw_vec = abs(Rv_cmd - R_Jd) > speed_err_thresh;
    speed_drop_vec = double(raw_vec);  % 형 변환

    if any(raw_vec)
        speed_drop_flg = 1;
    end

    if touch_start == 1 
        if abs(R_qt0) >= current_min_thresh && abs(R_qt0) < current_max_thresh
            q0_touch_flg = 1;
        end
        if abs(R_qt1) >= current_min_thresh && abs(R_qt1) < current_max_thresh
            q1_touch_flg = 1;
        end
        if abs(R_qt2) >= current_min_thresh && abs(R_qt2) < current_max_thresh
            q2_touch_flg = 1;
        end

        if q0_touch_flg || q1_touch_flg || q2_touch_flg
           touch_flg = 1;
        end

        % if  q_touch_flg
        %     touch_flg = 1;
        % end
    end
end
