function slip_flg = slip(R_Jt,RPos_3,touch_flg)

R_qt0 = R_jtc(1);
R_qt1 = R_jtc(2);
R_qt2 = R_jtc(3);


if joint0_curr>=joint_limit(1) || joint1_curr>=joint_limit(2) || joint2_curr>=joint_limit(3) 

       overload_flg = 1;
 else
       overload_flg = 0;
end 


end