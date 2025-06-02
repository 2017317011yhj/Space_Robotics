clear all;
close all;
clc;

sim_info = load('sim_info.mat');
case_num = sim_info.case_num;


for i=1:case_num

    str = sprintf('case_num%d = load(''case_num%d_Result.mat'');',i,i);
    eval(str);

    str = sprintf(['case_num%d_ARM_Contact_Flg = ' ...
                    'case_num%d.simResult.Simscape_ARM_Conatact_Flg;'],i,i);
    eval(str);
end

figure();
for i=1:case_num
    
    str = sprintf('subplot(case_num,1,i);');
    eval(str);
    str = sprintf('plot(case_num%d.simResult.tout, case_num%d_ARM_Contact_Flg)',i,i);
    eval(str);
    eval('grid on; axis equal;');
    str = sprintf('title(''case %d'')',i);
    eval(str);

end

