function dc= P2P_s_plannr(Xd,Xc,Td,t)
%% judge input if legal

%% user define:
a_max = 5;%m/s^2;
v_max = 6;% m/s;

%% planner:
ld = Xd-Xc;
l_max = v_max^2/a_max;

delta = (a_max*Td)^2-4*a_max*ld;
if delta>=0
    t_acc = (a_max*Td-delta^0.5)/(2*a_max); 
    if a_max*t_acc > v_max
       t_acc = v_max/a_max;
    end
elseif l_max>ld 
    t_acc = (ld/a_max)^0.5;
else
    t_acc = v_max/a_max;
end
v_mv =t_acc*a_max;
t_dacc = (ld - a_max*t_acc*t_acc)/v_mv+2*t_acc;
t_con = t_dacc - t_acc;
a_mv = a_max;

if Xd < Xc
    v_mv = -v_mv;
    a_mv = -a_max;
end

%% update planner:
if t<=t_acc
   dc = 0.5*a_mv*t*t;
elseif t<=t_con
   dc=0.5*a_mv*t_acc*t_acc+(t-t_acc)*v_mv;
elseif t<=t_dacc
    dc = a_mv*t_acc*t_acc+(t_con-t_acc)*v_mv-0.5*a_mv*(t_dacc-t)*(t_dacc-t);
else
    dc = a_mv*t_acc*t_acc+(t_con-t_acc)*v_mv;
end

