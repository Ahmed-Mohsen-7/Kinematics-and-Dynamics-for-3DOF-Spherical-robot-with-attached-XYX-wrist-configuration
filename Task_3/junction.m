
%%Enter joint parameters as [q0,qf,dq_m, ddq_m,]
j1 =[0,10,5,4]; 
j2=[0,20,4,5];


%calculate trajectory time for two joints with t0_2 = tf1
[t0_1,t1_1,T_1,tf_1]=trajectory_time(j1(1),j1(2),j1(3),j1(4),0);
[t0_2,t1_2,T_2,tf_2]=trajectory_time(j2(1),j2(2),j2(3),j2(4),tf_1);
%calculate trajectory time for two joints with t0_2 = T1
[t0_1_jun,t1_1_jun,T_1_jun,tf_1_jun]=trajectory_time(j1(1),j1(2),j1(3),j1(4),0);
[t0_2_jun,t1_2_jun,T_2_jun,tf_2_jun]=trajectory_time(j2(1),j2(2),j2(3),j2(4),T_1);


%calculate trajectory without junction 
[t_1,q_1,v_1,a_1] = plan_trajectory(j1(1),j1(2),j1(3),j1(4),t0_1,t1_1,T_1,tf_1);
[t_2,q_2,v_2,a_2] = plan_trajectory(j2(1),j2(2),j2(3),j2(4),t0_2,t1_2,T_2,tf_2);
%calculate each trajectory with junction
[t_1_jun,q_1_jun,v_1_jun,a_1_jun] = plan_trajectory(j1(1),j1(2),j1(3),j1(4),t0_1_jun,t1_1_jun,T_1_jun,tf_1_jun);
[t_2_jun,q_2_jun,v_2_jun,a_2_jun] = plan_trajectory(j2(1),j2(2),j2(3),j2(4),t0_2_jun,t1_2_jun,T_2_jun,tf_2_jun);



%calculate new trajectory solving the junction issue
t_new=union(t_1_jun,t_2_jun);
v_new(1:1700)=v_1(1:1700);






%plot position, velocity, acceleration trajectory for each joint
figure()
for i=1:2
subplot(2,1,1)
plot(eval(strcat('t_',string(i))),eval(strcat('v_',string(i))))
title('Two trajectories before junction ')
ylabel('v (degree/s)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([0 10])
ylim([0 6])
xline(eval(strcat('t1_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('t1_',string(i))))});
xline(eval(strcat('T_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('T_',string(i))))});
grid on
hold on

subplot(2,1,2)
plot(eval(strcat('t_',string(i),'_jun')),eval(strcat('v_',string(i),'_jun')))
title('Two trajectories with junction ')
ylabel('v (degree/s)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([0 10])
ylim([0 6])
xline(eval(strcat('t1_',string(i),'_jun')),'--',{sprintf('tb= %.2f ',eval(strcat('t1_',string(i),'_jun')))});
xline(eval(strcat('T_',string(i),'_jun')),'--',{sprintf('tb= %.2f ',eval(strcat('T_',string(i),'_jun')))});
grid on
hold on
end










