%% This code contaitns task 1,2,3,4. just press run and all results will be shown
%%%Task 6 is in a seperate file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task 3.1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%This portion caluclates trajectory for robot model with q,v,a plots

%%%make synchornized joint motion
t0=0;

%%Enter joint parameters as [q0,qf,dq_m, ddq_m,]
j1 =[0,10,5,4]; 
j2=[0,50,8,4];
j3 =[0,5,2,1]; %prismatic joint 
j4=[0,40,4,2];
j5 =[0,50,8,4]; 
j6=[0,70,5,2];

%calculate trajectory time for each
[t0_1,t1_1,T_1,tf_1]=trajectory_time(j1(1),j1(2),j1(3),j1(4),0);
[t0_2,t1_2,T_2,tf_2]=trajectory_time(j2(1),j2(2),j2(3),j2(4),0);
[t0_3,t1_3,T_3,tf_3]=trajectory_time(j3(1),j3(2),j3(3),j3(4),0);
[t0_4,t1_4,T_4,tf_4]=trajectory_time(j4(1),j4(2),j4(3),j4(4),0);
[t0_5,t1_5,T_5,tf_5]=trajectory_time(j5(1),j5(2),j5(3),j5(4),0);
[t0_6,t1_6,T_6,tf_6]=trajectory_time(j6(1),j6(2),j6(3),j6(4),0);


%calculate each trajectory
[t_1,q_1,v_1,a_1] = plan_trajectory(j1(1),j1(2),j1(3),j1(4),t0_1,t1_1,T_1,tf_1);
[t_2,q_2,v_2,a_2] = plan_trajectory(j2(1),j2(2),j2(3),j2(4),t0_2,t1_2,T_2,tf_2);
[t_3,q_3,v_3,a_3] = plan_trajectory(j3(1),j3(2),j3(3),j3(4),t0_3,t1_3,T_3,tf_3);
[t_4,q_4,v_4,a_4] = plan_trajectory(j4(1),j4(2),j4(3),j4(4),t0_4,t1_4,T_4,tf_4);
[t_5,q_5,v_5,a_5] = plan_trajectory(j5(1),j5(2),j5(3),j5(4),t0_5,t1_5,T_5,tf_5);
[t_6,q_6,v_6,a_6] = plan_trajectory(j6(1),j6(2),j6(3),j6(4),t0_6,t1_6,T_6,tf_6);


%plot position, velocity, acceleration trajectory for each joint
figure()
n=1;
for i=1:6
    
sgtitle(' Joint Trajectories') 
subplot(6,3,n);
plot(eval(strcat('t_',string(i))),eval(strcat('q_',string(i))))
title(sprintf('q(t) for joint %d',i))
ylabel('q (degree)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_',string(i))))  max(eval(strcat('t_',string(i))))])
ylim([min(eval(strcat('q_',string(i))))-2 max(eval(strcat('q_',string(i)))+2)])
xline(eval(strcat('t1_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('t1_',string(i))))});
xline(eval(strcat('T_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('T_',string(i))))});
grid on


subplot(6,3,n+1);
plot(eval(strcat('t_',string(i))),eval(strcat('v_',string(i))))
title(sprintf('v(t) for joint %d',i))
ylabel('v (degree/s)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_',string(i))))  max(eval(strcat('t_',string(i))))])
ylim([min(eval(strcat('v_',string(i))))-2 max(eval(strcat('v_',string(i)))+2)])
xline(eval(strcat('t1_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('t1_',string(i))))});
xline(eval(strcat('T_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('T_',string(i))))});
grid on


subplot(6,3,n+2);
plot(eval(strcat('t_',string(i))),eval(strcat('a_',string(i))))
title(sprintf('a(t) for joint %d',i))
ylabel('a (degree/s^2)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_',string(i))))  max(eval(strcat('t_',string(i))))])
ylim([min(eval(strcat('a_',string(i))))-2 max(eval(strcat('a_',string(i)))+2)])
xline(eval(strcat('t1_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('t1_',string(i))))});
xline(eval(strcat('T_',string(i))),'--',{sprintf('tb= %.2f ',eval(strcat('T_',string(i))))});
grid on
n=n+3;
end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task 3.2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%synchronization
%calculate new time parameters
rise_time = max([t1_1,t1_2,t1_3,t1_4,t1_5,t1_6]);
fprintf("\nJ1 rise time: %.2f, J2 rise time: %.2f ,J3 rise time: %.2f,\n J4 rise time: %.2f, J5 rise time: %.2f, J6 rise time: %.2f, synchronized rise time: %.2f",t1_1,t1_2,t1_3,t1_4,t1_5,t1_6,rise_time)
dwell = max([T_1-t1_1, T_2-t1_2,T_3-t1_3, T_4-t1_4,T_5-t1_5, T_6-t1_6]);
fprintf(" \n\n J1 dwell time: %.2f, J2 dwell time: %.2f ,J3 dwell time: %.2f,\n J4 dwell time: %.2f, J5 dwell time: %.2f, J6 dwell time: %.2f, synchronized dwell time: %.2f\n",T_1-t1_1, T_2-t1_2,T_3-t1_3, T_4-t1_4,T_5-t1_5, T_6-t1_6,dwell)
 T = dwell+rise_time;
 tf = T + rise_time;
fprintf("Synchronized trajectory time = %.2f + %.2f + %.2f = %.2fs",rise_time,dwell,rise_time,tf)




%recalculate velocity parameters
dq1_m = (j1(2)-j1(1))/T;
dq2_m = (j2(2)-j2(1))/T;
dq3_m = (j3(2)-j3(1))/T;
dq4_m = (j4(2)-j4(1))/T;
dq5_m = (j5(2)-j5(1))/T;
dq6_m = (j6(2)-j6(1))/T;


%recalculate acceleration parameters
ddq1_m = dq1_m/rise_time;
ddq2_m = dq2_m/rise_time;
ddq3_m = dq3_m/rise_time;
ddq4_m = dq4_m/rise_time;
ddq5_m = dq5_m/rise_time;
ddq6_m = dq6_m/rise_time;


fprintf("\n\nJoint 1 velocity modified from %.2f to %.2f and acceleration from %.2f to %.2f",j1(3), dq1_m, j1(4), ddq1_m)
fprintf("\nJoint 2 velocity modified from %.2f to %.2f and acceleration from %.2f to %.2f",j2(3), dq2_m, j2(4), ddq2_m)
fprintf("\nJoint 3 velocity modified from %.2f to %.2f and acceleration from %.2f to %.2f",j3(3), dq3_m, j3(4), ddq3_m)
fprintf("\nJoint 4 velocity modified from %.2f to %.2f and acceleration from %.2f to %.2f",j4(3), dq4_m, j4(4), ddq4_m)
fprintf("\nJoint 5 velocity modified from %.2f to %.2f and acceleration from %.2f to %.2f",j5(3), dq5_m, j5(4), ddq5_m)
fprintf("\nJoint 6 velocity modified from %.2f to %.2f and acceleration from %.2f to %.2f\n",j6(3), dq6_m, j6(4), ddq6_m)

%update parameters 
j1(3)=dq1_m;
j1(4)=ddq1_m;
j2(3)=dq2_m;
j2(4)=ddq2_m;
j3(3)=dq3_m;
j3(4)=ddq3_m;
j4(3)=dq4_m;
j4(4)=ddq4_m;
j5(3)=dq5_m;
j5(4)=ddq5_m;
j6(3)=dq6_m;
j6(4)=ddq6_m;


%plan new trajectory 
[t_1,q_1,v_1,a_1] = plan_trajectory(j1(1),j1(2),j1(3),j1(4),t0,rise_time,T,tf);
[t_2,q_2,v_2,a_2] = plan_trajectory(j2(1),j2(2),j2(3),j2(4),t0,rise_time,T,tf);
[t_3,q_3,v_3,a_3] = plan_trajectory(j3(1),j3(2),j3(3),j3(4),t0,rise_time,T,tf);
[t_4,q_4,v_4,a_4] = plan_trajectory(j4(1),j4(2),j4(3),j4(4),t0,rise_time,T,tf);
[t_5,q_5,v_5,a_5] = plan_trajectory(j5(1),j5(2),j5(3),j5(4),t0,rise_time,T,tf);
[t_6,q_6,v_6,a_6] = plan_trajectory(j6(1),j6(2),j6(3),j6(4),t0,rise_time,T,tf);



%%plot the new result after synchornization
n=1;
figure()
for i=1:6
sgtitle('synchronized Joint Trajectories') 
%plot q graph
subplot(6,3,n);
plot(eval(strcat('t_',string(i))),eval(strcat('q_',string(i))))
title(sprintf('q(t) for joint %d',i))
ylabel('q (degree)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_',string(i))))  max(eval(strcat('t_',string(i))))])
ylim([min(eval(strcat('q_',string(i))))-2 max(eval(strcat('q_',string(i)))+2)])
xline(rise_time,'--',{sprintf('tb= %.2f ',rise_time)});
xline(T,'--',{sprintf('tb= %.2f ',T)});
grid on

%plot v graph
subplot(6,3,n+1);
plot(eval(strcat('t_',string(i))),eval(strcat('v_',string(i))))
title(sprintf('v(t) for joint %d',i))
ylabel('v (degree/s)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_',string(i))))  max(eval(strcat('t_',string(i))))])
ylim([min(eval(strcat('v_',string(i))))-2 max(eval(strcat('v_',string(i)))+2)])
xline(rise_time,'--',{sprintf('tb= %.2f ',rise_time)});
xline(T,'--',{sprintf('tb= %.2f ',T)});
grid on


%plot a graph
subplot(6,3,n+2);
plot(eval(strcat('t_',string(i))),eval(strcat('a_',string(i))))
title(sprintf('a(t) for joint %d',i))
ylabel('a (degree/s^2)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_',string(i))))  max(eval(strcat('t_',string(i))))])
ylim([min(eval(strcat('a_',string(i))))-2 max(eval(strcat('a_',string(i)))+2)])
xline(rise_time,'--',{sprintf('tb= %.2f ',rise_time)});
xline(T,'--',{sprintf('tb= %.2f ',T)});
grid on
n=n+3;
end








%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task 3.3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%assume we have a controller with f frequency

f = 1;
delta_t=1/f;


%Enter joint parameters as [q0,qf,dq_m, ddq_m,]
j1 =[0,10,5,4]; 
j2=[0,50,8,4];
j3 =[0,5,2,1]; %prismatic joint 
j4=[0,40,4,2];
j5 =[0,50,8,4]; 
j6=[0,70,5,2];

t0=0;


%calculate new time parameters
rise_time_n = (floor(rise_time/delta_t) + 1)*delta_t;
fprintf("\n\nrise time:  %.2f numerical rise time: %.2f",rise_time,rise_time_n)
dwell_n = (floor(dwell/delta_t) + 1)*delta_t;
fprintf("\ndwell time:  %.2f numerical dwell time: %.2f\n",dwell,dwell_n) 
T_n = dwell_n+rise_time_n;
tf_n = T_n + rise_time_n;


%%recalculate velocity parameters
dq1_m = (j1(2)-j1(1))/T_n;
dq2_m = (j2(2)-j2(1))/T_n;
dq3_m = (j3(2)-j3(1))/T_n;
dq4_m = (j4(2)-j4(1))/T_n;
dq5_m = (j5(2)-j5(1))/T_n;
dq6_m = (j6(2)-j6(1))/T_n;


%%recalculate acceleration parameters
ddq1_m = dq1_m/rise_time_n;
ddq2_m = dq2_m/rise_time_n;
ddq3_m = dq3_m/rise_time_n;
ddq4_m = dq4_m/rise_time_n;
ddq5_m = dq5_m/rise_time_n;
ddq6_m = dq6_m/rise_time_n;


%%update parameters 
j1(3)=dq1_m;
j1(4)=ddq1_m;
j2(3)=dq2_m;
j2(4)=ddq2_m;
j3(3)=dq3_m;
j3(4)=ddq3_m;
j4(3)=dq4_m;
j4(4)=ddq4_m;
j5(3)=dq5_m;
j5(4)=ddq5_m;
j6(3)=dq6_m;
j6(4)=ddq6_m;


%%plan new trajectory 
[t_sn1,q_sn1,v_sn1,a_sn1] = plan_trajectory(j1(1),j1(2),j1(3),j1(4),t0,rise_time_n,T_n,tf_n);
[t_sn2,q_sn2,v_sn2,a_sn2] = plan_trajectory(j2(1),j2(2),j2(3),j2(4),t0,rise_time_n,T_n,tf_n);
[t_sn3,q_sn3,v_sn3,a_sn3] = plan_trajectory(j3(1),j3(2),j3(3),j3(4),t0,rise_time_n,T_n,tf_n);
[t_sn4,q_sn4,v_sn4,a_sn4] = plan_trajectory(j4(1),j4(2),j4(3),j4(4),t0,rise_time_n,T_n,tf_n);
[t_sn5,q_sn5,v_sn5,a_sn5] = plan_trajectory(j5(1),j5(2),j5(3),j5(4),t0,rise_time_n,T_n,tf_n);
[t_sn6,q_sn6,v_sn6,a_sn6] = plan_trajectory(j6(1),j6(2),j6(3),j6(4),t0,rise_time_n,T_n,tf_n);



%%%plot the new result after synchornization
n=1;
figure()
for i=1:6

%plot q graph
sgtitle('Synchornized Joint Trajectories with numerical control')
subplot(6,3,n);
plot(eval(strcat('t_sn',string(i))),eval(strcat('q_sn',string(i))))
title(sprintf('q(t) for joint %d',i))
ylabel('q (degree)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_sn',string(i))))  max(eval(strcat('t_sn',string(i))))])
ylim([min(eval(strcat('q_sn',string(i))))-2 max(eval(strcat('q_sn',string(i)))+2)])
xline(rise_time_n,'--',{sprintf('tb= %.2f ',rise_time_n)});
xline(T_n,'--',{sprintf('T= %.2f ',T_n)});
grid on

%plot v graph
subplot(6,3,n+1);
plot(eval(strcat('t_sn',string(i))),eval(strcat('v_sn',string(i))))
title(sprintf('v(t) for joint %d',i))
ylabel('v (degree/s)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_sn',string(i))))  max(eval(strcat('t_sn',string(i))))])
ylim([min(eval(strcat('v_sn',string(i))))-2 max(eval(strcat('v_sn',string(i)))+2)])
xline(rise_time_n,'--',{sprintf('tb= %.2f ',rise_time_n)});
xline(T_n,'--',{sprintf('T= %.2f ',T_n)});
grid on


%plot a graph
subplot(6,3,n+2);
plot(eval(strcat('t_sn',string(i))),eval(strcat('a_sn',string(i))))
title(sprintf('a(t) for joint %d',i))
ylabel('a (degree/s^2)','FontSize',12,'FontWeight','bold')
xlabel('t (s)','FontSize',12,'FontWeight','bold')
xlim([min(eval(strcat('t_sn',string(i))))  max(eval(strcat('t_sn',string(i))))])
ylim([min(eval(strcat('a_sn',string(i))))-2 max(eval(strcat('a_sn',string(i)))+2)])
xline(rise_time_n,'--',{sprintf('tb= %.2f ',rise_time_n)});
xline(T_n,'--',{sprintf('T= %.2f ',T_n)});
grid on
n=n+3;
end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task 3.4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculate FK before and after numerical control
Actual_FK= FK( q_1(length(q_1)) , q_2(length(q_2)),q_3(length(q_3)),q_4(length(q_4)),q_5(length(q_5)),q_6(length(q_6)));
Numerical_FK=FK(q_sn1(length(q_sn1)),q_sn2(length(q_sn2)),q_sn3(length(q_sn3)),q_sn4(length(q_sn4)),q_sn5(length(q_sn5)),q_sn6(length(q_sn6)));
  
%Calculate the difference between x,y,z postion 
x_error=Actual_FK(1:4)-Numerical_FK(1:4);
y_error=Actual_FK(2:4)-Numerical_FK(2:4);
z_error=Actual_FK(3:4)-Numerical_FK(3:4);

fprintf("\nError for end effector position is : %.2f in x and %.2f in y and %.2f in z\n",x_error,y_error,z_error)


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Task 3.5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



