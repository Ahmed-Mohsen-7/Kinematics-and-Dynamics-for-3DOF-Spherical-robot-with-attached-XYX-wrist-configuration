%This code can solve robot dynmaics symbolaically and thus saving much time in
%manual solvign for the problem 
clear 
clc

syms theta_1
rot_z1 =  [cos(theta_1)   -sin(theta_1)         0     0;
          sin(theta_1)    cos(theta_1)         0     0;
              0                0            1.0000   0
              0                0              0      1];

rot_x1 =  [1       0               0         0;
          0   cos(theta_1)   -sin(theta_1)  0;
          0   sin(theta_1)    cos(theta_1)  0;
          0       0               0         1];

syms theta_2
rot_z2 =  [cos(theta_2)   -sin(theta_2)         0     0;
          sin(theta_2)    cos(theta_2)         0     0;
              0                0            1.0000   0
              0                0              0      1];
 rot_y2 =  [cos(theta_2)    0   sin(theta_2)  0;
               0          1       0         0;
          -sin(theta_2)   0   cos(theta_2)  0
              0           0       0         1];
   rot_x2 =  [1       0               0         0;
          0   cos(theta_2)   -sin(theta_2)  0;
          0   sin(theta_2)    cos(theta_2)  0;
          0       0               0         1];


  syms theta_3
rot_z3 =  [cos(theta_3)   -sin(theta_3)         0     0;
          sin(theta_3)    cos(theta_3)         0     0;
              0                0            1.0000   0
              0                0              0      1]; 
rot_x3 =  [1       0               0         0;
          0   cos(theta_3)   -sin(theta_3)  0;
          0   sin(theta_3)    cos(theta_3)  0;
          0       0               0         1];
  syms theta_4
rot_z4 =  [cos(theta_4)   -sin(theta_4)         0     0;
          sin(theta_4)    cos(theta_4)         0     0;
              0                0            1.0000   0
              0                0              0      1];
          
 rot_x4 =  [1       0               0         0;
            0   cos(theta_4)   -sin(theta_4)  0;
            0   sin(theta_4)    cos(theta_4)  0;
            0       0               0         1];
        
        
 syms theta_5
rot_z5 =  [cos(theta_5)   -sin(theta_5)         0     0;
          sin(theta_5)    cos(theta_5)         0     0;
              0                0            1.0000   0
              0                0              0      1];
        
rot_y5 =  [cos(theta_5)    0   sin(theta_5)  0;
               0          1       0         0;
          -sin(theta_5)   0   cos(theta_5)  0
              0           0       0         1];

syms theta_6
rot_z6 =  [cos(theta_6)   -sin(theta_6)         0     0;
          sin(theta_6)    cos(theta_6)         0     0;
              0                0            1.0000   0
              0                0              0      1];

rot_x6 =  [1       0               0         0;
          0   cos(theta_6)   -sin(theta_6)  0;
          0   sin(theta_6)    cos(theta_6)  0;
          0       0               0         1];

syms d_1
trans_d1 =  [1  0 0 d_1;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
trans_y1 =  [1  0 0 0;
            0  1 0 d_1;
            0  0 1 0;
            0  0 0 1];
trans_z1 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 d_1;
            0  0 0 1];
   
syms d_2
trans_x2 =  [1  0 0 d_2;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
trans_z2 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 d_2;
            0  0 0 1];
        
syms l2 q3
trans_x3 =  [1  0 0 l2+q3;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
trans_z3 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 l2+q3;
            0  0 0 1];

syms d_4
trans_x4 =  [1  0 0 d_4;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
        
trans_z4 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 d_4;
            0  0 0 1];
trans_y4 =  [1  0 0 0;
            0  1 0 d_4;
            0  0 1 0;
            0  0 0 1];
syms d_5          
trans_z5 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 d_5;
            0  0 0 1];
        trans_x5 =  [1  0 0 d_5;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];

syms d_6          
trans_x6 =  [1  0 0 -d_6;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
 
        
        
syms c1
trans_c1=  [1  0 0  0;
            0  1 0  0;
            0  0 1  c1;
            0  0 0  1];
        
 syms c2
trans_c2=  [1  0 0  c2;
            0  1 0  0;
            0  0 1  0;
            0  0 0  1];
        
 syms q3 l2
trans_c3=  [1  0 0  0;
            0  1 0  0;
            0  0 1  q3+l2;
            0  0 0  1];
  syms c4
trans_c4=  [1  0 0  0;
            0  1 0  c4;
            0  0 1  0;
            0  0 0  1];
 syms c5
trans_c5=  [1  0 0  c5;
            0  1 0  0;
            0  0 1  0;
            0  0 0  1];
        
 syms c6
trans_c6=  [1  0 0  c6;
            0  1 0  0;
            0  0 1  0;
            0  0 0  1];

     
%These six matrices represnt the transformation between each two consecutive joints of the model        
a1 = rot_z1*trans_z1*rotm2tform(rotx(90));
a2 = rot_z2*rotm2tform(rotz(90))*rotm2tform(rotx(90));
a3 = rotm2tform(rotz(180))*trans_z3*rotm2tform(rotx(90));
a4 = rot_z4*rotm2tform(rotz(90))*trans_x4*rotm2tform(rotx(-90));
a5 = rot_z5*trans_x5*rotm2tform(rotx(90));
a6 = rot_z6*trans_x6;


%This represents tranformation needed for each COM. first CoM is easy so it
%written directly in the next step
COM_02 = a1*rot_z2*trans_c2;
COM_03 = a1*a2*trans_c3;
COM_04 = a1*a2*a3*rot_z4*trans_c4;
COM_05 = a1*a2*a3*a4*rot_z5*trans_c5;
COM_06 = a1*a2*a3*a4*a5*rot_z6*trans_c6;


%Center of mass as vector [x, y, z] for each frame with relative to the generalized coordinate system
CoM1 = [0; 0 ;c1];
CoM2 = COM_02  (1:3,4);
CoM3 = COM_03(1:3,4);
CoM4 = COM_04(1:3,4);
CoM5 = COM_05(1:3,4);
CoM6 = COM_06(1:3,4);


%Calculate linear jacobian for each CoM

jac_lin_1=[diff(CoM1,theta_1),diff(CoM1,theta_2),diff(CoM1,q3),diff(CoM1,theta_4),diff(CoM1,theta_5),diff(CoM1,theta_6)];
jac_lin_2=[diff(CoM2,theta_1),diff(CoM2,theta_2),diff(CoM2,q3),diff(CoM2,theta_4),diff(CoM2,theta_5),diff(CoM2,theta_6)];
jac_lin_3=[diff(CoM3,theta_1),diff(CoM3,theta_2),diff(CoM3,q3),diff(CoM3,theta_4),diff(CoM3,theta_5),diff(CoM3,theta_6)];
jac_lin_4=[diff(CoM4,theta_1),diff(CoM4,theta_2),diff(CoM4,q3),diff(CoM4,theta_4),diff(CoM4,theta_5),diff(CoM4,theta_6)];
jac_lin_5=[diff(CoM5,theta_1),diff(CoM5,theta_2),diff(CoM5,q3),diff(CoM5,theta_4),diff(CoM5,theta_5),diff(CoM5,theta_6)];
jac_lin_6=[diff(CoM6,theta_1),diff(CoM6,theta_2),diff(CoM6,q3),diff(CoM6,theta_4),diff(CoM6,theta_5),diff(CoM6,theta_6)];


% %Calculate angular jacobian for each CoM
% jac_ang_1=[0;0;1];
% jac_ang_2=COM_02(1:3,3);
% jac_ang_3=COM_03(1:3,3);
% jac_ang_4=COM_04(1:3,3);
% jac_ang_5=COM_05(1:3,3);
% jac_ang_6=COM_06(1:3,3);


jw=zeros(3,6);
for i=1:6
    
if i==3      %jacobian for prismatic joint
        jw(1:3,i)=[0 ;0 ;0];
else          %jacobian for revolute joint
        jw(1:3,i)=[0 ;0 ;1];
end
eval(strcat('jac_ang_',string(i),'=jw;'));

end


syms ixx ixy ixz iyx iyy iyz izx izy izz dq1 dq2 dq3 dq4 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6

I = [ixx ixy ixz ; iyx iyy iyz; izx izy izz];
dq=[dq1 ;dq2 ;dq3 ;dq4 ;dq5 ;dq6];
ddq=[ddq1; ddq2 ;ddq3 ;ddq4 ;ddq5 ;ddq6];
%%%%%%%%%%%%%%%%%%%%%%%%%%calculate M(q)%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms m1 m2 m3 m4 m5 m6  
M_q_lin = m1*transpose(jac_lin_1)*jac_lin_1+m2*transpose(jac_lin_2)*jac_lin_2+m3*transpose(jac_lin_3)*jac_lin_3+m4*transpose(jac_lin_4)*jac_lin_4+m5*transpose(jac_lin_5)*jac_lin_5+m6*transpose(jac_lin_6)*jac_lin_6;
M_q_ang = transpose(jac_ang_1)* rot_z1(1:3,1:3) *I* transpose(rot_z1(1:3,1:3))...
    *jac_ang_1+transpose(jac_ang_2)* rot_z2(1:3,1:3) *I* transpose(rot_z2(1:3,1:3))...
    *jac_ang_2+transpose(jac_ang_3)* rot_z3(1:3,1:3)*I * transpose(rot_z3(1:3,1:3))...
    *jac_ang_3+transpose(jac_ang_4)* rot_z4(1:3,1:3) *I* transpose(rot_z4(1:3,1:3))...
    *jac_ang_4+transpose(jac_ang_5)* rot_z5(1:3,1:3)*I* transpose(rot_z5(1:3,1:3))...
    *jac_ang_5+transpose(jac_ang_6)* rot_z6(1:3,1:3) *I* transpose(rot_z6(1:3,1:3))...
    *jac_ang_6;
M_q= M_q_lin + M_q_ang;

%gravity matrix
syms g_0
g0=[0;g_0;0];

%%%%%%%%%%%%%%%%%%%%%CALCULATING    Corriolis%%%%%%%%%%%%%%%%%%%%%
Corlios = sym(zeros(6));
for i=1:6
    for j=1:6
        eval(strcat('m',string(i),string(j),' = M_q(',string(i),',',string(j),');')) %mij = M_q(i,j)
        eval(strcat('c',string(i),string(j),' = sym(0);'))%cij = sym(0)
        for k=1:6
            eval(strcat('m',string(i),string(k),' = M_q(',string(i),',',string(k),');'))%mik = M_q(i,k)
            eval(strcat('m',string(j),string(k),' = M_q(',string(j),',',string(k),');'))%mjk = M_q(j,k)
           
            eval(strcat('c',string(i),string(j),string(k),' = 0.5*(diff(m',string(i),string(j),',theta_',string(k),')+diff(m',string(i),string(k),',theta_',string(j),')-diff(m',string(j),string(k),',theta_',string(i),'));')) 
            eval(strcat('c',string(i),string(j),' = c',string(i),string(j),' + c',string(i),string(j),string(k),'*dq',string(k),';'))%cij = cij + cijk*dqk;
        end
        eval(strcat('Corlios(',string(i),',',string(j),') = c',string(i),string(j),';'))%C(i,j) = cij
    end
end

%%%%%%%%%%%%%%%%%%%%%CALCULATING    Gravity%%%%%%%%%%%%%%%%%%%%%
g1 = -transpose(jac_lin_1(:,1)) * m1 * g0 -transpose(jac_lin_2(:,1)) * m2 * g0-transpose(jac_lin_3(:,1)) * m3 * g0 - transpose(jac_lin_4(:,1)) * m4 * g0 - transpose(jac_lin_5(:,1)) * m5 * g0 - transpose(jac_lin_6(:,1)) * m6 * g0;
g2 = -transpose(jac_lin_1(:,2)) * m1 * g0 -transpose(jac_lin_2(:,2)) * m2 * g0-transpose(jac_lin_3(:,2)) * m3 * g0 - transpose(jac_lin_4(:,2)) * m4 * g0 - transpose(jac_lin_5(:,2)) * m5 * g0 - transpose(jac_lin_6(:,2))* m6 * g0;
g3 = -transpose(jac_lin_1(:,3)) * m1 * g0 -transpose(jac_lin_2(:,3)) * m2 * g0-transpose(jac_lin_3(:,3)) * m3 * g0 - transpose(jac_lin_4(:,3)) * m4 * g0 - transpose(jac_lin_5(:,3)) * m5 * g0 - transpose(jac_lin_6(:,3))* m6 * g0;
g4 = -transpose(jac_lin_1(:,4)) * m1 * g0 -transpose(jac_lin_2(:,4)) * m2 * g0-transpose(jac_lin_3(:,4)) * m3 * g0 - transpose(jac_lin_4(:,4)) * m4 * g0 - transpose(jac_lin_5(:,4)) * m5 * g0 - transpose(jac_lin_6(:,4))* m6 * g0;
g5 = -transpose(jac_lin_1(:,5)) * m1 * g0 -transpose(jac_lin_2(:,5)) * m2 * g0-transpose(jac_lin_3(:,5)) * m3 * g0 - transpose(jac_lin_4(:,5)) * m4 * g0 - transpose(jac_lin_5(:,5)) * m5 * g0 - transpose(jac_lin_6(:,5))* m6 * g0;
g6 = -transpose(jac_lin_1(:,6)) * m1 * g0 -transpose(jac_lin_2(:,6)) * m2 * g0-transpose(jac_lin_3(:,6)) * m3 * g0 - transpose(jac_lin_4(:,6)) * m4 * g0 - transpose(jac_lin_5(:,6)) * m5 * g0 - transpose(jac_lin_6(:,6))* m6 * g0;


g_final=[g1 ; g2 ; g3 ; g4; g5 ; g6];


%%%%%%%%%%%%%%%%%%%%%CALCULATING    NET TOURQUE%%%%%%%%%%%%%%%%%%%%%

 T = M_q * ddq + Corlios*dq + g_final; 
 
 
 
DH=[20 10 1 0 50 0];  %Enter the values for each joint parameter RRPRRR
L=[1 1 0 1 1 1];  %Robot lengthes
C=[.1 .1  .1 .1 .1]; %Location of each COM relative to the local origin
m = [10 10 10 10 10 10]; %mass of each link
dq=[2 ;2; 4 ;3; 1; 5];%joint velocity
ddq=[1 ;.5 ;1 ;.1 ;.5 ;.6];%joint acceleration
izz=1;  %izz valie
g_0=9.81; % gravity

theta_1=DH(1);
theta_2=DH(2);
q3=DH(3);
theta_4=DH(4);
theta_5=DH(5);
theta_6=DH(6);

m1 = m(1);
m2 = m(2);
m3 = m(3);
m4 = m(4);
m5 = m(5);
m6 = m(6);

c1=C(1);
c2=C(2);
c3=DH(3);
c4=C(3);
c5=C(4);
c6=C(5);

dq1=1;
dq2=1;
dq3=1;
dq4=1;
dq5=1;
dq6=1;

izz=1;
d_1=L(1);
l2=L(2);
d_3=L(3);
d_4=L(4);
d_5=L(5);
d_6=L(6);   
CC=subs(Corlios);