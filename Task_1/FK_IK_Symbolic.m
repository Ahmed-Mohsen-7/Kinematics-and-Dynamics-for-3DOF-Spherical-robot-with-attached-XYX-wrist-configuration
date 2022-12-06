%This code can solve IK and FK symbolaically and thus saving much time in
%manual solvign for the problem 

%This matlab code is intended for symbolic IK and IK
%The first part contains basic rotaiton and translation matrices along x,y,z axes


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Part_1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        
syms d_3
trans_x3 =  [1  0 0 d_3;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
trans_z3 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 d_3;
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
syms d_5          
trans_z5 =  [1  0 0 0;
            0  1 0 0;
            0  0 1 d_5;
            0  0 0 1];

syms d_6          
trans_x6 =  [1  0 0 -d_6;
            0  1 0 0;
            0  0 1 0;
            0  0 0 1];
 
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Part_1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%This matrix represents an input matrix to the IK problem
syms nx ny nz sx sy sz ax ay az x y z
A_06 =     [nx  sx ax x;
            ny  sy ay y;
            nz  sz az z;
            0   0  0  1];
        
        
        

        
%These six matrices represnt the transformation between each two consecutive joints of the model        
a1 = rot_z1*rotm2tform(rotz(90))*trans_z1*rotm2tform(rotx(90));
a2 = rot_z2*rotm2tform(rotz(90))*rotm2tform(rotx(90));
a3 = rotm2tform(rotz(90))*trans_z3;
a4 = rotm2tform(roty(90))*rot_z4*rotm2tform(rotz(180))*rotm2tform(rotx(90));
a5 = rot_z5*rotm2tform(rotz(180))*rotm2tform(rotx(90));
a6 = rot_z6*trans_x6*rotm2tform(roty(-90));


%This is fk final matrix
fk=a1*a2*a3*a4*a5*a6;


%The following matrices are used in IK soltuion
A_03=a1*a2*a3;
A_36=a4*a5*a6;

R_03 = A_03([1 2 3],[1 2 3]);
R_06=A_06([1 2 3],[1 2 3]);

R_36_new= transpose(R_03) * R_06;