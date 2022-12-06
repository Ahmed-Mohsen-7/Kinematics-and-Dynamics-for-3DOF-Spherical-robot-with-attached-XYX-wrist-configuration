
DH=[50 30 1 20 40 0];
L=[1 1 0 1 1 1];  %The third link should be zero. It is a DH parameter
base_frame=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];   %base frame (joint1)coordinates

q1=DH(1);
q2=DH(2);
q3=DH(3);
q4=DH(4);
q5=DH(5);
q6=DH(6);

l1=L(1);
l2=L(2);
l4=L(4);
l5=L(5);
l6=L(6);

D3=l2+q3+l4;
D6=l5+l6;

a1 = rotm2tform(rotz(q1))*rotm2tform(rotz(90))*trvec2tform([0 0 l1])*rotm2tform(rotx(90));
a2 = rotm2tform(rotz(q2))*rotm2tform(rotz(90))*rotm2tform(rotx(90));
a3 = rotm2tform(rotz(90))*trvec2tform([0 0 D3]);
a4 = rotm2tform(roty(90))*rotm2tform(rotz(q4))*rotm2tform(rotz(180))*rotm2tform(rotx(90));
a5 = rotm2tform(rotz(q5))*rotm2tform(rotz(180))*rotm2tform(rotx(90));
a6 = rotm2tform(rotz(q6))*trvec2tform([-D6 0 0])*rotm2tform(roty(-90));


Tr=a1*a2*a3*a4*a5*a6;   %Transformation matrix from 0 to 6  (FK output)


%Position and orientation of each frame
p1= base_frame*a1;
p2= p1*a2;
p3= p2*a3;
p4= p3*a4;
p5= p4*a5;
p6= p5*a6;


%x,y,z coordinates of all frames
x_coordinates=[base_frame(1,4) p1(1,4) p2(1,4) p3(1,4) p4(1,4) p5(1,4) p6(1,4)];
y_coordinates=[base_frame(2,4) p1(2,4) p2(2,4) p3(2,4) p4(2,4) p5(2,4) p6(2,4)];
z_coordinates=[base_frame(3,4) p1(3,4) p2(3,4) p3(3,4) p4(3,4) p5(3,4) p6(3,4)];


%Plot the coordinates 
%Note that frames 1 and 2 are assumed to be on the same origin
%Note that frames 3,4,5 are assumed to be on the same origin
plot3(x_coordinates,y_coordinates,z_coordinates,'o-')
axis ([-5 5 -5 5 0 3 ])
xlabel('x')
ylabel('y')
zlabel('z')
grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Jacobian Matrix%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This to put all the transformations into a multi-dimensional array (to be used in the for loop)
frames(:,:,1)=base_frame;
frames(:,:,2)=p1;
frames(:,:,3)=p2;
frames(:,:,4)=p3;
frames(:,:,5)=p4;
frames(:,:,6)=p5;
frames(:,:,7)=p6;
j=zeros(6);    %empty jacobian matrix
pe = frames(1:3,4,7);   %Position of end effector


%This loop iterate for each joint and caluclate its jacobian based on the
%equation
for i=1:6
frame= frames(:,:,i);
pi_1 = frame(1:3,4);
zi_1 = frame(1:3,3);

if i==3      %jacobian for prismatic joint
        j(1:3,i)=zi_1;
        j(4:6,i)=[0 ;0 ;0];
else          %jacobian for revolute joint
        jp =cross(zi_1,pe-pi_1);

        j(1:3,i)=jp;
        j(4:6,i)=zi_1;
end


end


