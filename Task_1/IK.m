%These are the pose of the end effector (input to the IK problem)

%Try the same pose obtained from fk and it will result in the same joint
%parameters

nx=0;
ny=0;
nz=1;

sx=.7071;
sy=-.7071;
sz=0;

ax=.7071;
ay=.7071;
az=0;

x=1.4142;
y=1.4142;
z=6;

d1=1;
d2=1;
d4=1;
d5=1;
D6=1+d5;

T_06 =  [nx  sx ax x;
         ny  sy ay y;
         nz  sz az z;
         0   0  0  1];
 new_c=[x;y;z]-D6*[ax;ay;az];
 x=new_c(1);
 y=new_c(2);
 z=new_c(3);
 
% IK Equations as derived in the report
theta1=atan2(-x,y);
theta2=atan2(cos(theta1)*(z-d1),y);
D3=(y/(cos(theta1)*cos(theta2)))-d2-d4;
theta4=atan2((ny*cos(theta1)*cos(theta2)-nz*cos(theta2)-nx*sin(theta1)*sin(theta2)),(-nz*sin(theta2)-ny*cos(theta1)*cos(theta2)+nx*cos(theta1)*cos(theta2)));
theta6=atan2((sx*cos(theta1)+sy*sin(theta1)),(ax*cos(theta1)+ay*sin(theta1)));
theta5=atan2(((sx*cos(theta1)+sy*sin(theta1))/sin(theta6)),(nx*cos(theta1)+ny*sin(theta1)));
