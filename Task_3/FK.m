function Tr = FK(q1,q2,q3,q4,q5,q6)
DH=[q1,q2,q3,q4,q5,q6];
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



end


