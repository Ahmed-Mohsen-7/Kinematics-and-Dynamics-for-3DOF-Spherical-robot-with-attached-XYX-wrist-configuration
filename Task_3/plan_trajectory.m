%Function to calculate trajectory planning
function [t,q,v,a] = plan_trajectory(q0,qf,dq_m,ddq_m,t0,t1,T,tf)


t = linspace(t0,tf, 3000);
q = [];
v = [];
a = [];

for i = t
    if i <=t1
      qi = q0 + (0.5*ddq_m*(i-t0)^2);
      q02 = qi;
      vi = ddq_m*(i-t0);
      v02 = vi ;
      ai = ddq_m;
      
    elseif i > t1 && i <= T 
      vi = dq_m;
      qi =  q02 + v02*(i-t1);
      ai = 0 ;
      
    elseif i > T
      vi = ddq_m*(tf-i-t0);
      qi = qf - (0.5*ddq_m*(i-tf)^2);
      ai = -ddq_m;
    end
q=[q,qi];
v=[v,vi];
a=[a,ai];

end
end
%[t,q,v,a]=plan_trajectory(0,90, 8,4,0,2,11.25,13.25);