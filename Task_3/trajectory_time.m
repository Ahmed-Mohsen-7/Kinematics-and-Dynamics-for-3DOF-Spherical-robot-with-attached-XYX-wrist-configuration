%Function to calculate trajectory time

function [t0,t1,T,tf] = trajectory_time(q0,qf,dq_m,ddq_m,t0)
t0=t0;
dq = qf-q0;
%triangular check 
c = sqrt(dq*ddq_m); 

if c <= dq_m  %triangular
    t1 = sqrt(dq/ddq_m) ;
    T = t1;
    tf = 2*t1;
else 
    t1 = dq_m/ddq_m+t0; %trapazoidal
    T = dq/dq_m +t0;
    tf = T+t1 ;
end


end
