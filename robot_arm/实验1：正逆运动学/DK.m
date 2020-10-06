function [T] = DK(w)

T1 = solve(0,0,0.284,w(1));
T2 = T1*solve(pi/2,0,0,pi/2+w(2));
T3 = T2*solve(0,0.225,0,w(3));
T4 = T3*solve(pi/2,0,0.2289,w(4));
T5 = T4*solve(-pi/2,0,0,-pi/2+w(5));
T6 = T5*solve(pi/2,0,0.055,w(6));
T = T6*solve(-pi/2,0,0,0);

%disp('x,y,z')
T(1:3,4); %x,y,z
pitch = asin(-T(3,1));%beta
yaw = asin(T(2,1)/cos(pitch));%alpha
roll = acos(T(3,3)/cos(pitch));%gamma
%disp('roll,yaw,pitch')
roll;
pitch;
yaw;

end

function [Ttemp] = solve(alpha,a,d,w)
Ttemp=[cos(w) -sin(w) 0 a
    sin(w)*cos(alpha) cos(w)*cos(alpha) -sin(alpha) -sin(alpha)*d
    sin(w)*sin(alpha) cos(w)*sin(alpha) cos(alpha) cos(alpha)*d
    0 0 0 1];
end
