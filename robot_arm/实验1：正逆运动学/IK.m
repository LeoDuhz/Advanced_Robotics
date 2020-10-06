
function [wff] = IK(x,y,z,c,b,a)

d6=0.055;d1=0.284;a2=0.225;d4=0.2289;
T06=[cos(a)*cos(b) cos(a)*sin(b)*sin(c)-sin(a)*cos(c) cos(a)*sin(b)*cos(c)+sin(a)*sin(c) x
    sin(a)*cos(b) sin(a)*sin(b)*sin(c)+cos(a)*cos(c) sin(a)*sin(b)*cos(c)-cos(a)*sin(c) y
    -sin(b) cos(b)*sin(c) cos(b)*cos(c) z
    0 0 0 1];

Tt=T06;

trans=[1 0 0 0;0 0 -1 0;0 1 0 0; 0 0 0 1];
T06=T06*trans;

%calculate theta1  :2
w1(1)=atan((d6*T06(2,3)-y)/(d6*T06(1,3)-x));
w1(2)=w1(1)+pi;

%calculate theta3  :4
w3(1)=asin(((cos(w1(1))*x-d6*cos(w1(1))*T06(1,3)-T06(2,3)*sin(w1(1))*d6+sin(w1(1))*y)^2+(z-d1-d6*T06(3,3))^2-a2^2-d4^2)/(2*a2*d4));
w3(2)=pi-asin(((cos(w1(1))*x-d6*cos(w1(1))*T06(1,3)-T06(2,3)*sin(w1(1))*d6+sin(w1(1))*y)^2+(z-d1-d6*T06(3,3))^2-a2^2-d4^2)/(2*a2*d4));
w3(3)=asin(((cos(w1(2))*x-d6*cos(w1(2))*T06(1,3)-T06(2,3)*sin(w1(2))*d6+sin(w1(2))*y)^2+(z-d1-d6*T06(3,3))^2-a2^2-d4^2)/(2*a2*d4));
w3(4)=pi-asin(((cos(w1(2))*x-d6*cos(w1(2))*T06(1,3)-T06(2,3)*sin(w1(2))*d6+sin(w1(2))*y)^2+(z-d1-d6*T06(3,3))^2-a2^2-d4^2)/(2*a2*d4));

%calculate theta2  :8
w2=zeros(1,8);
for i=1:8
    t=mod(i,4);
    if t==0
        t=4;
    end
temp1=sqrt((d4*sin(w3(t))+a2)^2+d4*d4*cos(w3(t))*cos(w3(t)));
temp2=acos((d4*sin(w3(t))+a2)/temp1);

if mod(i,2)==1
w2(i)=asin((z-d6*T06(3,3)-d1)/temp1)+temp2;
else
   w2(i)=pi-(asin((z-d6*T06(3,3)-d1)/temp1)+temp2);
end

end

%calculate theta5  :16
w5=zeros(1,16);
for i=1:16
   t1=ceil(i/8);
   t3=ceil(i/4);
   t2=ceil(i/2);
if mod(i,2)==1
w5(i)=acos(T06(2,3)*sin(w1(t1))*sin(w2(t2)+w3(t3))+cos(w1(t1))*T06(1,3)*sin(w2(t2)+w3(t3))-T06(3,3)*cos(w2(t2)+w3(t3)));
else
   w5(i)=-acos(T06(2,3)*sin(w1(t1))*sin(w2(t2)+w3(t3))+cos(w1(t1))*T06(1,3)*sin(w2(t2)+w3(t3))-T06(3,3)*cos(w2(t2)+w3(t3)));
end
end


%calculate theta4 and theta 6 together  :32
w4=zeros(1,32);
w6=zeros(1,32);
w=zeros(32,6);
for i=1:32
    t1=ceil(i/16);
    t2=ceil(i/8);
    t3=ceil(i/4);
    t4=ceil(i/2);
if mod(i,2)==1
w4(i)=asin((-cos(w1(t1))*T06(2,3)+T06(1,3)*sin(w1(t1)))/sin(w5(t4)));
temp1=T06(1,2)*cos(w1(t1))*sin(w2(t3)+w3(t2))+T06(2,2)*sin(w1(t1))*sin(w2(t3)+w3(t2))-T06(3,2)*cos(w2(t3)+w3(t2));
w6(i)=asin(temp1/sin(w5(t4)));

else
w4(i)=pi-asin((-cos(w1(t1))*T06(2,3)+T06(1,3)*sin(w1(t1)))/sin(w5(t4)));
temp1=T06(1,2)*cos(w1(t1))*sin(w2(t3)+w3(t2))+T06(2,2)*sin(w1(t1))*sin(w2(t3)+w3(t2))-T06(3,2)*cos(w2(t3)+w3(t2));
w6(i)=pi-asin(temp1/sin(w5(t4)));

end
%put results into w
w(i,:)=[w1(t1) w2(t3)-pi/2 w3(t2) w4(i) w5(t4)+pi/2 w6(i)];
end


%delete the results out of scale
count=0;
wf=zeros(32,6);
for i=1:32

    if check(w(i,1),-pi,pi)==-1
        continue
    else
        w(i,1)=w(i,1)+pi*check(w(i,1),-pi,pi);
    end
    
    if check(w(i,2),-deg2rad(115),deg2rad(115))==-1
        continue
    else
        w(i,2)=w(i,2)+pi*check(w(i,2),-deg2rad(115),deg2rad(115));
    end
    
    if check(w(i,3),-deg2rad(130),deg2rad(130))==-1
        continue
    else
        w(i,3)=w(i,3)+pi*check(w(i,3),-deg2rad(40),deg2rad(220));
    end
    
    if check(w(i,4),-deg2rad(180),deg2rad(180))==-1
        continue
    else
        w(i,4)=w(i,4)+pi*check(w(i,4),-deg2rad(180),deg2rad(180));
    end
    
%     if check(w(i,5),-deg2rad(75),deg2rad(255))==-1
%         continue
%     else
%         w(i,5)=w(i,5)+pi*check(w(i,5),-deg2rad(75),deg2rad(255));
%     end
    if check(w(i,5),-0.78,3.92)==-1
        continue
    else
        w(i,5)=w(i,5)+pi*check(w(i,5),-0.78,3.92);
    end



    if check(w(i,6),-pi,pi)==-1
        continue
    else
        w(i,6)=w(i,6)+pi*check(w(i,6),-pi,pi);
    end
    count=count+1;
    wf(count,:)=w(i,:);
end
wf=wf(1:count,:);

%delete the results that could not be calculated by zhengyundongxue
Ncount=0;
wff=zeros(count,6);
for i=1:count
temp=DK(wf(i,:))-Tt;
t=norm(temp,1);
if t<1e-4
    Ncount=Ncount+1;
    wff(Ncount,:)=wf(i,:);
end

end
wff=wff(1:Ncount,:);
end

%check the rad and + -2pi with the scale
function [c]=check(w,a,b)
c=-1;
if (a<w)&&(w<b)
    c=0;
end

if (a<w+2*pi)&&(w+2*pi<b)
    c=2;
end

if (a<w-2*pi)&&(w-2*pi<b)
        c=-2;
end

end