clear all; close all; clc;

Vx=1;
Vy=0;
Vz=0;

% Vx=0;
Vy=-sqrt(3);
% Vy=1;
Vz=0;

Vt=Vx+Vy;

W=.5;

L=5*cos(45/(180/pi));

cos1=cos(-60/(180/pi));
cos2=cos(60/(180/pi));
sin1=sin(-60/(180/pi));
sin2=sin(60/(180/pi));

m1 = [L*cos(-150/(180/pi))+W*cos1 L*cos(-150/(180/pi))-W*cos1 L*sin(-150/(180/pi))+W*sin1 L*sin(-150/(180/pi))-W*sin1];
m2 = [L*cos(-30/(180/pi))+W*cos2 L*cos(-30/(180/pi))-W*cos2 L*sin(-30/(180/pi))+W*sin2 L*sin(-30/(180/pi))-W*sin2];
m3 = [-W W L L];




% speed1 = -((Vx)+(Vy/(2*sin1)))+Vz
% speed2 = -((Vx)+(Vy/(2*sin2)))+Vz
% speed3 = -Vx+Vz

speed1 = -(((1/3)*Vx)+((-2/sqrt(3))*Vy))
speed2 = -(((2/3)*Vx)+((2/sqrt(3))*Vy))
speed3 = -(((-1)*Vx))

speed1 = (sqrt(3)/2)*(((1/sqrt(3))*Vx)+(-1*Vy))
speed2 = (sqrt(3)/2)*(((1/sqrt(3))*Vx)+(1*Vy))
speed3 = (sqrt(3)/2)*((-(2/sqrt(3)*Vx)))

speed1 = (sin(atan(Vy/Vx)))*(((-2/sqrt(3))*Vx))+((2)*Vy)
speed2 = (sin(atan(Vy/Vx)))*((((-2/sqrt(3))*Vx))+((-2)*Vy))
speed3 = (Vx)

speed1 = (((-1/2)*Vx))+((sqrt(3)/2)*Vy)
speed2 = ((((-1/2)*Vx))+(-(sqrt(3)/2)*Vy))
speed3 = (Vx)


s1 = [speed1*cos1 speed1*sin1]
s2 = [speed2*cos2 speed2*sin2]
s3 = [speed3 0]

X = (speed1*cos1)+(speed2*cos2)-speed3
Y = (speed1*sin1)+(speed2*sin2)

figure
hold on
grid on
plot([0 L*cos(-30/(180/pi))],[0 L*sin(-30/(180/pi))],'k','LineWidth',3)
plot([0 L*cos(-150/(180/pi))],[0 L*sin(-150/(180/pi))],'k','LineWidth',3)
plot([0 0],[0 L],'k','LineWidth',3)

plot([m1(1) m1(2)],[m1(3) m1(4)],'k','LineWidth',10)
plot([m2(1) m2(2)],[m2(3) m2(4)],'k','LineWidth',10)
plot([m3(1) m3(2)],[m3(3) m3(4)],'k','LineWidth',10)

plot([0 Vx],[0 Vy],'g','LineWidth',5)
% plot([0 -Vx],[0 -Vy],'y','LineWidth',5)

plot([L*cos(-150/(180/pi)) L*cos(-150/(180/pi))+s1(1)],[L*sin(-150/(180/pi)) L*sin(-150/(180/pi))+s1(2)],'r','LineWidth',5)
plot([L*cos(-30/(180/pi)) L*cos(-30/(180/pi))+s2(1)],[L*sin(-30/(180/pi)) L*sin(-30/(180/pi))+s2(2)],'r','LineWidth',5)
plot([0 -s3(1)],[L L],'r','LineWidth',5)

plot([0 s1(1)+s2(1)],[-L -L],'b','LineWidth',5)

r=5*cos(pi/4);
th = 0:pi/50:2*pi;
xunit = r * cos(th);
yunit = r * sin(th);
plot(xunit, yunit);

xlim([-4 4]);
ylim([-4 4]);

set(gcf,'Position',[0 510 510 500])
