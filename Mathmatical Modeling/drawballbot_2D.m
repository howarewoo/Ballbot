function drawballbot_2D(x,y,m,M,L,t,tspan,motor)
xx = x(1);
thx = x(3);
xy = y(1);
thy = y(3);

% kinematics
% x = 3;        % cart position
% th = 3*pi/2;   % pendulum angle

% dimensions
% L = 2;  % pendulum length
W = 0.254;  % ball width (m)
H = 0.254; % ball height (m)
mr = .05*sqrt(m); % mass radius

% positions
% y = wr/2; % cart vertical position
x = H/2; % cart vertical position
y = H/2; % cart vertical position

pxx = xx + L*sin(thx);
pyx = x - L*cos(thx);
pxy = xy + L*sin(thy);
pyy = y - L*cos(thy);

subplot(2,1,1)
plot([-10 10],[0 0],'w','LineWidth',2)
hold on
rectangle('Position',[xx-W/2,x-H/2,W,H],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])

plot([xx pxx],[x 1.5*pyx],'w','LineWidth',5)

rectangle('Position',[pxx-mr/2,pyx-mr/2,mr,mr],'Curvature',.1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

xlim([-2.5 2.5]);
ylim([-.5 1]);
set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   

% box off
drawnow
hold off
% 
subplot(2,1,2)
plot([-10 10],[0 0],'w','LineWidth',2)
hold on
rectangle('Position',[xy-W/2,y-H/2,W,H],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])

plot([xy pxy],[y 1.5*pyy],'w','LineWidth',5)

rectangle('Position',[pxy-mr/2,pyy-mr/2,mr,mr],'Curvature',.1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-2.5 2.5]);
ylim([-.5 1]);
set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')   

box off
drawnow
hold off

% subplot(5,1,3);
% plot(tspan,motor(:,1));
% title('Motor Velocities with State-Feedback Control and Noise')
% ylim([-1 1]);
% xlim([0 t]);
% grid on
% set(gca,'Color','k','XColor','w','YColor','w')
% set(gcf,'Color','k')
% set(gcf,'InvertHardcopy','off') 
% 
% subplot(5,1,4);
% plot(tspan,motor(:,2));
% ylim([-1 1]);
% xlim([0 t]);
% grid on
% set(gca,'Color','k','XColor','w','YColor','w')
% set(gcf,'Color','k')
% set(gcf,'InvertHardcopy','off') 
% 
% subplot(5,1,5);
% plot(tspan,motor(:,3));
% ylim([-1 1]);
% xlim([0 t]);
% grid on
% set(gca,'Color','k','XColor','w','YColor','w')
% set(gcf,'Color','k')
% set(gcf,'InvertHardcopy','off') 
% 
set(gcf,'Position',[0 1100 1100 700])
% drawnow
% hold off
