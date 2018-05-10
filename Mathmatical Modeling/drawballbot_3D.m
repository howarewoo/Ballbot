function drawballbot_3D(xx,xy,m,M,L,t,Ts,tspan)

for k=1:1/(10*Ts):length(tspan)


%     hold on
%     subplot(4,2,5);
%     [AX,H1,H2] = plotyy(0:k-1,xx(1:k,1),0:k-1,(xx(1:k,3)-pi)*(180/pi),'plot');
%     grid on
%     set(get(AX(1),'Ylabel'),'String','ball position (m)')
%     set(get(AX(2),'Ylabel'),'String','chassis angle (degrees)')
%     title('Ballbot Positions along the X-Axis')
% 
%     subplot(4,2,7);
%     [AX,H1,H2] = plotyy(0:k-1,xx(1:k,2),0:k-1,xx(1:k,4)*(180/pi),'plot');
%     grid on
%     set(get(AX(1),'Ylabel'),'String','ball velocity (m/s)')
%     set(get(AX(2),'Ylabel'),'String','chassis angle (degrees/s)')
%     title('Ballbot Velocities along the X-Axis')
% 
%     subplot(4,2,6);
%     [AX,H1,H2] = plotyy(0:k-1,xy(1:k,1),0:k-1,(xy(1:k,3)-pi)*(180/pi),'plot');
%     grid on
%     set(get(AX(1),'Ylabel'),'String','ball position (m)')
%     set(get(AX(2),'Ylabel'),'String','chassis angle (degrees)')
%     title('Ballbot Positions along the Y-Axis')
% 
%     subplot(4,2,8);
%     [AX,H1,H2] = plotyy(0:k-1,xy(1:k,2),0:k-1,xy(1:k,4)*(180/pi),'plot');
%     grid on
%     set(get(AX(1),'Ylabel'),'String','ball velocity (m/s)')
%     set(get(AX(2),'Ylabel'),'String','chassis angle (degrees/s)')
%     title('Ballbot Velocities along the Y-Axis')

    x = xx(k,1);
    y = xy(k,1);
    thx = xx(k,3);
    thy = xy(k,3);

    % kinematics
    % x = 3;        % cart position
    % th = 3*pi/2;   % pendulum angle

    % dimensions
    % L = 2;  % pendulum length
    r = 0.254/2;  % ball radius (m)
    mr = .05*sqrt(m); % mass radius

    % positions
    % y = wr/2; % cart vertical position
    z = r; % cart vertical position

    px = x + L*sin(thx);
    py = y + L*sin(thy);
    pz = x*tan(thx);

    % plot([-10 10],[0 0],'w','LineWidth',2)
%     subplot(4,2,[1,3]);
    [X,Y,Z] = sphere;
    surf((X*r)+x,(Y*r)+y,(Z*r)+z,'LineStyle','none');
    hold on

    [X,Y,Z] = cylinder(r);
    h=surf(X+x,Y+y,Z+z,'FaceColor','g');
    rotate(h,[1 0 0],thx*(180/pi),[x y r]);
    rotate(h,[0 1 0],thy*(180/pi),[x y r]);

    plot3([x x+xx(k,2)],[y y+xy(k,2)],[0 0 ],'r','LineWidth',5)



    % rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])

    % rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',.1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

    % set(gca,'YTick',[])
    % set(gca,'XTick',[])
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([0 1]);
    % set(gca,'Color','k','XColor','w','YColor','w')
    % set(gcf,'Color','k')
    % set(gcf,'InvertHardcopy','off')   

    % box off
%     drawnow
%     hold off

%     subplot(4,2,[2,4]);
%     m1 = -(xx(k,2)/2)+(0.866*(xy(k,2)));
%     m2 = -(xx(k,2)/2)-(0.866*(xy(k,2)));
%     m3 = xx(k,2);
% 
%     mp1 = 0.066715892579886*sin(45);
%     mp2 = 0.066715892579886*sin(45);
%     mp3 = 0.066715892579886;
%    
%     plot([0 xx(2)],[0 xy(2)],'g','LineWidth',5)
%     hold on
%     plot([-mp1 -mp1-(m1*.5)],[-mp1 -mp1+(m1*.5)],'r','LineWidth',5)
%     plot([mp2 mp2-(m2*.5)],[-mp2 -mp2-(m2*.5)],'r','LineWidth',5)
%     plot([0 m3],[mp3 mp3],'r','LineWidth',5)
%     grid on
%     % set(gca,'YTick',[])
%     % set(gca,'XTick',[])
%     xlim([-.5 .5]);
%     ylim([-.5 .5]);
%     % set(gca,'Color','k','XColor','w','YColor','w')
%     % set(gcf,'Color','k')
%     % set(gcf,'InvertHardcopy','off')   
% 
%     % box off
    drawnow
    hold off
    set(gcf,'Position',[0 1300 1300 800])
end
