
clear;clc;close all;
currentdir = pwd;
%logn = 67;
%path = strcat(currentdir,'\log',num2str(logn),'.txt');

logname = 'Saturn V - Lunar_log2';
path = strcat(currentdir,'\',logname,'.txt');


A = dlmread(path,'', 1, 0);


t=A(:,1);
alt=A(:,2);
dst=A(:,3);
stg=A(:,4);
m=A(:,5);
twr=A(:,6);
thr=A(:,7);
az=A(:,8);
haoa=A(:,9);
pch=A(:,10);
vaoa=A(:,11);
surfv=A(:,12);
orbv=A(:,13);
incl=A(:,14);
ap=A(:,15);
ap=ap*6571;
t=t/60;


figure
%grid on
plot(dst,alt);
title('Altitude vs Downrange Distance');
grid on

figure
grid on
plot(t,m);
title('Mass vs Time');
xticks(0:1:12)
grid on

figure
grid on
plot(t,twr);
title('TWR vs Time');
xticks(0:1:12)
grid on

figure
grid on
plot(t,pch);
title('Pitch vs Time');
xticks(0:1:12)
grid on

figure
plot(t,vaoa);
title('AOA vs Time');
grid on
xticks(0:1:12)

figure
plot(t,orbv);
title('Orbital Velocity vs Time');
xticks(0:1:12)
grid on

figure
plot(t,ap);
title('Apoapsis vs Time');
xticks(0:1:12)
grid on



return


dt = 0.05;
time = 0:dt:A(778,1);
figure(1)
%hold all
grid on
xlabel('TIME (s)') % x-axis label
ylabel('TWR') % y-axis label
axis([-5,525,0,3.5])
width=960;
height=272;
set(gcf,'units','points','position',[0,0,width,height])


plott = animatedline('Color','r','LineWidth',1);
F= struct('cdata',[],'colormap',[]);
x=1;
writerObj = VideoWriter('test2.avi');
open(writerObj);
for i = 1:length(time)

            
        if time(i)>=A(x,1)
            addpoints(plott,A(x,1),A(x,6)); 
            x = x + 1;
        
            drawnow ;
            
        end
         F =  getframe(gcf)  ;
             writeVideo(writerObj, F)
        %pause(dt)

end

hold off



close(writerObj);


% 
% time = 100;
% for t = 1:time
%    fplot(@(x) sin(x*50/t),[0,2*pi]);  % plot
%    ylim([-1,1]);                      % guarantee consistent height
%    F(t) = getframe;                   % capture it
% end
% 
% writerObj = VideoWriter('test2.avi');
% open(writerObj);
% writeVideo(writerObj, F)
% close(writerObj);