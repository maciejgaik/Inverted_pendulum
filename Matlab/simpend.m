clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

dt=.001;

tspan = 0:dt:100;
x0 = [0; 0; pi-0.5; 0];


A = [0    1            0        0;
     0  -d/M         m*g/M      0;
     0    0            0        1;
     0 -d/(M*L) -(m+M)*g/(M*L)  0];

B = [0; 1/M; 0; 1/(M*L);];

% eig(A)
% rank(ctrb(A,B))

Q=eye(4);
R=.0001;

K=lqr(A,B,Q,R);

xend=[1;0;pi;0];
% u=@(x)-K*(x-xend);

% pidArr = [300  10;       %[KpTh KpX;
%           33   0.5;      % KiTh KiX;
%           75   10;];     % KdTh KdX;];
      
pidArr = [250  10;       %[KpTh KpX;
          30   0.5;      % KiTh KiX;
          170   10;];     % KdTh KdX;];

u=@(x) mypid(x,pidArr,dt);
[t,x] = ode45(@(t,x)cartpend(x,m,M,L,g,d,u(x)),tspan,x0);

figure(1);
plot(tspan,x.','LineWidth',2); hold on;
l1 = legend('x','v','\theta','\omega');
set(l1,'Location','SouthEast');
%set(gcf,'Position',[100 100 500 200]);
xlabel('Time');
ylabel('State');
grid on;
%set(gcf,'PaperPositionMode','auto');

for t=1:100:length(tspan)
    drawpend(x(t,:),m,M,L);
end 
