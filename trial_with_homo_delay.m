% Controller modified from "Nonlinear Consensus-Based Connected Vehicle Platoon Control
% Incorporating Car-Following Interactions and Heterogeneous Time Delays"

% Including Homogeneous Communication Time Delay

%%
clc
clear all

%% Constants
alpha=1; % displacement constant
beta=1; % velocity constant
gamma=1; % acceleration constant

A=[0,1,0;
   0,0,1;
   0,0,0];

B=[0,0,0;
   0,0,0;
   alpha,beta,gamma];

%% Timing of Simulation
time=60; % total simulation time set at 60
h=0.01; % sampling interval set at 0.01
len=length(0:h:time);

%% Initialising Controller Values
u0=zeros(3,len);
u1=zeros(3,len);
u2=zeros(3,len);
u3=zeros(3,len);
u4=zeros(3,len);
u5=zeros(3,len);
u6=zeros(3,len);

%% Initial Values
x0(:,1)=10*[6; 5; 0.1926]; % [displacement, velocity, acceleration]
x1(:,1)=8*[5; 2; 0.2926];
x2(:,1)=6*[4; 3; 0.1623];
x3(:,1)=5*[3; 1; 0.4508];
x4(:,1)=4*[2; 2; 0.4508];
x5(:,1)=2*[1; 1; 0.1182];
x6(:,1)=1*[0; 1; 0.2107];

%% Pre-Set Safety Distances
r0=[0; 0; 0];
r1=3*[-30; 0; 0];
r2=3*[-30; 0; 0];
r3=3*[-30; 0; 0];
r4=3*[-30; 0; 0];
r5=3*[-30; 0; 0];
r6=3*[-30; 0; 0];

%% Homogeneous Time Delay Values
%d=1;

%% Control Law

u0(:,1)=0;
u1(:,1)=-(x1(:,1)-x0(:,1)-r1);
u2(:,1)=-2*(x2(:,1)-x1(:,1)-r2);
u3(:,1)=-4*(x3(:,1)-x2(:,1)-r3);
u4(:,1)=-(x4(:,1)-x3(:,1)-r4);
u5(:,1)=-(x5(:,1)-x4(:,1)-r5);
u6(:,1)=-3*(x6(:,1)-x5(:,1)-r6);

for k=2:1:len
    % for leader
    if (x0(2,k-1)>=70)
        x0(2,k-1)=70;
    elseif (x0(2,k-1)<0)
        x0(2,k-1)=0;
    else
        x0(2,k-1)=x0(2,k-1);
    end
    
    k1=h*(A*x0(:,k-1));
    k2=h*(A*(x0(:,k-1)+0.5*k1));
    k3=h*(A*(x0(:,k-1)+0.5*k2));
    k4=h*(A*(x0(:,k-1)+k3));
    x0(:,k)=x0(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
    
    % for follower 1
    if (x1(2,k-1)>=70)
        x1(2,k-1)=70;
    elseif (x1(2,k-1)<0)
        x1(2,k-1)=0;
    else
        x1(2,k-1)=x1(2,k-1);
    end

    k1=h*(A*x1(:,k-1)+B*u1(:,k-1));
    k2=h*(A*(x1(:,k-1)+0.5*k1));
    k3=h*(A*(x1(:,k-1)+0.5*k2));
    k4=h*(A*(x1(:,k-1)+k3));   
    x1(:,k)=x1(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
    
    % for follower 2
    if (x2(2,k-1)>=70)
        x2(2,k-1)=70;
    elseif (x2(2,k-1)<0)
        x2(2,k-1)=0;
    else
        x2(2,k-1)=x2(2,k-1);
    end

    k1=h*(A*x2(:,k-1)+B*u2(:,k-1));        
    k2=h*(A*(x2(:,k-1)+0.5*k1));
    k3=h*(A*(x2(:,k-1)+0.5*k2));
    k4=h*(A*(x2(:,k-1)+k3));   
    x2(:,k)=x2(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
    
    % for follower 3
    if (x3(2,k-1)>=70)
        x3(2,k-1)=70;
    elseif (x3(2,k-1)<0)
        x3(2,k-1)=0;
    else
        x3(2,k-1)=x3(2,k-1);
    end

    k1=h*(A*x3(:,k-1)+B*u3(:,k-1));
    k2=h*(A*(x3(:,k-1)+0.5*k1));
    k3=h*(A*(x3(:,k-1)+0.5*k2));
    k4=h*(A*(x3(:,k-1)+k3));
    x3(:,k)=x3(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
        
    % for follower 4
    if (x4(2,k-1)>=70)
        x4(2,k-1)=70;
    elseif (x4(2,k-1)<0)
        x4(2,k-1)=0;
    else
        x4(2,k-1)=x4(2,k-1);
    end

    k1=h*(A*x4(:,k-1)+B*u4(:,k-1));        
    k2=h*(A*(x4(:,k-1)+0.5*k1));
    k3=h*(A*(x4(:,k-1)+0.5*k2));
    k4=h*(A*(x4(:,k-1)+k3));   
    x4(:,k)=x4(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
    
    % for follower 5
    if (x5(2,k-1)>=70)
        x5(2,k-1)=70;
    elseif (x5(2,k-1)<0)
        x5(2,k-1)=0;
    else
        x5(2,k-1)=x5(2,k-1);
    end

    k1=h*(A*x5(:,k-1)+B*u5(:,k-1));        
    k2=h*(A*(x5(:,k-1)+0.5*k1));
    k3=h*(A*(x5(:,k-1)+0.5*k2));
    k4=h*(A*(x5(:,k-1)+k3));   
    x5(:,k)=x5(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
    
    % for follower 6
    if (x6(2,k-1)>=70)
        x6(2,k-1)=70;
    elseif (x6(2,k-1)<0)
        x6(2,k-1)=0;
    else
        x6(2,k-1)=x6(2,k-1);
    end

    k1=h*(A*x6(:,k-1)+B*u6(:,k-1));        
    k2=h*(A*(x6(:,k-1)+0.5*k1));
    k3=h*(A*(x6(:,k-1)+0.5*k2));
    k4=h*(A*(x6(:,k-1)+k3));   
    x6(:,k)=x6(:,k-1)+(1/6)*(k1+2*k2+2*k3+k4);
    %%%%%%%%%%%%%%%%%%%%%
    
    if (k>5)
        d=5;
        u1(:,k)=-(x1(:,k)-x0(:,k-d)-r1);
        u2(:,k)=-(x2(:,k)-x1(:,k-d)-r2);
        u3(:,k)=-(x3(:,k)-x2(:,k-d)-r3);
        u4(:,k)=-(x4(:,k)-x3(:,k-d)-r4);
        u5(:,k)=-(x5(:,k)-x4(:,k-d)-r5);
        u6(:,k)=-(x6(:,k)-x5(:,k-d)-r6);
    else
        d=1;
        u1(:,k)=-(x1(:,k)-x0(:,k-d)-r1);
        u2(:,k)=-(x2(:,k)-x1(:,k-d)-r2);
        u3(:,k)=-(x3(:,k)-x2(:,k-d)-r3);
        u4(:,k)=-(x4(:,k)-x3(:,k-d)-r4);
        u5(:,k)=-(x5(:,k)-x4(:,k-d)-r5);
        u6(:,k)=-(x6(:,k)-x5(:,k-d)-r6);
    end
end

%% Figure 1: Plotting Displacement Curves
figure
plot(0:h:time, x0(1,:),'-k','LineWidth',2)
hold on
plot(0:h:time, x1(1,:),'-r','LineWidth',2)
hold on
plot(0:h:time, x2(1,:),'-g','LineWidth',2)
hold on
plot(0:h:time, x3(1,:),'-b','LineWidth',2)
hold on
plot(0:h:time, x4(1,:),'-m','LineWidth',2)
hold on
plot(0:h:time, x5(1,:),'-c','LineWidth',2)
hold on
plot(0:h:time, x6(1,:),'-y','LineWidth',2)
hold on
grid on
xlabel('Time (second)','Fontname', 'Times New Roman','fontsize',10)
ylabel('The displacement of the vehicles','Fontname', 'Times New Roman','fontsize',10)
l=legend('leader_{0}','follower_{1}','follower_{2}','follower_{3}','follower_{4}','follower_{5}','follower_{6}');
set(l,'Fontname', 'Times New Roman','FontWeight','bold','FontSize',8)


%% Figure 2: Plotting Velocity Curves
figure          % the velocitiesof mass 1
plot(0:h:time, x0(2,:),'-k','LineWidth',2)
hold on
plot(0:h:time, x1(2,:),'-r','LineWidth',2)
hold on
plot(0:h:time, x2(2,:),'-g','LineWidth',2)
hold on
plot(0:h:time, x3(2,:),'-b','LineWidth',2)
hold on
plot(0:h:time, x4(2,:),'-m','LineWidth',2)
hold on
plot(0:h:time, x5(2,:),'-c','LineWidth',2)
hold on
plot(0:h:time, x6(2,:),'-y','LineWidth',2)
hold on
grid on

xlabel('Time (second)','Fontname', 'Times New Roman','fontsize',10)
ylabel('The velocities of the vehicles','Fontname', 'Times New Roman','fontsize',10)
l=legend('leader_{0}','follower_{1}','follower_{2}','follower_{3}','follower_{4}','follower_{5}','follower_{6}');
set(l,'Fontname', 'Times New Roman','FontWeight','bold','FontSize',8)

%% Figure 3: Plotting Acceleration Curves
figure
plot(0:h:time, x0(3,:),'-k','LineWidth',2)
hold on
plot(0:h:time, x1(3,:),'-r','LineWidth',2)
hold on
plot(0:h:time, x2(3,:),'-g','LineWidth',2)
hold on
plot(0:h:time, x3(3,:),'-b','LineWidth',2)
hold on
plot(0:h:time, x4(3,:),'-m','LineWidth',2)
hold on
plot(0:h:time, x5(3,:),'-c','LineWidth',2)
hold on
plot(0:h:time, x6(3,:),'-y','LineWidth',2)
hold on
grid on

xlabel('Time (second)','Fontname', 'Times New Roman','fontsize',10)
ylabel('The acceleration of the vehicles','Fontname', 'Times New Roman','fontsize',10)
l=legend('leader_{0}','follower_{1}','follower_{2}','follower_{3}','follower_{4}','follower_{5}','follower_{6}');
set(l,'Fontname', 'Times New Roman','FontWeight','bold','FontSize',8)

%% Figure 4: Plotting Locations of each CAV
figure
plot(0,x0(1,1),'>k','linewidth' ,3)
hold on
plot(0,x1(1,1),'or','linewidth' ,3)
hold on 
plot(0,x2(1,1),'+g','linewidth' ,3)
hold on 
plot(0,x3(1,1),'*b','linewidth' ,3)
hold on
plot(0,x4(1,1),'sm','linewidth' ,3)
hold on
plot(0,x5(1,1),'dc','linewidth' ,3)
hold on
plot(0,x6(1,1),'<y','linewidth' ,3)
hold on

plot(1001,x0(1,1001),'>k','linewidth' ,3)
hold on
plot(1001,x1(1,1001),'or','linewidth' ,3)
hold on 
plot(1001,x2(1,1001),'+g','linewidth' ,3)
hold on 
plot(1001,x3(1,1001),'*b','linewidth' ,3)
hold on
plot(1001,x4(1,1001),'sm','linewidth' ,3)
hold on
plot(1001,x5(1,1001),'dc','linewidth' ,3)
hold on
plot(1001,x6(1,1001),'<y','linewidth' ,3)
hold on

plot(2001,x0(1,2001),'>k','linewidth' ,3)
hold on
plot(2001,x1(1,2001),'or','linewidth' ,3)
hold on 
plot(2001,x2(1,2001),'+g','linewidth' ,3)
hold on 
plot(2001,x3(1,2001),'*b','linewidth' ,3)
hold on
plot(2001,x4(1,2001),'sm','linewidth' ,3)
hold on
plot(2001,x5(1,2001),'dc','linewidth' ,3)
hold on
plot(2001,x6(1,2001),'<y','linewidth' ,3)
hold on

plot(3001,x0(1,3001),'>k','linewidth' ,3)
hold on
plot(3001,x1(1,3001),'or','linewidth' ,3)
hold on 
plot(3001,x2(1,3001),'+g','linewidth' ,3)
hold on 
plot(3001,x3(1,3001),'*b','linewidth' ,3)
hold on
plot(3001,x4(1,3001),'sm','linewidth' ,3)
hold on
plot(3001,x5(1,3001),'dc','linewidth' ,3)
hold on
plot(3001,x6(1,3001),'<y','linewidth' ,3)
hold on

plot(4001,x0(1,4001),'>k','linewidth' ,3)
hold on
plot(4001,x1(1,4001),'or','linewidth' ,3)
hold on 
plot(4001,x2(1,4001),'+g','linewidth' ,3)
hold on 
plot(4001,x3(1,4001),'*b','linewidth' ,3)
hold on
plot(4001,x4(1,4001),'sm','linewidth' ,3)
hold on
plot(4001,x5(1,4001),'dc','linewidth' ,3)
hold on
plot(4001,x6(1,4001),'<y','linewidth' ,3)
hold on
grid on

plot(5001,x0(1,5001),'>k','linewidth' ,3)
hold on
plot(5001,x1(1,5001),'or','linewidth' ,3)
hold on 
plot(5001,x2(1,5001),'+g','linewidth' ,3)
hold on 
plot(5001,x3(1,5001),'*b','linewidth' ,3)
hold on
plot(5001,x4(1,5001),'sm','linewidth' ,3)
hold on
plot(5001,x5(1,5001),'dc','linewidth' ,3)
hold on
plot(5001,x6(1,5001),'<y','linewidth' ,3)
hold on

plot(6001,x0(1,6001),'>k','linewidth' ,3)
hold on
plot(6001,x1(1,6001),'or','linewidth' ,3)
hold on 
plot(6001,x2(1,6001),'+g','linewidth' ,3)
hold on 
plot(6001,x3(1,6001),'*b','linewidth' ,3)
hold on
plot(6001,x4(1,6001),'sm','linewidth' ,3)
hold on
plot(6001,x5(1,6001),'dc','linewidth' ,3)
hold on
plot(6001,x6(1,6001),'<y','linewidth' ,3)
hold on

xlabel('Time (second)','Fontname', 'Times New Roman','fontsize',10)
ylabel('Locations of the vehicles','Fontname', 'Times New Roman','fontsize',10)
l=legend('leader_{0}','follower_{1}','follower_{2}','follower_{3}','follower_{4}','follower_{5}','follower_{6}');
set(l,'Fontname', 'Times New Roman','FontWeight','bold','FontSize',8)
%axis tight

%% Plotting Spacing Displacement between Vehicles
figure
plot(0:h:time, x0(1,:)-x0(1,:),'-k','LineWidth',2)
hold on
plot(0:h:time, x0(1,:)-x1(1,:),'-r','LineWidth',2)
hold on
plot(0:h:time, x0(1,:)-x2(1,:),'-g','LineWidth',2)
hold on
plot(0:h:time, x0(1,:)-x3(1,:),'-b','LineWidth',2)
hold on
plot(0:h:time, x0(1,:)-x4(1,:),'-m','LineWidth',2)
hold on
plot(0:h:time, x0(1,:)-x5(1,:),'-c','LineWidth',2)
hold on
plot(0:h:time, x0(1,:)-x6(1,:),'-y','LineWidth',2)
hold on
grid on

xlabel('Time (second)','Fontname', 'Times New Roman','fontsize',10)
ylabel('The spacing distances of the vehicles','Fontname', 'Times New Roman','fontsize',10)
l=legend('leader_{0}','follower_{1}','follower_{2}','follower_{3}','follower_{4}','follower_{5}','follower_{6}');
set(l,'Fontname', 'Times New Roman','FontWeight','bold','FontSize',8)
%axis tight

%% Figure 5: Plotting the Errors
figure
plot(0:h:time,u1,'-r','linewidth' ,1)
hold on
plot(0:h:time,u2,'-g','linewidth' ,1)
hold on
plot(0:h:time,u3,'-b','linewidth' ,1)
hold on
plot(0:h:time,u4,'-m','linewidth' ,1)
hold on
plot(0:h:time,u5,'-c','linewidth' ,1)
hold on
plot(0:h:time,u6,'-y','linewidth' ,1)
hold on
grid on

axis tight

