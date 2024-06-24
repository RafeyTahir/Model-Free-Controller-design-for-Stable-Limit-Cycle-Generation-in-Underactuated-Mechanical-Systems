%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%---Leg-foot model System - Quasi-Static Equilibrium----%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

%%%% Physical Parameters
m_l= 10;  %mass of the leg
m_f=2;  %mass of the foot
l=5;  %5  %length 1.2
I_f=(1/12) * m_f * l^2 ;  %Inertia of the Foot
I_l=(1/12) * m_l * l^2 ;  %Inertia of the Leg
k=5.0000e+3;  %5.0000e+3
delta=0.5;%0.7
g=9.8; 


theta=[0];        %   Actuated Joint angle
alpha=[-0.0785];  
y=[0];
theta_2dot_o=[0];
theta_2_o=[0];
% theta_1=[0.4];        %   Actuated Joint angle
% theta_2=[-0.314];        %   Unactuated Joint angle
theta_dot=[0];
theta_ddot=[0];
alpha_dot=[0];
alpha_ddot=[0];
y_dot=[0];
y_ddot=[0];
e2o=[0];
e2do=[0];
t1=6;
t2=4;

theta_1dots=zeros(1,30000);
theta_1s=zeros(1,30000);

extended_trajectory = [];
amplitude_factor = [-0.0872, 0.0872];
amplitude_factor1 = [0.0872, -0.0872];
aaa=0.174;
normalized_signal=[0];
p_min=[0];
p_max=[0];

kd=10; %5 40       50       % Controller parameter
kp=100; %19 45   45
kp_1=2; %2
kd_1=6; %6
b1=2;
a=50;
%dist=0.2;
% distt=-4;

Phi_hat=[0]; Phi_hatdot=[0]; Phidk_2=[0];
Xd=[0]; YY=[0]; Xk_1=[0]; XX=[0]; U1=[0];

Phi_hatk_1=[0]; Phik_1=[0]; Phik=Phik_1; Phi_hatk=Phi_hatk_1;
Phi_hatd=[0];

p=[0];
Ts=0.002;
Ts_2=0.002;
Ti=0;
Tf=20;
t = Ti:0.002:Tf-0.002;       %%%% Time vector
amp=0;
ref_2 = amp*ones(size(t));
ref_2dot=diff(ref_2)/(Ts_2);
ref_2ddot=diff(ref_2dot)/(Ts_2);

if length(ref_2dot(1,:))< length(ref_2)
    ref_2dot(1,length(ref_2dot(1,:)):length(ref_2))=ref_2dot(1,length(ref_2dot(1,:)));
end

if length(ref_2ddot(1,:))< length(ref_2)
    ref_2ddot(1,length(ref_2ddot(1,:)):length(ref_2))=ref_2ddot(1,length(ref_2ddot(1,:)));
end


traj=zeros(1,1000);
for b=Ti+1:Tf
    %p1=p(:,b);
    [ref1]=polynomial12(b,U1);
    ref2=ref1*aaa;
repeated_trajectory = flip(ref2);
    ref = [ref2, repeated_trajectory];
    %ref=ref*aaa(mod(b,2)+1);

    ref_dot=diff(ref)/Ts;
    ref_ddot=diff(ref_dot)/Ts;

    if length(ref_dot(1,:))< length(ref)
        ref_dot(1,length(ref_dot(1,:)):length(ref))=ref_dot(1,length(ref_dot(1,:)));
    end

    if length(ref_ddot(1,:))< length(ref)
        ref_ddot(1,length(ref_ddot(1,:)):length(ref))=ref_ddot(1,length(ref_ddot(1,:)));
    end
    for i=Ti+1:2000

        e_2=alpha-ref(1,i);
        ed_2=alpha_dot-ref_dot(1,i);
        edd_2=alpha_ddot-ref_ddot(1,i);
        thetaddot_3=ref_ddot(1,i);

        a1=m_f+m_l;
a2=-l*m_l*sin(theta+alpha);
a3=m_l*l^2+I_f+I_l;
a4=m_l*l^2 +I_l;
a5=-l*m_l*cos(theta + alpha)*(theta_dot + 2*alpha_dot);
a6=-l*m_l*cos(theta+alpha)*alpha_dot;
a7=0;
a8=0;
a9=-g*l*m_l*sin(theta+alpha);
Phi= alpha_ddot + (a2^2*a8)./(a4*(a4*a1 -a2^2)) - (a2^2*a4)./(a4*(a1*a4 - a2^2)) - a9/a4 + a8/a4;


        Phik1=Phi;                                          % k1=k+1
        Phidd=(Phik1-Phik)/(0.002);
        Phik_1=Phik;
        Phik=Phik1;

        Phidk_1=a*Xk_1+Phi_hatd+b1*ed_2;
        Phid=Phidk_1+Phidk_1-Phidk_2;
        Phidk_2=Phidk_1;
%         
        % XX=edd_2+kd*ed_2+kp*e_2;
        XX=edd_2+kd*ed_2+kp*e_2;
        Xk_1=XX;

        Phi_hatd=-a*XX-b1*ed_2+Phidk_1;
        Phi_hat= 0.002*Phi_hatd+Phi_hat;


        U=(Phi_hat+thetaddot_3 - kd*ed_2 -kp*e_2); %(-inv(n1)*n2)

        % U3=(Phi_hat+thetaddot_2 - kd*ed_2 -kp*e_2)*1/(-m21*inv(m11));

        Phi_hatstore(:,(b-1)*2000 + i)=Phi_hat+theta_ddot;
        Phi_store(:,(b-1)*2000 + i)=Phi+theta_ddot;
        U=U ; %(-inv(n1)*n2)


        XXstore(:,i)=XX;
  alpha_ddot= U/a4  -  (a2^2*a8)./(a4*(a4*a1 -a2^2)) + (a2^2*a4)./(a4*(a1*a4 - a2^2)) + a9/a4 + a8/a4;
       
        
        theta_ddot= -a8/a4 -alpha_ddot + (a2*a7)./(a4*a1 -a2^2) + (a2*a6*alpha_dot)./(a4*a1 -a2^2) + (a2*a5*theta_dot)./(a4*a1 -a2^2) + (a2^2*a8)./(a4*(a4*a1 - a2^2));


        y_ddot= ((-a7/a1 - (a6*alpha_dot)./a1 - (a5*theta_dot)./a1 + (a2*a8)./(a1*a4)) * (a4*a1) )./(a4*a1 -a2^2);     
       
        alpha_store(:,(b-1)*2000 + i)=alpha;
        alphad_store(:,(b-1)*2000 + i)=alpha_dot;
        alphadd_store(:,(b-1)*2000 + i)=alpha_ddot;
        U_store(:,(b-1)*2000 + i)=U;
        traj(:,(b-1)*2000 + i)=ref(:,i);
        trajdot(:,(b-1)*2000 + i)=ref_dot(:,i);
        Norm_store(:,(b-1)*2000 + i)=p;
        error(:,(b-1)*2000 + i)=e_2;


       [theta, theta_dot, theta_ddot] = RK4_integration(theta, theta_dot, theta_ddot, Ts);
        %theta_1ddot= (U - (0.5*m2*l2*cos(theta_2))*theta_3ddot - ((0.5*m1 + m2)*l2*cos(theta_1))*theta_2ddot + 0.5*m1*m2*l1*sin(theta_1)*theta_1dot*theta_2dot + 0.5*m2*l2*sin(theta_2)*theta_2dot*theta_3dot)./(m0+m1+m2);
       [alpha, alpha_dot, alpha_ddot] = RK4_integration(alpha, alpha_ddot, alpha_ddot, Ts);
       [y, y_dot, y_ddot] = RK4_integration(y, y_dot, y_ddot, Ts);

         theta_store(:,(b-1)*2000 + i)=theta;
         thetad_store(:,(b-1)*2000 + i)=theta_dot;
        thetadd_store(:,(b-1)*2000 + i)=theta_ddot;

    end

p=0.5;
U1=p;

     end
 

%save('data.mat','theta_1s','theta_1dots','theta2_store','theta2d_store')
difference = traj - alpha_store;

% Calculate the square of the difference
squared_difference = difference .^ 2;

% Calculate the mean of the squared difference
mean_squared_difference = mean(squared_difference);

% Calculate the RMSE
rmse = sqrt(mean_squared_difference);
fprintf('The RMSE between the Desired and Estimated is: %f\n', rmse);
% Display the RMSE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%-----Ploting------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% subplot(2,2,1)
% %figure(1)
% plot(traj,'--r','LineWidth',1)
% hold on
% plot(theta3_store,'b','LineWidth',1)
% title(['Tracked output Position of Pendulum']);
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000 35000 40000 45000 50000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30 35 40 45 50],'FontSize',12)
% legend({'$\theta^{*}_{na}$','$\hat{\theta}_{na}$'},'interpreter','latex')
% ylabel('$\theta_{na}(rad)$','interpreter','latex')
% xlabel('$time(s)$','interpreter','latex')
% %xlim(gca(),[0,20000])
% % % print('q_{na}&hat{q}_{na}.eps','-depsc')
% 
% figure(1)
% subplot(2,2,2)
% %figure(2)
% plot(theta1_store,'b','LineWidth',1)
% hold on
% title(['Cart Position']);
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30])
% % %ylim(gca(),[-1,0.5])
% xlabel('$time(s)$','interpreter','latex')
% % % print('q_{a}.eps','-depsc')
% % %
% 
% % subplot(2,2,3)
% % plot(trajdot,'--r')
% % hold on
% % plot(theta2d_store,'b')
% % title(['Pendulum Velocity']);
% % set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000 35000 40000 45000 50000] );
% % set(gca,'XTickLabel',[0 5 10 15 20 25 30 35 40 45 50])
% 
% subplot(2,2,4)
% plot(theta1d_store)
% title(['Cart Velocity']);
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30])
% 
% figure(2)
% subplot(2,3,1)
% plot(Norm_store)
% % % hold on
% % % plot(U_store1)
% title(['2nd Controller input b']);
% % ylim(gca(),[-1,1])
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000 35000 40000 45000 50000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30 35 40 45 50])
% xlabel('$time(s)$','interpreter','latex')
% % % print('p.eps','-depsc')
% % %
% 
% subplot(2,3,4)
% plot(U_store)
% % % hold on
% % % plot(U_store1)
% title(['1st Controller input u']);
% %ylim(gca(),[-1,1])
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000 35000 40000 45000 50000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30 35 40 45 50])
% xlabel('$time(s)$','interpreter','latex')
%ylim(gca(),[-0.1,0.1])
% % print('u.eps','-depsc')
%

% subplot(2,3,2)
% plot(theta2_store,theta2d_store)
% title(['Phase portraits Pendulum']);
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000 35000 40000 45000 50000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30 35 40 45 50])
% grid
% 
% subplot(2,3,3)
% plot(theta1_store,theta1d_store)
% title(['Phase portraits Cart']);
% set(gca,'XTick',[0 5000 10000 15000 20000 25000 30000 35000 40000 45000 50000] );
% set(gca,'XTickLabel',[0 5 10 15 20 25 30 35 40 45 50])
% grid