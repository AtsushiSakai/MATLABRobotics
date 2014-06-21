% -------------------------------------------------------------------------
%
% File : ExtendedKalmanFilterLocalization.m
%
% Discription : Mobible robot localization sample code with
% Extended Kalman Filter (EKF)
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
 
 
function [] = ExtendedKalmanFilterLocalization()
 
close all;
clear all;
 
disp('Extended Kalman Filter (EKF) sample program start!!')
 
time = 0;
endtime = 60; % [sec]
global dt;
dt = 0.1; % [sec]
 
nSteps = ceil((endtime - time)/dt);
 
result.time=[];
result.xTrue=[];
result.xd=[];
result.xEst=[];
result.z=[];
result.PEst=[];
result.u=[];
 
 
% State Vector [x y yaw v]'
xEst=[0 0 0 0]';
 
% True State
xTrue=xEst;
 
% Dead Reckoning
xd=xTrue;
 
% Observation vector [x y yaw v]'
z=[0 0 0 0]';
 
% Covariance Matrix for motion
Q=diag([0.1 0.1 toRadian(1) 0.05]).^2;
 
% Covariance Matrix for observation
R=diag([1.5 1.5 toRadian(3) 0.05]).^2;
 
% Simulation parameter
global Qsigma
Qsigma=diag([0.1 toRadian(20)]).^2; %[v yawrate]
 
global Rsigma
Rsigma=diag([1.5 1.5 toRadian(3) 0.05]).^2;%[x y z yaw v]

PEst = eye(4);
 
tic;

% Main loop
for i=1 : nSteps
    time = time + dt;
    % Input
    u=doControl(time);
    % Observation
    [z,xTrue,xd,u]=Observation(xTrue, xd, u);
    
    % ------ Kalman Filter --------
    % Predict
    F=jacobF(xEst, u);
    xPred = f(xEst, u);
    PPred= F*PEst*F' + Q;
    
    % Update
    H=jacobH(xPred);
    y = z - h(xPred);
    S = H*PPred*H' + R;
    K = PPred*H'*inv(S);
    xEst = xPred + K*y;
    PEst = (eye(size(xEst,1)) - K*H)*PPred;
    
    %Animation (remove some flames)
    if rem(i,5)==0
        %hold off;
        plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
        plot(result.z(:,1),result.z(:,2),'.g');hold on;
        plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
        plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
        ShowErrorEllipse(xEst,PEst);
        axis equal;
        grid on;
        drawnow;
    end
    
    % Simulation Result
    result.time=[result.time; time];
    result.xTrue=[result.xTrue; xTrue'];
    result.xd=[result.xd; xd'];
    result.xEst=[result.xEst;xEst'];
    result.z=[result.z; z'];
    result.PEst=[result.PEst; diag(PEst)'];
    result.u=[result.u; u'];
end
toc
 
DrawGraph(result);

function ShowErrorEllipse(xEst,PEst)
%誤差分散円を計算し、表示する関数
Pxy=PEst(1:2,1:2);%x,yの共分散を取得
[eigvec, eigval]=eig(Pxy);%固有値と固有ベクトルの計算
%固有値の大きい方のインデックスを探す
if eigval(1,1)>=eigval(2,2)
    bigind=1;
    smallind=2;
else
    bigind=2;
    smallind=1;
end

chi=9.21;%誤差楕円のカイの二乗分布値　99%

%楕円描写
t=0:10:360;
a=sqrt(eigval(bigind,bigind)*chi);
b=sqrt(eigval(smallind,smallind)*chi);
x=[a*cosd(t);
   b*sind(t)];
%誤差楕円の角度を計算
angle = atan2(eigvec(bigind,2),eigvec(bigind,1));
if(angle < 0)
    angle = angle + 2*pi;
end

%誤差楕円の回転
R=[cos(angle) sin(angle);
   -sin(angle) cos(angle)];
x=R*x;
plot(x(1,:)+xEst(1),x(2,:)+xEst(2))


function x = f(x, u)
% Motion Model

global dt;
 
 
F = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 0];
 
 
B = [
    dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0];
 
 
x= F*x+B*u;
 
function jF = jacobF(x, u)
% Jacobian of Motion Model

global dt;
 
jF=[
    1 0 0 0
    0 1 0 0
    -dt*u(1)*sin(x(3)) dt*cos(x(3)) 1 0
    dt*u(1)*cos(x(3)) dt*sin(x(3)) 0 1];

function z = h(x)
%Observation Model

H = [1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1 ];
 
z=H*x;

function jH = jacobH(x)
%Jacobian of Observation Model

jH =[1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1];

function u = doControl(time)
%Calc Input Parameter

T=10; % [sec]
 
% [V yawrate]
V=1.0; % [m/s]
yawrate = 5; % [deg/s]
 
u =[ V*(1-exp(-time/T)) toRadian(yawrate)*(1-exp(-time/T))]';
 

function [z, x, xd, u] = Observation(x, xd, u)
%Calc Observation from noise prameter
global Qsigma;
global Rsigma;

x=f(x, u);% Ground Truth
u=u+Qsigma*randn(2,1);%add Process Noise
xd=f(xd, u);% Dead Reckoning
z=h(x+Rsigma*randn(4,1));%Simulate Observation


function []=DrawGraph(result)
%Plot Result

figure(1);
x=[ result.xTrue(:,1:2) result.xEst(:,1:2) result.z(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,5), x(:,6),'.g','linewidth', 4); hold on;
plot(x(:,1), x(:,2),'-.b','linewidth', 4); hold on;
plot(x(:,3), x(:,4),'r','linewidth', 4); hold on;
plot(result.xd(:,1), result.xd(:,2),'--k','linewidth', 4); hold on;
 
title('EKF Localization Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','GPS','Dead Reckoning','EKF','Error Ellipse');
grid on;
axis equal;

function angle=Pi2Pi(angle)
angle = mod(angle, 2*pi);

i = find(angle>pi);
angle(i) = angle(i) - 2*pi;

i = find(angle<-pi);
angle(i) = angle(i) + 2*pi;


function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;