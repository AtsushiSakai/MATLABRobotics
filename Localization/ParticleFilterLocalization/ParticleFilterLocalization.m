% -------------------------------------------------------------------------
%
% File : ParticleFilterLocalization.m
%
% Discription : Mobible robot localization sample code with
%               ParticleFilterLocalization (PF)
%               The Robot can get a range data from RFID that its position
%               known.
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------
 
function [] = ParticleFilterLocalization()
 
close all;
clear all;
 
disp('Particle Filter (PF) sample program start!!')
 
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

% State Vector [x y yaw]'
xEst=[0 0 0]';
 
% True State
xTrue=xEst;
 
% Dead Reckoning Result
xd=xTrue;
 
% Covariance Matrix for predict
Q=diag([0.2 0.2 toRadian(10)]).^2;
 
% Covariance Matrix for observation
R=diag([1]).^2;%[range [m]

% Simulation parameter
global Qsigma
Qsigma=diag([0.1 toRadian(20)]).^2;
 
global Rsigma
Rsigma=diag([0.1]).^2;

%RFIFタグの位置 [x, y]
RFID=[10 0;
      10 10;
      0  15
      -5 20];
  
MAX_RANGE=20;%最大観測距離
NP=100;%パーティクル数
NTh=NP/2.0;%リサンプリングを実施する有効パーティクル数

px=repmat(xEst,1,NP);%パーティクル格納変数
pw=zeros(1,NP)+1/NP;%重み変数
 
tic;
%movcount=0;
% Main loop
for i=1 : nSteps
    time = time + dt;
    % Input
    u=doControl(time);
    % Observation
    [z,xTrue,xd,u]=Observation(xTrue, xd, u, RFID, MAX_RANGE);
    
    % ------ Particle Filter --------
    for ip=1:NP
        x=px(:,ip);
        w=pw(ip);
        
        % Dead Reckoning and random sampling
        x=f(x, u)+Q*randn(3,1);
    
        % Calc Inportance Weight
        for iz=1:length(z(:,1))
            pz=norm(x(1:2)'-z(iz,2:3));
            dz=pz-z(iz,1);
            w=w*Gauss(dz,0,R);
        end
        px(:,ip)=x;%格納
        pw(ip)=w;
    end
    
    pw=Normalize(pw,NP);%正規化
    [px,pw]=Resampling(px,pw,NTh,NP);%リサンプリング
    xEst=px*pw';%最終推定値は期待値
    
    % Simulation Result
    result.time=[result.time; time];
    result.xTrue=[result.xTrue; xTrue'];
    result.xd=[result.xd; xd'];
    result.xEst=[result.xEst;xEst'];
    result.u=[result.u; u'];
    
    %Animation (remove some flames)
    if rem(i,5)==0 
        hold off;
        arrow=0.5;
        %パーティクル表示
        for ip=1:NP
            quiver(px(1,ip),px(2,ip),arrow*cos(px(3,ip)),arrow*sin(px(3,ip)),'ok');hold on;
        end
        plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
        plot(RFID(:,1),RFID(:,2),'pk','MarkerSize',10);hold on;
        %観測線の表示
        if~isempty(z)
            for iz=1:length(z(:,1))
                ray=[xTrue(1:2)';z(iz,2:3)];
                plot(ray(:,1),ray(:,2),'-r');hold on;
            end
        end
        plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
        plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
        axis equal;
        grid on;
        drawnow;
        %動画を保存する場合
        %movcount=movcount+1;
        %mov(movcount) = getframe(gcf);% アニメーションのフレームをゲットする
    end
   
end
toc

%アニメーション保存
%movie2avi(mov,'movie.avi');

DrawGraph(result);

function [px,pw]=Resampling(px,pw,NTh,NP)
%リサンプリングを実施する関数
%アルゴリズムはLow Variance Sampling
Neff=1.0/(pw*pw');
if Neff<NTh %リサンプリング
    wcum=cumsum(pw);
    base=cumsum(pw*0+1/NP)-1/NP;%乱数を加える前のbase
    resampleID=base+rand/NP;%ルーレットを乱数分増やす
    ppx=px;%データ格納用
    ind=1;%新しいID
    for ip=1:NP
        while(resampleID(ip)>wcum(ind))
            ind=ind+1;
        end
        px(:,ip)=ppx(:,ind);%LVSで選ばれたパーティクルに置き換え
        pw(ip)=1/NP;%尤度は初期化
    end
end

function pw=Normalize(pw,NP)
%重みベクトルを正規化する関数
sumw=sum(pw);
if sumw~=0
    pw=pw/sum(pw);%正規化
else
    pw=zeros(1,NP)+1/NP;
end
    

function p=Gauss(x,u,sigma)
%ガウス分布の確率密度を計算する関数
p=1/sqrt(2*pi*sigma^2)*exp(-(x-u)^2/(2*sigma^2));

function x = f(x, u)
% Motion Model
 
global dt;
 
F = [1 0 0
    0 1 0
    0 0 1];
 
B = [
    dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt];
  
x= F*x+B*u;

function u = doControl(time)
%Calc Input Parameter
T=10; % [sec]
 
% [V yawrate]
V=1.0; % [m/s]
yawrate = 5; % [deg/s]
 
u =[ V*(1-exp(-time/T)) toRadian(yawrate)*(1-exp(-time/T))]';

%Calc Observation from noise prameter
function [z, x, xd, u] = Observation(x, xd, u, RFID,MAX_RANGE)
global Qsigma;
global Rsigma;
 
x=f(x, u);% Ground Truth
u=u+Qsigma*randn(2,1);%add Process Noise
xd=f(xd, u);% Dead Reckoning
%Simulate Observation
z=[];
for iz=1:length(RFID(:,1))
    d=norm(RFID(iz,:)-x(1:2)');
    if d<MAX_RANGE %観測範囲内
        z=[z;[d+Rsigma*randn(1,1) RFID(iz,:)]];
    end
end

function []=DrawGraph(result)
%Plot Result
 
figure(1);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-.b','linewidth', 4); hold on;
plot(x(:,3), x(:,4),'r','linewidth', 4); hold on;
plot(result.xd(:,1), result.xd(:,2),'--k','linewidth', 4); hold on;
 
title('PF Localization Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','PF','Dead Reckoning');
grid on;
axis equal;
 
function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;