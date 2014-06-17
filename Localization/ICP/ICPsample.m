% -------------------------------------------------------------------------
%
% File : ICPSample.m
%
% Discription : Sample code to estimate relative motion with 
%               Iterative Closest Point (ICP) algorithm.
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function []=ICPsample()

close all;
clear all;

%Simulation Parameters
nPoint=100;%レーザ点の数
fieldLength=5;%点をばら撒く最大距離
motion=[0.5, 0, 10];%真の移動量[並進x[m],並進y[m],回転[deg]]
transitionSigma=0.01;%並進方向の移動誤差標準偏差[m]
thetaSigma=1;   %回転方向の誤差標準偏差[deg]

% 点をランダムでばら撒く(t-1の時の点群)
data1=fieldLength*rand(2,nPoint)-fieldLength/2;

% data2= data1を移動させる & ノイズ付加
% 回転方向 ＆ ノイズ付加
theta=toRadian(motion(3))+toRadian(thetaSigma)*rand(1);
% 並進ベクトル ＆ ノイズ付加
t=repmat(motion(1:2)',1,nPoint)+transitionSigma*randn(2,nPoint);
% 回転行列の作成
A=[cos(theta) sin(theta);-sin(theta) cos(theta)];
% data1を移動させてdata2を作る
data2=t+A*data1;

% ICPアルゴリズム data2とdata1のMatching
% R:回転行列　t:併進ベクトル
% [R,T]=icp(data1,data2)
[R,T] = ICPMatching(data2,data1);

%結果の表示
disp('True Motion [m m deg]:');
motion
disp('Estimated Motion [m m deg]:')
theta = acos(R(1,1))/pi*180;
Est=[T' theta]
disp('Error [m m deg]:')
Error=Est-motion

% --------グラフ---------
figure(1);
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(data1(1,:),data1(2,:),'.b');hold on;
plot(data2(1,:),data2(2,:),'.g');hold on;
z=repmat(T,1,nPoint)+R*data1;
plot(z(1,:),z(2,:),'.r');hold off;

xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Data (t-1)', 'Data (t)','ICP Matching Result');
grid on;
axis([-fieldLength/2 fieldLength/2 -fieldLength/2 fieldLength/2]);



function [R, t]=ICPMatching(data1, data2)
% ICPアルゴリズムによる、並進ベクトルと回転行列の計算を実施する関数
% data1 = [x(t)1 x(t)2 x(t)3 ...]
% data2 = [x(t+1)1 x(t+1)2 x(t+1)3 ...]
% x=[x y z]'

%ICP パラメータ
error=1000;  %各点群のノルム誤差の総和
preError=0;%一つ前のイタレーションのerror値
dError=1000;%エラー値の差分
EPS=0.0001;%収束判定値
maxIter=100;%最大イタレーション数
count=0;%ループカウンタ

R=eye(2);%回転行列
t=zeros(2,1);%並進ベクトル

while ~(dError < EPS)
	count=count+1;
    
    [ii, error]=FindNearestPoint(data1, data2);%最近傍点探索
    [R1, t1]=SVDMotionEstimation(data1, data2, ii);%特異値分解による移動量推定
    data2=R1*data2;
    data2=[data2(1,:)+t1(1) ; data2(2,:)+t1(2)];
    R = R1*R;
    t = R1*t + t1; 
    dError=abs(preError-error);%エラーの改善量
    preError=error;%一つ前のエラーの総和値を保存
    
    if count > maxIter %収束しなかった
        disp('Max Iteration');return;
    end
end
disp(['Convergence:',num2str(count)]);

function [index, error]=FindNearestPoint(data1, data2)
%data2に対するdata1の最近傍点のインデックスを計算する関数
m1=size(data1,2);
m2=size(data2,2);
index=[];
error=0;

for i=1:m1
    dx=(data2-repmat(data1(:,i),1,m2));
    dist=sqrt(dx(1,:).^2+dx(2,:).^2);
    [dist, ii]=min(dist);
    index=[index; ii];
    error=error+dist;
end

function [R, t1]=SVDMotionEstimation(data1, data2, index)
%特異値分解法による並進ベクトルと、回転行列の計算

n = length(index);

%各点群の重心の計算
M = data1; 
mm = mean(M,2);
S = data2(:,index);
ms = mean(S,2); 

%各点群を重心中心の座標系に変換
Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2);];
Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2);];

W = Sshifted*Mshifted';
[U A V] = svd(W);%特異値分解

R = (U*V')';%回転行列の計算

if det(R)<0
    B = eye(2)
    B(2,2) = det(V*U');
    R = V*B*U';
end

t1 = mm - R*ms;%並進ベクトルの計算

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;






