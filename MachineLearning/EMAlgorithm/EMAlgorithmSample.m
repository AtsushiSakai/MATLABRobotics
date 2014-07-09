% -------------------------------------------------------------------------
%
% File : EMAlgorithmSample.m
%
% Discription : Sample code for estimate probablistic parameters 
%               with EM algorithm.
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------

function EMAlgorithmSample()
close all;
clear all;

%正規分布のパラメータ1
muTrue1=5;
siTrue1=3;
nData1=300;

%正規分布パラメータからサンプルデータを作成
r1 = muTrue1 + siTrue1.*randn(nData1,1);

%正規分布のパラメータ1
muTrue2=10;
siTrue2=1;
nData2=300;

%正規分布パラメータからサンプルデータを作成
r2 = muTrue2 + siTrue2.*randn(nData2,1);

r=[r1;r2];

%EMAlgorithmによる正規分布パラメータの推定
%パラメータ初期値
si1=10;
si2=10;
z=[0.5 0.5];%重み行列

for i=1:10
    
    %モデルの当てはめて期待値の計算 Expectation
    e1=[];
    e2=[];
    for ip=1:length(r)
        p1=Gauss(r(ip),muTrue1,si1);
        p2=Gauss(r(ip),muTrue2,si2);
        e1=[e1;p1/(p1+p2)];
        e2=[e2;p2/(p1+p2)];
    end
    
    %期待値の最大化 Maximization
    sumz=sum(z);
    z(1)=sum(e1)/sumz;
    z(2)=sum(e2)/sumz;
    
    si1=UpdateSigma(e1,r,muTrue1);
    si2=UpdateSigma(e2,r,muTrue2);
    
end

disp('True STD');
[siTrue1 siTrue2]
disp('Estimated STD');
[si1 si2]
disp('Weight Vector');
z

hist(r,20);hold on;
est=[];
nData=nData1+nData2;
for id=-5:0.1:15
    est=[est; [id z*[Gauss(id,muTrue1,si1);Gauss(id,muTrue2,si2)]*nData]];
end
plot(est(:,1),est(:,2),'-r');hold on;
grid on;

function si=UpdateSigma(e,r,mu)
%標準偏差を更新する関数
tmp=0;
for ie=1:length(e)
    tmp=tmp+(e(ie)*(r(ie)-mu)^2);
end
si=sqrt(1/sum(e)*tmp);

function p=Gauss(x,mu,sigma)
%ガウス分布における確率を計算する関数
p=(1/sqrt(2*pi*sigma^2)*exp(-(x-mu)^2/(2*sigma^2)));
