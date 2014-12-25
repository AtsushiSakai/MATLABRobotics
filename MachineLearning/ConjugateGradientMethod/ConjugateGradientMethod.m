% -------------------------------------------------------------------------
%
% File : ConjugateGradientMethod.m
%
% Discription : Non-Linear optimizion with Conjugate Gradient Method
% 
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [] = ConjugateGradientMethod()
close all;
clear all;

disp('Optimization simulation with conjugate gradient method starts!!')
%シミュレーション空間の最大最小範囲
maxxy=5;
minxy=-5;

%メッシュデータの生成
[x1range,x2range,yrange]=CreateMeshData(minxy,maxxy);

%初期パラメータの生成[x1 x2 y]
param=InitSearchParameter(minxy,maxxy);

finalParam=ConjugateGradientMethodOptimization(param,x1range,x2range,yrange);
disp('Final Param is ');
finalParam

function finalParam=ConjugateGradientMethodOptimization(param,x1range,x2range,yrange)
%最急降下法による最適化

iterationMax=50;%イタレーションの最大回数
result=param;%パラメータの履歴

%共役勾配法用パラメータ初期化
d=-CalcJacobi(param(1),param(2));%勾配ベクトルの初期化
preJ=CalcJacobi(param(1),param(2));

for i=1:iterationMax
    
    %ヤコビ行列の取得
    J=CalcJacobi(param(1),param(2));
    
    %勾配のベクトルの更新
    beta=max(0,((J-preJ)*J')/(preJ*preJ'));
    d=-J+beta*d;
    
    %学習率の線形探索
    alpha=GoldenSection(0,0.5,param,d);
    
    % 共役勾配の計算
    param(1:2)=param(1:2)+alpha*d;
    param(3)=f(param(1),param(2));%評価関数の計算
    preJ=J;%一つ前のヤコビ行列をストア
    
    %シミュレーション結果の描画
    figure(1)
    hold off;
    contour(x1range,x2range,yrange,100); hold on;
    result=[result;param];
    plot3(result(:,1),result(:,2),result(:,3),'.-k','markersize',10);
    xlabel('x1');
    ylabel('x2');
    view(2)
    drawnow;
    pause(0.5);
    
    %収束判定
    if sum(abs(alpha*J))<=0.01
        disp(['Converge:',i]);
        break;
    end
end

finalParam=param(end,:);

function [minX]=GoldenSection(a,b,param,d)
%黄金分割法による線形探索関数

%黄金率
GOLDEN_RATIO = 1.6180339887498948482045868343656;
 
%内分点の計算
x1 = (a-b)/(GOLDEN_RATIO + 1.0) + b;
x2 = (a-b)/GOLDEN_RATIO + b;
%評価関数を両方計算するのは最初だけ
paramTmp=param(1:2)+x1*d;
f1=f(paramTmp(1),paramTmp(2));
paramTmp=param(1:2)+x2*d;
f2=f(paramTmp(1),paramTmp(2));
while 1
    %ループを回して両点を更新
    if f1 < f2
        a = x2;
        x2 = x1;
        f2 = f1;
        x1 = (a - b)/(GOLDEN_RATIO + 1.0) + b;
        paramTmp=param(1:2)+x1*d;
        f1=f(paramTmp(1),paramTmp(2));
    else
        b = x1;
        x1 = x2;
        f1 = f2;
        x2 = (a - b)/GOLDEN_RATIO + b;
        paramTmp=param(1:2)+x2*d;
        f2=f(paramTmp(1),paramTmp(2));
    end
    %収束判定
    if abs(a-b)<=10^-3
        minX=(a+b)/2;
        break
    end
end

function param=InitSearchParameter(minxy,maxxy)
%初期パラメータを作成する関数

x1=maxxy - (maxxy-minxy).*rand(1,1);
x2=maxxy - (maxxy-minxy).*rand(1,1);
y=f(x1,x2);
param=[x1' x2' y'];

function [x1range,x2range,yrange]=CreateMeshData(minxy,maxxy)
%シミュレーション空間のメッシュデータを作成する関数

%メッシュデータの作成
[x1range,x2range]=meshgrid(minxy:0.3:maxxy);
yrange=f(x1range,x2range);

function y = f(x1,x2)
% Himmelblau's function
% see Himmelblau's function - Wikipedia, the free encyclopedia 
% http://en.wikipedia.org/wiki/Himmelblau%27s_function
y=(x1.^2+x2-11).^2+(x1+x2.^2-7).^2;

function J= CalcJacobi(x,y)
% jacobi matrix of Himmelblau's function
dx=4*x^3+4*x*y-44*x+2*x+2*y^2-14;
dy=2*x^2+4*x*y+4*y^3-26*y-22;
J=[dx dy];