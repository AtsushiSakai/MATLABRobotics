% -------------------------------------------------------------------------
%
% File : NelderMead.m
%
% Discription : Non-Linear optimizion with Nelder-Mead Method
%
% Reference: http://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method
% 
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [] = NelderMead()
close all;
clear all;

disp('NelderMead Simulation start!!')
%シミュレーション空間の最大最小範囲
maxxy=5;
minxy=-5;

%メッシュデータの生成
[x1range,x2range,yrange]=CreateMeshData(minxy,maxxy);

%初期パラメータの生成[x1 x2 y]
param=InitSearchParameter(minxy,maxxy);

iterationMax=50;%イタレーションの最大回数

%Parameter
row=1;%Reflection Parameter
kai=2;%Expansion Parameter
gamma=0.5;%Contraction Paremter 
sigma=0.5;%Shrink Parameter

for i=1:iterationMax
    %xmax change flag
    cf=0;
    
    %(1) Sort
    param=sortrows(param,3);
    
    %(2) Reflection
    xavg=mean(param(1:2,1:2));
    xr=xavg+row*(xavg-param(3,1:2));
    xr=[xr f(xr(1),xr(2))];
    
    %(3) Expand
    if xr(3)<param(1,3)
        xe=kai*xr(1:2)+(1-kai)*xavg;
        xe=[xe f(xe(1),xe(2))];
        if xe(3)<param(1,3)
            param(3,:)=xe;
            cf=1;
        elseif param(1,3)<xe(3)
            param(3,:)=xr;
            cf=1;
        end
    end
    %(4) contract
    if param(2,3)<xr(3)
        %(4-1) contract outside
        if param(2,3)<xr(3)&& xr(3)<param(3,3)
            xc=gamma*xr(1:2)+(1-gamma)*xavg;
            xc=[xc f(xc(1),xc(2))];
            if xc(3)<xr(3)
                param(3,:)=xc;
                cf=1;
            end
            %(4-2) contract inside
        elseif param(3,3)<xr(3)
            xcc=param(3,1:2)+(1-gamma)*xavg;
            xcc=[xcc f(xcc(1),xcc(2))];
            if xcc(3)<xr(3)
                param(3,:)=xcc;
                cf=1;
            end
        end
    end
    
    %(5) shrink
    if cf==0
        param(2,:)=sigma*(param(2,:)+param(1,:));
        param(3,:)=sigma*(param(3,:)+param(1,:));
    end
    
    %収束判定
    norm12=norm(param(1,:)-param(2,:));
    norm23=norm(param(2,:)-param(3,:));
    norm31=norm(param(3,:)-param(1,:));
    normSum=norm12+norm23+norm31;
    if normSum<0.001
        disp('Convergence!! final param [x1, x2, y] is');
        mean(param,1)
        break;
    end
    
    %シミュレーション結果の描画
    figure(1)
    hold off;
    contour(x1range,x2range,yrange,100); hold on;
    ag=[param;param];
    plot3(ag(:,1),ag(:,2),ag(:,3),'-k','markersize',10);
    plot3(ag(:,1),ag(:,2),ag(:,3),'.k','markersize',10);
    xlabel('x1');
    ylabel('x2');
    view(2)
    drawnow;
    pause(0.5);
     
end


function param=InitSearchParameter(minxy,maxxy)
%初期パラメータを作成する関数

x1=maxxy - (maxxy-minxy).*rand(1,3);
x2=maxxy - (maxxy-minxy).*rand(1,3);
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
