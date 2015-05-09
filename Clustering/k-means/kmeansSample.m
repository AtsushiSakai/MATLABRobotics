% -------------------------------------------------------------------------
%
% File : kmeansSample.m
%
% Discription : Sample code for k-means clustering
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2015 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------

function kmeansSample()
close all;
clear all;

disp('k-means clustering sample start!!');

data=GetRawData();%生データを取得する関数

%k-means法によるクラスタリング
nCluster=2;%クラスタの数
[result,means]=kmeansClustering(data,nCluster);
%クラスタリングの結果を表示
ShowClusteringResult(result,means,nCluster);

function [result,means]=kmeansClustering(data,nCluster)
%k-meansを使ってクラスタリングを実施する方法

%初期クラスタリング ランダムにクラスを振り分ける
result=[data randi(nCluster,[length(data(:,1)),1])];

while 1
    means=[];
    for i=1:nCluster
        %各クラスタの平均値を計算
        means=[means;mean(result(result(:,3)==i,1:2))];
    end
    
    %クラスタリングの結果を表示
    ShowClusteringResult(result,means,nCluster);
    
    %各データを平均値から近い順に再クラスタリング
    nUpdate=0;%クラスタが更新された数
    for j=1:length(result(:,1))
        
        %各データと各平均値までの距離を計算
        d=[];
        for k=1:nCluster
            d=[d norm(result(j,1:2)-means(k,:))];
        end
        [c,i]=min(d);%一番距離の小さいクラスタのIDを計算
        if result(j,3)~=i 
            result(j,3)=i;%クラスタを更新
            nUpdate=nUpdate+1;
        end
    end
    
    if nUpdate==0
        break;%クラスタが変化しなくなったら終了
    end
    
    pause;
end

function ShowClusteringResult(result,means,nCluster)
%クラスタリングの結果をグラフに描画する関数
hold off;

%グラフの色用のデータを作成
cc=hsv(nCluster);

%各クラスタ毎にデータを描画
for i=1:nCluster
    data=result(result(:,3)==i,1:2);
    plot(data(:,1),data(:,2),'.','Color',cc(i,:)); hold all;
    plot(means(i,1),means(i,2),'x','Color',cc(i,:));hold all;
end
axis equal;
grid on;
hold on;
	

function data=GetRawData()
%擬似データのクラスターの中心と誤差量
nSample=[0 0 2;
        10 10 5];
ndata=30;%一つのクラスタに関するデータ点
data=[];

for nc=1:length(nSample(:,1))
    for i=1:ndata
        xy=nSample(nc,1:2)+randn(1,2)*nSample(nc,3);
        data=[data;xy];
    end
end
data=sortrows(data,2);%データをソートして混ぜる
    


