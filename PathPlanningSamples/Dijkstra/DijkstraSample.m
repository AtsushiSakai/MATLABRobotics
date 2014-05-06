function [] = DijkstraSample()
%DijkstraSample() ダイクストラ法による最短経路探索プログラム
%
% Author: Atsushi Sakai
%
% Copyright (c) 2014, Atsushi Sakai
% All rights reserved.
% License : Modified BSD Software License Agreement

clear all;
close all;
disp('Dijkstra Path Planning start!!');

%パラメータ
p.start=[1,1];  %スタート地点
p.goal=[10,3];  %ゴール地点
p.XYMAX=11;     %マップの最大縦横

%境界データの取得
obstacle=GetBoundary(p);

%障害物データの取得 境界データと合わせて取得する
nObstacle=20;%障害物の数
obstacle=GetObstacle(nObstacle,obstacle,p);

%最短経路を生成
path=Dijkstra(obstacle,p);

%グラフ作成
figure(1)
if length(obstacle)>=1
    plot(obstacle(:,1),obstacle(:,2),'om');hold on;
end
plot(p.start(1),p.start(2),'*r');hold on;
plot(p.goal(1),p.goal(2),'*b');hold on;
if length(path)>=1
    plot(path(:,1),path(:,2),'-r');hold on;
end
axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
grid on;

%DPの計算をする
%DPWithDijkstra(obstacle,p)

end

function DPWithDijkstra(obstacle,p)
%ダイクストラ法によって全ノードにおける最適移動方向を計算する
%(DynamicPrograming)関数

%ゴールとスタートを交換して計算
tmp=p.start;
p.start=p.goal;
p.goal =tmp;

%計算中ノード情報格納用[x,y,cost,px(親ノード),py(親ノード)] startノードを格納する
open=[p.start(1) p.start(2) 0 p.start(1) p.start(2)];
close=[];%計算済みノード情報格納用

%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
next=MotionModel();

findFlag=false;%ゴール発見フラグ

while ~findFlag
    %すべてのopenなノードに対して隣接ノードのコストを計算する
    initNopen=length(open(:,1));
    %openにデータがない場合はパスが見つからなかった。
    if initNopen==0 disp('All Node Search!!'); break; end
    for io=1:initNopen
        
        for in=1:length(next(:,1))
            %隣接ノードの位置とコストの計算
            m=[open(io,1)+next(in,1) open(io,2)+next(in,2) open(io,3)+next(in,3)];
            
            %隣接ノードが障害物だったら次のノードを探す
            if isObstacle(m,obstacle) continue; end
            
            %openとcloseのリストの中にmが含まれるかを探索
            [flag, targetInd]=FindList(m,open,close);
            
            %openリストにある場合
            if flag==1
                if m(3)<open(targetInd,3)
                    open(targetInd,3)=m(3);
                    open(targetInd,4)=open(io,1);
                    open(targetInd,5)=open(io,2);
                end
            elseif flag==2 %closeリストにある場合
                if m(3)<close(targetInd,3)
                    %親ノードの更新
                    close(targetInd,4)=open(io,1);
                    close(targetInd,5)=open(io,2);
                    open=[open; close(targetInd,:)];
                    close(targetInd,:)=[];%Openリストに移動
                end
            else %どちらにも無い場合
                %openリストに親ノードのインデックスと共に追加
                open=[open;[m open(io,1) open(io,2)]];
            end
        end
    end
    %隣接ノード計算したopenノードはcloseノードへ移動
    if findFlag==false
        close=[close; open(1:initNopen,:)];
        open(1:initNopen,:)=[];
    end
    
    %パス探索のステップ動画
    %animation(open,close,p,obstacle);

end

%DPの結果を矢印で表示する
figure(2)
if length(obstacle)>=1
    plot(obstacle(:,1),obstacle(:,2),'om');hold on;
end
plot(p.start(1),p.start(2),'*r');hold on;
plot(p.goal(1),p.goal(2),'*b');hold on;

for in=1:length(close(:,1))
    if isSamePosi(close(in,:),p.start)
        continue;
    elseif isSamePosi(close(in,:),p.goal)
        continue
    else
        policy=close(in,4:5)-close(in,1:2);
        if isSamePosi(policy,[1 0])
            quiver(close(in,1),close(in,2),0.5,0,'ok');hold on;
        elseif isSamePosi(policy,[0 1])
            quiver(close(in,1),close(in,2),0,0.5,'ok');hold on;
        elseif isSamePosi(policy,[-1 0])
            quiver(close(in,1),close(in,2),-0.5,0,'ok');hold on;
        elseif isSamePosi(policy,[0 -1])
            quiver(close(in,1),close(in,2),0,-0.5,'ok');hold on;
        elseif isSamePosi(policy,[1 1])
            quiver(close(in,1),close(in,2),0.5,0.5,'ok');hold on;
        elseif isSamePosi(policy,[-1 1])
            quiver(close(in,1),close(in,2),-0.5,0.5,'ok');hold on;
        elseif isSamePosi(policy,[1 -1])
            quiver(close(in,1),close(in,2),0.5,-0.5,'ok');hold on;
        elseif isSamePosi(policy,[-1 -1])
            quiver(close(in,1),close(in,2),-0.5,-0.5,'ok');hold on;
        end
    end
    
end

axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
grid on;

end

function path=Dijkstra(obstacle,p)
% ダイクストラ法によって最短経路を探索するプログラム
% 最短経路のパスの座標リストを返す

path=[];%パス
%計算中ノード情報格納用[x,y,cost,px(親ノード),py(親ノード)] startノードを格納する
open=[p.start(1) p.start(2) 0 p.start(1) p.start(2)];
close=[];%計算済みノード情報格納用

%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
next=MotionModel();

findFlag=false;%ゴール発見フラグ

while ~findFlag
    %すべてのopenなノードに対して隣接ノードのコストを計算する
    initNopen=length(open(:,1));
    %openにデータがない場合はパスが見つからなかった。
    if initNopen==0 disp('No path to goal!!'); return; end
    for io=1:initNopen
        %ゴール判定
        if isSamePosi(open(io,1:2),p.goal)
            disp('Find Goal!!');
            %openリストとgoalリストを結合
            close=[open;close];
            findFlag=true;
            break;
        end
        
        for in=1:length(next(:,1))
            %隣接ノードの位置とコストの計算
            m=[open(io,1)+next(in,1) open(io,2)+next(in,2) open(io,3)+next(in,3)];
            
            %隣接ノードが障害物だったら次のノードを探す
            if isObstacle(m,obstacle) continue; end
            
            %openとcloseのリストの中にmが含まれるかを探索
            [flag, targetInd]=FindList(m,open,close);
            
            %openリストにある場合
            if flag==1
                if m(3)<open(targetInd,3)
                    open(targetInd,3)=m(3);
                    open(targetInd,4)=open(io,1);
                    open(targetInd,5)=open(io,2);
                end
            elseif flag==2 %closeリストにある場合
                if m(3)<close(targetInd,3)
                    %親ノードの更新
                    close(targetInd,4)=open(io,1);
                    close(targetInd,5)=open(io,2);
                    open=[open; close(targetInd,:)];
                    close(targetInd,:)=[];%Openリストに移動
                end
            else %どちらにも無い場合
                %openリストに親ノードのインデックスと共に追加
                open=[open;[m open(io,1) open(io,2)]];
            end
        end
    end
    %隣接ノード計算したopenノードはcloseノードへ移動
    if findFlag==false
        close=[close; open(1:initNopen,:)];
        open(1:initNopen,:)=[];
    end
    
    %パス探索のステップ動画
    %animation(open,close,p,obstacle);

end

%最短パスの座標リストを取得
path=GetPath(close,p.start,io);

end

function obstacle=GetObstacle(nob,obstacle,p)
%乱数で指定された個数の障害物を作成し、
%それらの中からスタートorゴール地点に配置されたもの以外を返す関数

%乱数で障害物を作成
ob=round(rand([nob,2])*p.XYMAX);

%スタート地点とゴールに障害物が配置された場合は省く
removeInd=[];%削除する障害物のインデックス格納用リスト
for io=1:length(ob(:,1))
    if(isSamePosi(ob(io,:),p.start) || isSamePosi(ob(io,:),p.goal))
        removeInd=[removeInd;io];
    end   
end
ob(removeInd,:)=[];%リスト

obstacle=[obstacle;ob];

end

function result=isSamePosi(a,b)
%2x1のベクトルが同じかどうかを判断する関数
result=false;
if a(1)==b(1) && a(2)==b(2)
    result=true;
end
end

function boundary=GetBoundary(p)
% エリア境界データの取得
boundary=[];
for i1=0:(p.XYMAX+1)
    boundary=[boundary;[0 i1]];
end
for i2=0:(p.XYMAX+1)
    boundary=[boundary;[i2 0]];
end
for i3=0:(p.XYMAX+1)
    boundary=[boundary;[p.XYMAX+1 i3]];
end
for i4=0:(p.XYMAX+1)
    boundary=[boundary;[i4 p.XYMAX+1]];
end
boundary=[boundary;[11 11]];
boundary=[boundary;[9 1]];
boundary=[boundary;[10 2]];
boundary=[boundary;[11 3]];
boundary=[boundary;[10 1]];
boundary=[boundary;[11 2]];
boundary=[boundary;[11 1]];

end

function animation(open,close,p,obstacle)
% 探索の様子を逐次的に表示する関数

figure(1)
if length(obstacle)>=1
    plot(obstacle(:,1),obstacle(:,2),'om');hold on;
end
plot(p.start(1),p.start(2),'*r');hold on;
plot(p.goal(1),p.goal(2),'*b');hold on;
plot(open(:,1),open(:,2),'xr');hold on;
plot(close(:,1),close(:,2),'xk');hold on;

axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
grid on;
pause;

end

function flag=isObstacle(m,obstacle)

for io=1:length(obstacle(:,1))
    if isSamePosi(obstacle(io,:),m(1:2))
        flag=true;return;
    end
end
flag=false;%障害物ではない
end

function next=MotionModel()
%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
%またコストの値を調整することで、ロボットの高度な経路設定が実現できる
% [x y cost]
next=[1 1 1
      1 0 1
      0 1 1
      -1 0 1
      0 -1 1
      -1 -1 1
      -1 1 1
      1 -1 1];

end

function path=GetPath(close,start,ig)
%スタートからゴールまでの座標リストを取得する関数
ind=ig;%goalのインデックス
path=[];
while 1
    %座標をリストに登録
    path=[path; close(ind,1:2)];
    
    %スタート地点まで到達したか判断
    if isSamePosi(close(ind,1:2),start)
        break;
    end
    
    %closeリストの中で親ノードを探す
    for io=1:length(close(:,1))
        if isSamePosi(close(io,1:2),close(ind,4:5))
            ind=io;
            break;
        end
    end
end

end

function [flag, targetInd]=FindList(m,open,close)
    targetInd=0;
    %openリストにあるか?
    if ~isempty(open)
        for io=1:length(open(:,1))
            if isSamePosi(open(io,:),m(1:2))
                flag=1;
                targetInd=io;
                return;
            end
        end
    end
    %closeリストにあるか?
    if ~isempty(close)
        for ic=1:length(close(:,1))
            if isSamePosi(close(ic,:),m(1:2))
                flag=2;
                targetInd=ic;
                return;
            end
        end
    end
    %どちらにも無かった
    flag=3;return;
end
    

    



