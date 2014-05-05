function [] = AStarSample()
%DijkstraSample() ダイクストラ法による最短経路探索プログラム
%
% Author: Atsushi Sakai
%
% Copyright (c) 2014, Atsushi Sakai
% All rights reserved.
% License : Modified BSD Software License Agreement
clear all;
close all;
disp('A Star Path Planning start!!');

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
path=AStar(obstacle,p);

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

end

function path=AStar(obstacle,p)
% A*法によって最短経路を探索するプログラム
% 最短経路のパスの座標リストを返す

path=[];%パス
%計算中ノード情報格納用[x,y,cost,px(親ノード),py(親ノード)] startノードを格納する
open=[p.start(1) p.start(2) h(p.start,p.goal) p.start(1) p.start(2)];
close=[];%計算済みノード情報格納用

%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
next=MotionModel();

findFlag=false;%ゴール発見フラグ

while ~findFlag
      %openにデータがない場合はパスが見つからなかった。  
      if isempty(open(:,1)) disp('No path to goal!!'); return; end
      %openなノードの中で最もコストが小さいものを選ぶ
      [Y,I] = sort(open(:,3));
      open=open(I,:);
      
      %ゴール判定
      if isSamePosi(open(1,1:2),p.goal)
          disp('Find Goal!!');
          %ゴールのノードをCloseの先頭に移動
          close=[open(1,:);close];open(1,:)=[];
          findFlag=true;
          break;
      end
      
      for in=1:length(next(:,1))
          %隣接ノードの位置とコストの計算
          m=[open(1,1)+next(in,1) open(1,2)+next(in,2) open(1,3)];
          m(3)=m(3)+next(in,3)+h(m(1:2),p.goal)-h(open(1,1:2),p.goal);%コストの計算
          
          %隣接ノードが障害物だったら次のノードを探す
          if isObstacle(m,obstacle) continue; end
          
          %openとcloseのリストの中にmが含まれるかを探索
          [flag, targetInd]=FindList(m,open,close);

          if flag==1 %openリストにある場合
              if m(3)<open(targetInd,3)
                  open(targetInd,3)=m(3);
                  open(targetInd,4)=open(1,1);
                  open(targetInd,5)=open(1,2);
              end
          elseif flag==2 %closeリストにある場合
              if m(3)<close(targetInd,3)
                  %親ノードの更新
                  close(targetInd,4)=open(1,1);
                  close(targetInd,5)=open(1,2);
                  open=[open; close(targetInd,:)];
                  close(targetInd,:)=[];%Openリストに移動
              end
          else %どちらにも無い場合
              %openリストに親ノードのインデックスと共に追加
              open=[open;[m open(1,1) open(1,2)]];
          end
      end

      %隣接ノード計算したopenノードはcloseノードへ移動
      if findFlag==false
          close=[close; open(1,:)];
          open(1,:)=[];
      end
      
      %パス探索のステップ動画
      %animation(open,close,p,obstacle);

end

%最短パスの座標リストを取得
path=GetPath(close,p.start);

end

function result=h(a,b)
%ヒューリスティック関数
%ここでは二次元空間のa,bのノルム距離
result=norm(a-b);

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

function [next,cost]=MotionModel()
%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
% [x y cost]
next=[1 1 1
      1 0 1
      0 1 1
      -1 0 1
      0 -1 1
      -1 -1 1];
end

function path=GetPath(close,start)
%スタートからゴールまでの座標リストを取得する関数
ind=1;%goalはcloseリストの先頭に入っている
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
    

    



