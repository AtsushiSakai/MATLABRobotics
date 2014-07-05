function [] = HybridAStarSample()
%AStarSample() A*法による最短経路探索プログラム
%
% Author: Atsushi Sakai
%
% Copyright (c) 2014, Atsushi Sakai
% All rights reserved.
% License : Modified BSD Software License Agreement

clear all;
close all;
disp('Hybrid A Star Path Planning start!!');

%パラメータ
p.angleTick=5; %角度解像度[deg]
p.start=[1, 1,  0/p.angleTick];%スタート地点 [x,y,yaw]
p.goal =[9,7,-135/p.angleTick];%ゴール地点   [x,y,yaw]
p.XYMAX=11;     %マップの最大縦横

%境界データの取得
%obstacle=GetBoundary(p);

%障害物データの取得 境界データと合わせて取得する
%nObstacle=20;%障害物の数
%obstacle=GetObstacle(nObstacle,obstacle,p);
obstacle=[];

%最短経路を生成
path=HybridAStar(obstacle,p);

%グラフ作成
figure(1)
hold off;
% if length(obstacle)>=1
%     plot(obstacle(:,1),obstacle(:,2),'om');hold on;
% end
PlotArrow(p.start,p.angleTick);
PlotArrow(p.goal,p.angleTick);
for i=1:length(path(:,1))
    PlotArrow(path(i,:),p.angleTick);hold on;
end
plot(path(:,1),path(:,2),'-r');hold on;
axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
grid on;

end

function path=HybridAStar(obstacle,p)
% A*法によって最短経路を探索するプログラム
% 最短経路のパスの座標リストを返す

path=[];%パス
%計算中ノード情報格納用[x,y,yaw,cost,px(親ノード),py,pyaw] startノードを格納する
%open=[p.start h(p.start,p.goal) p.start];
oepn.x=p.start;%ノード
oepn.cost=h(p.start,p.goal);%コスト
oepn.px=p.start;%親ノード

close=[];%計算済みノード情報格納用

%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
next=MotionModel(p.angleTick);

findFlag=false;%ゴール発見フラグ

while ~findFlag
      %openにデータがない場合はパスが見つからなかった。  
      if isempty(open(:,1)) disp('No path to goal!!'); return; end
      %openなノードの中で最もコストが小さいものを選ぶ
      [Y,I] = sort(open(:,4));
      open=open(I,:);
      
      %ゴール判定
      if isSamePosi(open(1,1:3),p.goal)
          disp('Find Goal!!');
          %ゴールのノードをCloseの先頭に移動
          close=[open(1,:);close];open(1,:)=[];
          findFlag=true;
          break;
      end
      
      for in=1:length(next(:,1))
          %隣接ノードの位置とコストの計算
          phi=toRadian(open(1,3)*p.angleTick);
          R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
          xy=next(in,1:2)*R;
          m=[open(1,1)+xy(1) open(1,2)+xy(2) open(1,3)+next(in,3) open(1,4)];
          m(3)=PI2PI(m(3),p.angleTick);
          m(4)=m(4)+next(in,4)+h(m(1:3),p.goal)-h(open(1,1:3),p.goal);%コストの計算
  
          %pause;
          %隣接ノードが障害物だったら次のノードを探す
          %if isObstacle(m,obstacle) continue; end
          
          %openとcloseのリストの中にmが含まれるかを探索
          [flag, targetInd]=FindList(m,open,close);

          if flag==1 %openリストにある場合
              if m(4)<open(targetInd,4)
                  open(targetInd,4)=m(4);
                  open(targetInd,5)=open(1,1);
                  open(targetInd,6)=open(1,2);
                  open(targetInd,7)=open(1,3);
              end
          elseif flag==2 %closeリストにある場合
              if m(4)<close(targetInd,4)
                  %親ノードの更新
                  %close(targetInd,4)=m(4);
                  close(targetInd,5)=open(1,1);
                  close(targetInd,6)=open(1,2);
                  close(targetInd,7)=open(1,3);
                  open=[open; close(targetInd,:)];
                  close(targetInd,:)=[];%Openリストに移動
              end
          else %どちらにも無い場合
              %openリストに親ノードのインデックスと共に追加
              open=[open;[m open(1,1) open(1,2) open(1,3)]];
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
result=norm(a(1:2)-b(1:2));

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
%3x1のベクトルが同じかどうかを判断する関数
result=false;
d=a(1:3)-b;
if abs(d(1))<0.3 && abs(d(2))<0.3 && abs(d(3))<1
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
hold off;
%if length(obstacle)>=1
%    plot(obstacle(:,1),obstacle(:,2),'om');hold on;
%end
plot3(p.start(1),p.start(2),p.start(3),'*r');hold on;
plot3(p.goal(1),p.goal(2),p.goal(3),'*b');hold on;
plot3(open(:,1),open(:,2),open(:,3),'xr');hold on;
plot3(close(:,1),close(:,2),close(:,3),'xk');hold on;

%axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
grid on;
%pause;

end

function flag=isObstacle(m,obstacle)

for io=1:length(obstacle(:,1))
    if isSamePosi(obstacle(io,:),m(1:2))
        flag=true;return;
    end
end
flag=false;%障害物ではない
end

function next=MotionModel(tick)
%隣接ノードへの移動モデル これを変えることでロボットの移動を指定できる
% [dx dy dyaw cost]
next=[1 0    0/tick 1
      1 1   45/tick 2
      2 1   45/tick 2
      1 -1  -45/tick 1
      2 -1  -45/tick 1
      -1 0   0/tick 3
      -1 1  45/tick 4
      -1 -1 -45/tick 4];
end

function path=GetPath(close,start)
%スタートからゴールまでの座標リストを取得する関数
ind=1;%goalはcloseリストの先頭に入っている
path=[];
while 1
    %座標をリストに登録
    path=[path; close(ind,1:3)];
    
    %スタート地点まで到達したか判断
    if isSamePosi(close(ind,1:3),start)   
        break;
    end
    
    %closeリストの中で親ノードを探す
    for io=1:length(close(:,1))
        if isSamePosi(close(io,1:3),close(ind,5:7))
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
            if isSamePosi(open(io,:),m(1:3))
                flag=1;
                targetInd=io;
                return;
            end
        end
    end
    %closeリストにあるか?
    if ~isempty(close)
        for ic=1:length(close(:,1))
            if isSamePosi(close(ic,:),m(1:3))
                flag=2;
                targetInd=ic;
                return;
            end
        end
    end
    %どちらにも無かった
    flag=3;return;
end

function angle=PI2PI(angle,tick)
    angledeg=angle*tick;
    while angledeg>180
        angledeg=angledeg-360;
    end
    while angledeg<-180
        angledeg=angledeg+360;
    end
    angle=angledeg/tick;
end

function PlotArrow(x,tick)
ArrowLength=0.5;%矢印の長さ
yaw=x(3)*tick*pi/180;
quiver(x(1),x(2),ArrowLength*cos(yaw),ArrowLength*sin(yaw),'ok');hold on;
end   

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;
end
    



