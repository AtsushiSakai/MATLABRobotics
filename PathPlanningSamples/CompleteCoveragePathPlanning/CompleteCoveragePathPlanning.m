function [] = CompleteCoveragePathPlanning()
clear all;
close all;
disp('CompleteCoveragePathPlanning start!!');

%パラメータ
p.start=[1,1];  %スタート地点
p.goal=[10,10]; %ゴール地点
p.XYMAX=11;     %マップの最大縦横

%Mapの初期化　Unknown=inf
map=zeros(p.XYMAX,p.XYMAX)+inf;

%ゴールをコスト0に設定
map(p.goal(1),p.goal(2))=0;

%境界データの取得
p=GetBoundary(p);

%障害物データ
p.obstacle=[];
%p.obstacle=[3 3];
%p.obstacle=[3 3;5 7];
p.obstacle=[3 1;3 2;3 3;3 4;3 5;3 6;7 7];

%MapをDistance transform
%map=DistanceTransform(map,p);

%Obstacle Transform
map=PathTransform(map,p);

%最短経路を生成
sind=ComputeShortestPath(map,p);

%カバレージパスの生成
cind=ComputeCoveragePath(map,p);

%パスのステップ動画
animation(cind,p);

%グラフ作成
figure(1)
plot(p.boundary(:,1),p.boundary(:,2),'xk');hold on;
if length(p.obstacle)>=1
    plot(p.obstacle(:,1),p.obstacle(:,2),'om');hold on;
end
plot(p.start(1),p.start(2),'*r');hold on;
plot(p.goal(1),p.goal(2),'*b');hold on;
plot(cind(:,1),cind(:,2),'-b');hold on;
plot(sind(:,1),sind(:,2),'-r');hold on;
axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
grid on;

end

function map=PathTransform(map,p)

obmap=ObstacleTransform(map,p);

while(1)
    maptmp=map;%参照用にコピー
    for ix=1:p.XYMAX
        for iy=1:p.XYMAX
            for incx=-1:1
                for incy=-1:1
                    %参照先のグリッドのインデックス
                    indx=ix+incx;
                    indy=iy+incy;
                    
                    %自分自身を参照していたらスキップ
                    if (incx==0)&&(incy==0) continue;end
                    
                    %インデックスがオーバフローしたらスキップ
                    if(indx<=0||indx>p.XYMAX) continue;end
                    if(indy<=0||indy>p.XYMAX) continue;end
                    
                    %目標のグリットがinf以外の場合
                    if maptmp(indx,indy)~=inf
                        c=maptmp(indx,indy)+1+1/obmap(ix,iy);
                        if c<=map(ix,iy) map(ix,iy)=c; end;
                    end
                end
            end
        end
    end
    %Start地点まで探索完了
    if map(p.start(1),p.start(2))~=inf break; end;
end

end

function map=ObstacleTransform(map,p)
%すべてのグリットに対して、障害物との距離を計算し、その最小値を格納する

%境界データと障害物データを格納
ob=[p.boundary;p.obstacle];
for ix=1:p.XYMAX
    for iy=1:p.XYMAX
        n=[];
        for io=1:length(ob(:,1))
            n=[n norm(ob(io,:)-[ix iy])];
        end
        map(ix,iy)=min(n);
    end
end

end

function p=GetBoundary(p)
p.boundary=[];
for i1=0:(p.XYMAX+1)
    p.boundary=[p.boundary;[0 i1]];
end
for i2=0:(p.XYMAX+1)
    p.boundary=[p.boundary;[i2 0]];
end
for i3=0:(p.XYMAX+1)
    p.boundary=[p.boundary;[p.XYMAX+1 i3]];
end
for i4=0:(p.XYMAX+1)
    p.boundary=[p.boundary;[i4 p.XYMAX+1]];
end
p.boundary=[p.boundary;[11 11]];
p.boundary=[p.boundary;[9 1]];
p.boundary=[p.boundary;[10 2]];
p.boundary=[p.boundary;[11 3]];
p.boundary=[p.boundary;[10 1]];
p.boundary=[p.boundary;[11 2]];
p.boundary=[p.boundary;[11 1]];

end

function animation(ind,p)

for i=1:length(ind(:,1))
    hold off;
    plot(p.boundary(:,1),p.boundary(:,2),'xk');hold on;
    if length(p.obstacle)>=1
        plot(p.obstacle(:,1),p.obstacle(:,2),'og');hold on;
    end
    plot(p.start(1),p.start(2),'*r');hold on;
    plot(p.goal(1),p.goal(2),'*b');hold on;
    plot(ind(i,1),ind(i,2),'*r');hold on;
    plot(ind(1:i,1),ind(1:i,2),'-b');hold on;
    axis([0-0.5 p.XYMAX+1+0.5 0-0.5 p.XYMAX+1+0.5])
    grid on;
    pause;
end

end

function indresult=ComputeCoveragePath(map,p)

indresult=[];
ix=p.start(1);
iy=p.start(2);
while(1)
    result=[];
    indresult=[indresult;[ix iy]];
    map(ix,iy)=-1;
    for incx=-1:1
        for incy=-1:1
            indx=ix+incx;
            indy=iy+incy;
            
            %自分自身を参照していたらスキップ
            if (incx==0)&&(incy==0) continue;end
 
            %インデックスがオーバフローしたらスキップ
            if(indx<=0||indx>p.XYMAX) continue;end
            if(indy<=0||indy>p.XYMAX) continue;end
            
            if (map(indx,indy)>=0)&&(map(indx,indy)~=inf)
                result=[result;[map(indx,indy) indx indy]];
            end
        end
    end
    %ローカルミニマ
    if length(result)==0
        disp('local minima!!');
        map(indresult(end-1,1),indresult(end-1,2))=100;
        continue;
        %break;
    end
    rsize=size(result(:,1));
    %result
    %indresult
    %resultが一個しか無いとき
    if rsize(1,1)~=1
        [mi mj]=max(result);
        ix=result(mj(1),2);
        iy=result(mj(1),3);
    else
        ix=result(1,2);
        iy=result(1,3);
    end
    if ix==p.goal(1)&&iy==p.goal(2)
        indresult=[indresult;[ix iy]];
        break;
    end
end

end

function indresult=ComputeShortestPath(map,p)
indresult=[];
ix=p.start(1);
iy=p.start(2);
while(1)
    result=[];
    indresult=[indresult;[ix iy]];
    for incx=-1:1
        for incy=-1:1
            indx=ix+incx;
            indy=iy+incy;
            
            %自分自身を参照していたらスキップ
            if (incx==0)&&(incy==0) continue;end
            
            %インデックスがオーバフローしたらスキップ
            if(indx<=0||indx>p.XYMAX) continue;end
            if(indy<=0||indy>p.XYMAX) continue;end
            
            result=[result;[map(indx,indy) indx indy]];
        end
    end
    [mi mj]=min(result);
    ix=result(mj(1),2);
    iy=result(mj(1),3);
    
    if ix==p.goal(1)&&iy==p.goal(2)
        indresult=[indresult;[ix iy]];
        break;
    end
    
end

end

function map=DistanceTransform(map,p)

while(1)
    maptmp=map;%参照用にコピー
    for ix=1:p.XYMAX
        for iy=1:p.XYMAX
            for incx=-1:1
                for incy=-1:1
                    indx=ix+incx;
                    indy=iy+incy;
                    
                    %自分自身を参照していたらスキップ
                    if (incx==0)&&(incy==0) continue;end
                    
                    %インデックスがオーバフローしたらスキップ
                    if(indx<=0||indx>p.XYMAX) continue;end
                    if(indy<=0||indy>p.XYMAX) continue;end
                    
                    %カレントグリットがinfで目標のグリットがinf以外の場合
                    if maptmp(indx,indy)~=inf&&(map(ix,iy)==inf)
                        map(ix,iy)=maptmp(indx,indy)+1;
                    end
                end
            end
        end
    end
    %Start地点まで探索完了
    if map(p.start(1),p.start(2))~=inf break; end;
end

end

