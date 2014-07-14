% -------------------------------------------------------------------------
%
% File : GridMapSample.m
%
% Discription : Sample code to build grid map
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------

function GridMapSample()
close all;
clear all;

%Grid Mapのパラメータ
gm.CENTER=[0 0];%グリッドマップの中心座標[x y]
gm.RESO=1;%解像度
gm.WIDTH=100;
gm.HEIGHT=100;
gm.nGrid=gm.WIDTH*gm.HEIGHT;
%dataには物体の存在確率[0,1]を格納する。
%初期値は不明なので0.5とする
gm.data=zeros(1,gm.nGrid)+0.5;

pose=[0 0 0];%ロボットの現在姿勢[x,y,yaw]

z=GetObservation();%センサの観測点の取得

%シミュレーション1:End Point Update
gm1=HitGridUpdate(gm,z);
figure(1);
ShowGridMap(gm1,z);%グラフ表示

%シミュレーション2:Likelihood Update
gm2=LikelihoodUpdate(gm,z);
figure(2);
ShowGridMap(gm2,z);%グラフ表示

%シミュレーション3:Ray Casting Update
gm3=RayCastingUpdate(gm,z);
figure(3);
ShowGridMap(gm3,z);%グラフ表示

function gm=RayCastingUpdate(gm,z)
%レイキャスティングによるGridの更新

%事前レイキャスティングモデルの作成
gm=PreCasting(gm,z.ANGLE_TICK);

rayId=0;
%事前レイキャスティングモデルに従ってグリッドの確率の更新
for iz=1:length(z.data(:,1))%それぞれの観測点に対して
    range=z.data(iz,1);
    
    rayId=rayId+1;%レイキャスティングクラスタにおけるデータID
    %各観測点はそれぞれのクラスタから取得できるとする。
    
    %クラスタ内の各gridのデータからビームモデルによるupdate
    for ir=1:length(gm.precasting(rayId).grid(:,1))
        grange=gm.precasting(rayId).grid(ir,1);
        gid=gm.precasting(rayId).grid(ir,5);
        
        if grange<(range-gm.RESO/2) %free
            gm.data(gid)=0;
        elseif grange<(range+gm.RESO/2) %hit
            gm.data(gid)=1;
        end %それ以上の距離のgridはunknownなので何もしない
    end
end

function gm=PreCasting(gm,angleTick)
%事前レイキャスティングモデルの作成

%各角度について対応するグリッドを追加していく
precasting=[];%プレキャスティングの結果 [最小角度,最大角度,中に入るgridのデータ]
for ia=(0-angleTick/2):angleTick:(360+angleTick/2)
    %角度範囲の保存
    ray.minAngle=ia;
    ray.maxAngle=ia+angleTick;
    grid=[];%角度範囲に入ったグリッドのデータ
    for ig=1:(gm.nGrid)
        %各グリッドのxy値を取得
        gxy=GetXYFromDataIndex(ig,gm);
        range=norm(gxy);
        angle=atan2(gxy(2),gxy(1));
        if angle<0 %[0 360]度に変換
            angle=angle+2*pi;
        end
        if ray.minAngle<=rad2deg(angle) && ray.maxAngle>=rad2deg(angle)
            grid=[grid;[range,angle,gxy,ig]];
        end
    end
    %rangeの値でソーティングしておく
    if ~isempty(grid)
        ray.grid=sortrows(grid,1);
    end
    precasting=[precasting;ray];
end
gm.precasting=precasting;%Grid Mapデータに追加


function gm=LikelihoodUpdate(gm,z)
%尤度場のGridMapを作る関数

for ig=1:(gm.nGrid-1)
    gxy=GetXYFromDataIndex(ig,gm);
    zxy=FindNearest(gxy,z);%最近傍の観測値の取得
    p=GaussLikelihood(gxy,zxy);%ガウシアン尤度の計算
    gm.data(ig)=p*10;%グリッドへの格納
end
%gm.data=Normalize(gm.data);%正規化

function p=GaussLikelihood(gxy,zxy)
%ガウス分布の尤度を計算する関数
Sigma=diag([3,3]);%共分散行列
p=det(2*pi*Sigma)^(-0.5)*exp(-0.5*(gxy-zxy)*inv(Sigma)*(gxy-zxy)');

function zxy=FindNearest(xy,z)
%ある座標値xyに一番近いzの値を返す関数

%すべてのzとxyの差を計算
d=z.data(:,3:4)-repmat(xy,length(z.data(:,1)),1);

%ノルム距離の最小値のインデックスを取得
min=inf;%最小値
minid=0;
for id=1:length(d(:,1))
    nd=norm(d(id,:));
    if min>nd
        min=nd;
        minid=id;
    end
end
zxy=z.data(minid,3:4);

function xy=GetXYFromDataIndex(ig,gm)
%Gridのデータインデックスから,そのグリッドのx,y座標を取得する関数

%x,yインデックスの取得
indy=rem(ig,gm.WIDTH)-1;
indx=fix(ig/gm.WIDTH);

x=GetXYPosition(indx,gm.WIDTH,gm.RESO);
y=GetXYPosition(indy,gm.HEIGHT,gm.RESO);
xy=[x y];

function position=GetXYPosition(index, width, resolution)
%x-yインデックスの値から、位置を取得する関数
position=resolution*(index-width/2)+resolution/2;

function gm=HitGridUpdate(gm,z)
%観測点がヒットしたグリッドの確率を1にする関数

for iz=1:length(z.data(:,1))
    zx=z.data(iz,3);
    zy=z.data(iz,4);
    ind=GetDBIndexFromXY(zx,zy,gm);
    gm.data(ind)=1.0;
end
gm.data=Normalize(gm.data);%正規化

function ind=GetDBIndexFromXY(x,y,gm)
%xyの値から対応するグリッドのベクトルインデックスを取得する関数
%グリッドマップ中心座標系に変換
x=x-gm.CENTER(1);
y=y-gm.CENTER(2);

%グリッドマップのインデックスを取得
indX=GetXYIndex(x,gm.WIDTH, gm.RESO);%x軸方向のindex
indY=GetXYIndex(y,gm.HEIGHT,gm.RESO)+1;%y軸方向のindex
ind=GetDBIndexFromIndex(indX,indY,gm);%DBのインデックス

function ind=GetDBIndexFromIndex(indX,indY,gm)
%全体インデックス
ind=gm.WIDTH*indX+indY;
%X方向のインデックスチェック
if(indX>=gm.WIDTH)
    ind=-1;
%Y方向のインデックスチェック
elseif(indY>=gm.HEIGHT)
    ind=-1;        
%全体インデックスのチェック
elseif(ind>=gm.nGrid)
    ind=-1;
end

function ind=GetXYIndex(position, width, resolution)
%x-y方向の値から、インデックスを取得する関数
ind=fix((position/resolution+width/2.0));

function z=GetObservation()
%観測点をセンサのモデルに基いて、ランダムに取得する関数
z.data=[];% 観測値[range, angle x y;...]
z.ANGLE_TICK=10;%スキャンレーザの角度解像度[deg]
z.MAX_RANGE=50;%スキャンレーザの最大観測距離[m]
z.MIN_RANGE=5;%スキャンレーザの最小さい観測距離[m]

for angle=0:z.ANGLE_TICK:360
    range=rand()*(z.MAX_RANGE-z.MIN_RANGE)+z.MIN_RANGE;
    rad=toRadian(angle);
    x=range*cos(rad);
    y=range*sin(rad);
    z.data=[z.data;[range rad x y]]; 
end

function ShowGridMap(gm,z)
%グリッドマップを表示する関数
xmin=gm.CENTER(1)-gm.WIDTH*gm.RESO/2;
xmax=gm.CENTER(1)+gm.WIDTH*gm.RESO/2-gm.RESO;
ymin=gm.CENTER(2)-gm.HEIGHT*gm.RESO/2;
ymax=gm.CENTER(2)+gm.HEIGHT*gm.RESO/2-gm.RESO;
%XYのインデックス用変数の作成
[X,Y] = meshgrid(xmin:gm.RESO:xmax, ymin:gm.RESO:ymax);
sizeX=size(X);
data=reshape(gm.data,sizeX(1),sizeX(2));%ベクトルデータを行列に
surf(X,Y,data);hold on;
view(2)
%観測点
plot3(z.data(:,3),z.data(:,4),zeros(1,length(z.data(:,1)))+1.0,'xy');
plot3(0,0,1.0,'^c');%ロボットの位置

function Z=Normalize(Z)
%グリッドマップの尤度を正規化する関数
sumZ=sum(sum(Z,1,'double'),'double');
Z=Z./sumZ;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;





