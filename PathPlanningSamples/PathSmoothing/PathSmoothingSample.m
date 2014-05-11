function PathSmoothingSample()
%PathSmoothingSample() パス平滑化用MATLABサンプルコード
%
% Author: Atsushi Sakai
%
% Reference:MATLABよる経路平滑化(Path Smoothing)プログラム - 
%           http://d.hatena.ne.jp/meison_amsl/20140510/1399694663
%
% Copyright (c) 2014, Atsushi Sakai
% All rights reserved.
% License : Modified BSD Software License Agreement
clear all;
close all;

%スムージング前のパス
path=[0 0;
      1 0;
      2 0;
      3 0
      3 1
      3 2
      3 3
      3 4
      4 4
      5 4
      6 4];
  
optPath=PathSmoothing(path);
  
plot(path(:,1),path(:,2),'-.b');hold on;
plot(optPath(:,1),optPath(:,2),'-or');hold on;
axis([-1 7 -2 6])
legend('Before','After');
title('Path Smoothing');
grid on;

end

function optPath=PathSmoothing(path)
optPath=path;%元のパスをコピー

%平準化パラメータ
alpha=0.5;
beta=0.2;

torelance=0.00001;%パスの変化量の閾値(変化量がこの値以下の時平滑化を終了)
change=torelance;%パスの位置の変化量
while change>=torelance 
    change=0;%初期化
    for ip=2:(length(path(:,1))-1) %始点と終点は固定
        prePath=optPath(ip,:);%変化量計測用
        optPath(ip,:)=optPath(ip,:)-alpha*(optPath(ip,:)-path(ip,:));
        optPath(ip,:)=optPath(ip,:)-beta*(2*optPath(ip,:)-optPath(ip-1,:)-optPath(ip+1,:));
        change=change+norm(optPath(ip,:)-prePath);
    end
end

end