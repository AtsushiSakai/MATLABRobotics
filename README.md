MATLABRobotics
==============

MATLAB Sample Codes for Robotics


MATLABを使った自律移動用ロボット用サンプルコードです。

それぞれのコードの概要は下記の通りです。

それぞれのアルゴリズムやコードの説明は、

各ブログの記事を御覧ください。


# Localization
位置計測系のサンプルコード用フォルダです。

##ExtenedKalmanFilterLocalization
![EKF](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20130413/20130413125635.jpg)

拡張カルマンフィルタを使用した自己位置推定プログラムです。

拡張カルマンフィルタを使用した自己位置推定MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20130413/1365826157

##UnscentedKalmanFilterLocalization
![UKF](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140614/20140614163514.png)

Unscentedカルマンフィルタ(シグマポイントカルマンフィルタ)を使用した自己位置推定プログラムです。

Unscentedカルマンフィルタを使用した自己位置推定MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140614/1402731732

##ParticleFilterLocalization
![PF](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140628/20140628203642.png)

Particle Filterを使用した自己位置推定プログラムです。

Particle Filterを使用した自己位置推定MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140628/1403956852


#PathPlanning
経路生成系のサンプルコード用フォルダです。

##Dijkstra
![Dijkstra](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140502/20140502120424.png)

ダイクストラ法を用いた経路生成プログラムです。

動的計画法(Dyamic Programing)的なシミュレーションも可能です。

ダイクストラ法による最短経路探索MATLABプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140502/1399001915

##AStar
![AStar](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140503/20140503100405.png)

A*を用いた最短経路生成プログラムです。

A*による最短経路探索MATLABプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140503/1399080847

##PathSmoothing
![PathSmoothing](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140510/20140510123208.png)

単純な勾配法を使用したパス平滑化プログラムです。

MATLABよる経路平滑化(Path Smoothing)プログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140510/1399694663

##Dynamic Window Approach
![Dynamic Window Approach](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140624/20140624230043.png)

Dynamic Window Approachを使用したLocal Path Planningプログラムです。

Dynamic Window ApproachのMATLAB サンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140624/1403618922

#SLAM
Simultaneous Localization And Mapping(SLAM)のサンプルコード用フォルダです。

##ICP
![ICP](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140617/20140617112008.png)

Iterative Closest Point (ICP) Algorithmを使用したScanデータに基づく相対移動量推定プログラムです。

ICPアルゴリズムを利用したSLAM用MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140617/1402971928

##EKF SLAM
![EKFSLAM](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140720/20140720215913.png)

拡張カルマンフィルタを使用した特徴ベースのSLAMのサンプルプログラムです。

EKFによるSLAMのためのMATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140720

#Machine Learning
機械学習やパターン認識のサンプルコード用フォルダです。

## EM Algorithm
![EM](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20140710/20140710202357.png)

EMアルゴリズムによる混合正規分布のパラメータ学習のサンプルコードです。

EMアルゴリズムによる確率分布学習のMATLABプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140710/1404997476

## Nelder Mead
![NelderMead](http://cdn-ak.f.st-hatena.com/images/fotolife/m/meison_amsl/20141216/20141216222923.png)

Nelder Meadアルゴリズムによる非線形最適化のサンプルコードです。

Nelder-Mead法(シンプレックス法)による非線形最適化MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20141217/1418824777

## Steepest Descent Method
![Steepest](http://f.st-hatena.com/images/fotolife/m/meison_amsl/20141221/20141221192623.png)

最急降下法による非線形最適化のサンプルコードです。

最急降下法による非線形最適化MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20141221/1419163905

