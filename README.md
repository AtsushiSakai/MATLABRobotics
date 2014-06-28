MATLABRobotics
==============

MATLAB Sample Codes for Robotics


MATLABを使った自律移動用ロボット用サンプルコードです。

それぞれのコードの概要は下記の通りです。

それぞれのアルゴリズムやコードの説明は、

参照されているブログの記事を御覧ください。


# Localization
位置計測系のサンプルコード用フォルダです。

##ExtenedKalmanFilterLocalization
拡張カルマンフィルタを使用した自己位置推定プログラムです。

拡張カルマンフィルタを使用した自己位置推定MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20130413/1365826157

##UnscentedKalmanFilterLocalization
Unscentedカルマンフィルタ(シグマポイントカルマンフィルタ)を使用した自己位置推定プログラムです。

Unscentedカルマンフィルタを使用した自己位置推定MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140614/1402731732



#PathPlanning
経路生成系のサンプルコード用フォルダです。

##Dijkstra
ダイクストラ法を用いた経路生成プログラムです。
動的計画法(Dyamic Programing)的なシミュレーションも可能です。

ダイクストラ法による最短経路探索MATLABプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140502/1399001915

##AStar
A*を用いた最短経路生成プログラムです。

A*による最短経路探索MATLABプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140503/1399080847

##PathSmoothing
単純な勾配法を使用したパス平滑化プログラムです。

MATLABよる経路平滑化(Path Smoothing)プログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140510/1399694663

##Dynamic Window Approach
Dynamic Window Approachを使用したLocal Path Planningプログラムです。

Dynamic Window ApproachのMATLAB サンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140624/1403618922

#SLAM
Simultaneous Localization And Mapping(SLAM)のサンプルコード用フォルダです。

## ICP
Iterative Closest Point (ICP) Algorithmを使用したScanデータに基づく相対移動量推定プログラムです。

ICPアルゴリズムを利用したSLAM用MATLABサンプルプログラム - MY ENIGMA http://d.hatena.ne.jp/meison_amsl/20140617/1402971928


