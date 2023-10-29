# はじめに
本プログラムは、Social Force Modelと呼ばれる歩行者の移動モデルを用いて、群衆の避難行動をシミュレーションすることを目的に作成されました。Social Force Modelは、群衆シミュレーションを行うために[HelbingとMolnár(1995)](https://journals.aps.org/pre/pdf/10.1103/PhysRevE.51.4282?casa_token=OjqzH1SsARQAAAAA%3AaXoqzGndMGMGdjRnVV0xuwH9tfE3L8crWieBiWEsn6hMq5GAg85ZhVg0YQXM8CzRlbxKM_8heI5jvw)によって導入された歩行者の移動モデルとなっています。

本プログラムではさらに、masterブランチでは取り扱うことができなかった「避難誘導」を取り扱うことができるようプログラムの拡張を行いました。拡張にあたっては、[Leiら(2014)](https://www.sciencedirect.com/science/article/pii/S0378437113011904?casa_token=wtXKcXoswTgAAAAA:awG8jGQGKglefYlPcXq1KVWBLmKcbpZYb2-1CGpcfp3SyqPWtJMh75rCtrTUxlJNTNeV_NyIZA)と[Yangら(2016)](https://www.sciencedirect.com/science/article/pii/S0378437115006755?casa_token=ZLFRPx-iwRcAAAAA:KornCFvE3JvD6WpKjWGn3tRHwBGzoGwjQQJUdb1mYy-BXrzBQzxwIwiaDXqh0lDwZdKyaXlxPQ)の研究を参考にしました。

# 動作環境
OS:Microsoft Windows 11  
実行環境:VisualStudio2022  
C++のバージョン:C++20

# 使用方法
## パラメータの設定
プログラム内のAgent.cppファイルの中にあるコンストラクタ（Agent::Agent()）からエージェントのパラメータを設定します。  
ここでは、masterブランチのパラメータに加えて  
*・誘導者の誘導半径*  
*・エージェントの視界半径*  
をそれぞれ設定できるようにしています。

```
mass = 80;			//エージェントの質量(kg)
radius = 0.25;		//エージェント半径(m)
desiredSpeed = 1;	//希望速さ(m/s)
R_ind = 1;			//誘導者の誘導半径(m)
R_vis = 3;			//エージェントの視界半径(m)
```

同様に、プログラム内のmain.cppファイル内から部屋の形状及びシミュレーション条件を設定します。

```
//部屋の形状
const double room_size_x = 10;  //部屋のx（横）方向のサイズ(m)
const double room_size_y = 10;  //部屋のy（縦）方向のサイズ(m)
const double width_exit = 1;    //出口幅のサイズ(m)

//シミュレーション条件
const int N_sample = 100;             //サンプル数
const int N_guide = 1;              //初期誘導者数
const int N_evacuee = 25;          //初期避難者数
const double stepTime = 0.005;      //時間幅
const int N_step = 18200;           //シミュレーションステップ数
```

## プログラムの実行
プログラムを実行すると、SFM(basic)ファイル内にシミュレーション結果を格納したCSVファイルが生成されます。各ファイルが表すデータは次の通りです。

**①simulation(basic)#.csv**  
冒頭２行には各種シミュレーション条件が、３行以降は1秒毎の避難完了人数と各避難者の位置座標が書き込まれたCSVデータです。#にはサンプル番号が入ります。
避難の様子を描画する際には、evacuation-drawingリポジトリ内の避難誘導用ブランチにある[避難_描画プログラム.py](https://github.com/SakamotoNorihito/evacuation-drawing.git)を利用してください。

**②simulation(basic)_statisticalAnalysis.csv**  
①で出力されたCSVデータの避難完了人数について、各時刻ごとの平均及び標準偏差を格納したCSVデータです。

# 作成者
坂本矩仁
