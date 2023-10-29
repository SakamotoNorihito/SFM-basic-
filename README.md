# はじめに
本プログラムは、Social Force Modelと呼ばれる歩行者の移動モデルを用いて、群衆の避難行動をシミュレーションすることを目的に作成されました。Social Force Modelは、群衆シミュレーションを行うために[HelbingとMolnár(1995)](https://journals.aps.org/pre/pdf/10.1103/PhysRevE.51.4282?casa_token=OjqzH1SsARQAAAAA%3AaXoqzGndMGMGdjRnVV0xuwH9tfE3L8crWieBiWEsn6hMq5GAg85ZhVg0YQXM8CzRlbxKM_8heI5jvw)によって導入された歩行者の移動モデルとなっています。

# 動作環境
OS:Microsoft Windows 11  
実行環境:VisualStudio2022  
C++のバージョン:C++20

# 使用方法
## パラメータの設定
プログラム内のAgent.cppファイルの中にあるコンストラクタ（Agent::Agent()）からエージェントのパラメータを設定します。

```
mass = 80;          //エージェントの質量(kg)
radius = 0.25;		  //エージェント半径(m)
desiredSpeed = 1;	  //希望速さ(m/s)
```

同様に、プログラム内のmain.cppファイル内から部屋の形状及びシミュレーション条件を設定します。

```
//部屋の形状
const double room_size_x = 10;
const double room_size_y = 10;
const double width_exit = 1;

//シミュレーション条件
const int N_sample = 10;             //サンプル数
const int N_evacuee = 100;          //初期避難者数
const double stepTime = 0.005;      //時間幅
const int N_step = 18200;           //シミュレーションステップ数
```
