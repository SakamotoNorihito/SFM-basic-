#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include<time.h>
#include"Vector2d_math.h"
#include "Agent.h"
#include"Room.h"
using namespace std;

//部屋の形状
const double room_size_x = 10;
const double room_size_y = 10;
const double width_exit = 1;

//シミュレーション条件
const int N_sample = 1;             //サンプル数
const int N_evacuee = 100;          //初期避難者数
const double stepTime = 0.005;      //時間幅
const int N_step = 18200;           //シミュレーションステップ数

int countEscapeCompleteNumber(const int N_initial, vector<Agent>& agents);

template<class T>
std::string toString(const T& ct)
{
    std::ostringstream os;
    os << ct;
    return os.str();
}

int main()
{
    clock_t start = clock();

    for (int N = 0; N < N_sample; ++N) 
    {
        //各サンプル毎にデータを出力
        std::string fname("simulation(basic)" + toString(N) + ".csv");
        std::ofstream ofs(fname.c_str());

        if (!ofs)
        {
            cout << "ファイルが開けませんでした。" << endl;
            return 0;
        }

        Room room;
        room.setRoom_size_x(room_size_x);
        room.setRoom_size_y(room_size_y);
        room.setWidth_exit(width_exit);        

        cout << "部屋のサイズ：x" << "," << "部屋のサイズ：y" << "," << "出口幅：w" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << "\n";
        
        ofs << "部屋のサイズ：x" << "," << "部屋のサイズ：y" << "," << "出口幅：w" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << "\n";

        cout << "時間" << ","
            << "避難完了人数" << ",";

        ofs << "時間" << ","
            << "避難完了人数" << ",";

        for (int i = 0; i < N_evacuee; ++i)
        {           
            cout << "避難者" << i + 1 << "のx座標" << "," << "避難者" << i + 1 << "のy座標" << ",";
            ofs << "避難者" << i + 1 << "のx座標" << "," << "避難者" << i + 1 << "のy座標" << ",";
        }       
        cout << "\n";
        ofs << "\n";

        vector<Agent> evacuee(N_evacuee);
        setInitialPosition(evacuee, room);

        for (int n = 0; n < N_step; ++n) 
        {
            removeAgent(evacuee, room);

            int N_escapeCurrent = evacuee.size();
                        
            for (int i = 0; i < N_escapeCurrent; ++i) 
            {
                evacuee[i].move(evacuee, room, stepTime);
            }                

            if (n % (int)(1 / stepTime) == 0) //1秒ごとに避難者の位置を記録する
            {
                int N_escapeComplete = countEscapeCompleteNumber(N_evacuee, evacuee);

                cout << n * stepTime << "," << N_escapeComplete << ",";
                ofs << n * stepTime << "," << N_escapeComplete << ",";

                for (int i = 0; i < N_escapeCurrent; ++i) 
                {
                    cout << evacuee[i].getPosition().x << "," << evacuee[i].getPosition().y << ",";
                    ofs << evacuee[i].getPosition().x << "," << evacuee[i].getPosition().y << ",";
                }

                cout << "\n";
                ofs << "\n";
            }            
        }
        

        ofs.close();
    }
    
    clock_t end = clock();
    cout << "処理時間：" << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";

    return 0;
}

int countEscapeCompleteNumber(const int N_initialNumber, vector<Agent>& agents) 
{
    int N_escapeCurrent = agents.size();

    return N_initialNumber - N_escapeCurrent;
}

//C++ thisポインタ【オブジェクト自身を示す隠されたポインタ】
//https://monozukuri-c.com/langcpp-this-pointer/

//C++ の基礎 : アクセス制限
//http://www.s-cradle.com/developer/sophiaframework/tutorial/Cpp/access.html

//csvファイル開けないバグを解決する