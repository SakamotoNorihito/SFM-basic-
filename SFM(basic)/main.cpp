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
const int N_sample = 5;             //サンプル数
const int N_evacuee = 100;          //初期避難者数
const double stepTime = 0.005;      //時間幅
const int N_step = 18200;           //シミュレーションステップ数

int countEscapeCompleteNumber(const int N_initial, vector<Agent>& agents);
vector<double> calculateAverage(const vector<vector<int>>& data);
vector<double> calculateStandardDeviation(const vector<vector<int>>& data);

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

    //int型にキャストすべき？
    vector<vector<int>> recordEscapeCompleteNumber(N_sample, vector<int>(N_step * stepTime));

    for (int N = 0; N < N_sample; ++N) 
    {
        //各サンプル毎にデータを出力
        std::string fname("simulation(basic)" + toString(N) + ".csv");
        std::ofstream ofs(fname.c_str());

        if (!ofs)
        {
            cout << "ファイルが開けませんでした。" << endl;
            return 1;
        }

        Room room;
        room.setRoom_size_x(room_size_x);
        room.setRoom_size_y(room_size_y);
        room.setWidth_exit(width_exit);

        vector<Agent> evacuee(N_evacuee);

        double radius = evacuee[0].getRadius();

        cout << "部屋のサイズ：x" << "," << "部屋のサイズ：y" << "," << "出口幅：w" << "," << "エージェント半径" << "," << "避難者数" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << "," << radius << "," << N_evacuee << "\n";
        
        ofs << "部屋のサイズ：x" << "," << "部屋のサイズ：y" << "," << "出口幅：w" << "," << "エージェント半径" << "," << "避難者数" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << "," << radius << "," << N_evacuee << "\n";

        cout << "時間" << ","
            << "避難完了人数" << ",";

        ofs << "時間" << ","
            << "避難完了人数" << ",";

        for (int i = 0; i < N_evacuee; ++i)
        {           
            cout << "避難者" << i + 1 << "のx座標" << "," << "避難者" << i + 1 << "のy座標" << ","
                << "避難者" << i + 1 << "のv_x" << "," << "避難者" << i + 1 << "のv_y" << ","
                << "避難者" << i + 1 << "のf_driv_x" << "," << "避難者" << i + 1 << "のf_driv_y" << ","
                << "避難者" << i + 1 << "のf_ij_x" << "," << "避難者" << i + 1 << "のf_ij_y" << ","
                << "避難者" << i + 1 << "のf_iw_x" << "," << "避難者" << i + 1 << "のf_iw_y" << ",";

            ofs << "避難者" << i + 1 << "のx座標" << "," << "避難者" << i + 1 << "のy座標" << ","
                << "避難者" << i + 1 << "のv_x" << "," << "避難者" << i + 1 << "のv_y" << ","
                << "避難者" << i + 1 << "のf_driv_x" << "," << "避難者" << i + 1 << "のf_driv_y" << ","
                << "避難者" << i + 1 << "のf_ij_x" << "," << "避難者" << i + 1 << "のf_ij_y" << ","
                << "避難者" << i + 1 << "のf_iw_x" << "," << "避難者" << i + 1 << "のf_iw_y" << ",";
        }       
        cout << "\n";
        ofs << "\n";

        setInitialPosition(evacuee, room);        

        for (int n = 0; n < N_step; ++n) 
        {
            removeAgent(evacuee, room); //避難完了した避難者を排除

            int N_escapeCurrent = evacuee.size();
                        
            for (int i = 0; i < N_escapeCurrent; ++i) 
            {
                evacuee[i].move(evacuee, room, stepTime);
            }                

            if (n % (int)(1 / stepTime) == 0) //1秒ごとに避難者の位置を記録する
            {
                //避難完了者数を数える
                int N_escapeComplete = countEscapeCompleteNumber(N_evacuee, evacuee);
                //避難完了者数を記録する（添え字１…サンプル数、添え字２…秒数）
                recordEscapeCompleteNumber[N][n * stepTime] = N_escapeComplete;

                cout << n * stepTime << "," << N_escapeComplete << ",";
                ofs << n * stepTime << "," << N_escapeComplete << ",";

                for (int i = 0; i < N_escapeCurrent; ++i) 
                {
                    cout << evacuee[i].getPosition().x << "," << evacuee[i].getPosition().y << ","
                        << evacuee[i].getVelocity().x << "," << evacuee[i].getVelocity().y << ","
                        << evacuee[i].getF_driv().x << "," << evacuee[i].getF_driv().y << ","
                        << evacuee[i].getF_ij().x << "," << evacuee[i].getF_ij().y << ","
                        << evacuee[i].getF_iw().x << "," << evacuee[i].getF_iw().y << ",";

                    ofs << evacuee[i].getPosition().x << "," << evacuee[i].getPosition().y << ","
                        << evacuee[i].getVelocity().x << "," << evacuee[i].getVelocity().y << ","
                        << evacuee[i].getF_driv().x << "," << evacuee[i].getF_driv().y << ","
                        << evacuee[i].getF_ij().x << "," << evacuee[i].getF_ij().y << ","
                        << evacuee[i].getF_iw().x << "," << evacuee[i].getF_iw().y << ",";
                }

                cout << "\n";
                ofs << "\n";
            }
        }
        ofs.close();
    }

    //統計処理用ファイル
    std::string fname("simulation(basic)_statisticalAnalysis.csv");
    std::ofstream ofs(fname);

    if (!ofs)
    {
        cout << "ファイルが開けませんでした。" << endl;
        return 1;
    }

    vector<double> ave = calculateAverage(recordEscapeCompleteNumber);
    vector<double> sd = calculateStandardDeviation(recordEscapeCompleteNumber);

    int N_sample = recordEscapeCompleteNumber.size();
    int N_record = recordEscapeCompleteNumber.at(0).size();

    cout << "サンプル数" << "," << N_sample << "\n" << "時間" << "," << "平均避難者数" << "," << "標準偏差" << "\n";
    ofs << "サンプル数" << "," << N_sample << "\n" << "時間" << "," << "平均避難者数" << "," << "標準偏差" << "\n";

    for (int i = 0; i < N_record; ++i)
    {
        cout << i << "," << ave[i] << "," << sd[i] << "\n";
        ofs << i << "," << ave[i] << "," << sd[i] << "\n";
    }

    ofs.close();
    
    clock_t end = clock();
    cout << "処理時間：" << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";

    return 0;
}

int countEscapeCompleteNumber(const int N_initialNumber, vector<Agent>& agents) 
{
    int N_escapeCurrent = agents.size();

    return N_initialNumber - N_escapeCurrent;
}

vector<double> calculateAverage(const vector<vector<int>>& data)
{
    int N_sample = data.size();
    int N_record = data.at(0).size();
    vector<double> average(N_record);

    for (int i = 0; i < N_record; ++i)
    {
        double temp = 0;

        for (int j = 0; j < N_sample; ++j)
        {
            temp += data[j][i];            
        }

        average[i] = temp / N_sample;
    }

    return average;
}

vector<double> calculateStandardDeviation(const vector<vector<int>>& data)
{
    int N_sample = data.size();
    int N_record = data.at(0).size();
    vector<double> sd(N_record);

    vector<double> ave = calculateAverage(data);

    for (int i = 0; i < N_record; ++i)
    {
        double temp = 0;
        double var = 0;

        for (int j = 0; j < N_sample; ++j)
        {
            temp += (data[j][i] - ave[i]) * (data[j][i] - ave[i]);            
        }

        var = temp / N_sample;
        sd[i] = sqrt(var);
    }

    return sd;

    //参考
    //https://daeudaeu.com/c-statistics/
}