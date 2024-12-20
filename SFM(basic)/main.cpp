#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include<time.h>
#include"Vector2d_math.h"
#include "Agent.h"
#include"Room.h"
#include <random>
using namespace std;

//部屋の形状
const double room_size_x = 10;
const double room_size_y = 10;
const double width_exit = 1;

//シミュレーション条件
const int N_sample = 1;             //サンプル数
const int N_guide = 1;              //初期誘導者数
const int N_evacuee = 100;          //初期避難者数
const double stepTime = 0.005;      //時間幅（初期値：0.005[s]）
const double coeff_dataOutPut = 1;   //データ出力の時間幅を決定する係数（1…1/1(= 1)秒、10…1/10(= 0.1)秒、100…1/100(= 0.01)秒）
const int N_step = 60200;           //シミュレーションステップ数(300[s])
//避難時間 = 時間幅 × シミュレーションステップ数

int countN_inR_ind(const vector<Agent>& guide, const vector<Agent>& evacuee);
int countEscapeCompleteNumber(const int N_initial, vector<Agent>& agents);
double calculateAverage(const vector<double>& data);
double calculateStandardDeviation(const vector<double>& data);
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
    vector<double> recordGuideEscapeTime(N_sample);
    vector<double> recordLastEvacueeEscapeTime(N_sample);    

    for (int N = 0; N < N_sample; ++N) 
    {
        bool recordGuideEscapeTimeFlag = true;      //最後の誘導者が避難完了した時間を記録するためのフラグ
        bool recordLastEvacueeEscapeTimeFlag = true;     //最後の避難者が避難完了した時間を記録するためのフラグ

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

        vector<Agent> guide(N_guide);
        vector<Agent> evacuee(N_evacuee);

        double R_ind = guide[0].getR_ind();
        double R_vis = evacuee[0].getR_vis();
        double R_exitSign = evacuee[0].getR_exitSign();
        double radius = evacuee[0].getRadius();

        cout << "部屋のサイズ：x" << "," << "部屋のサイズ：y" << "," << "出口幅：w" << ","
            << "エージェント半径" << "," << "誘導半径" << "," << "視界半径" << "," << "避難口誘導灯の音声半径" << ","
            << "誘導者数" << "," << "避難者数" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << ","
            << radius << "," << R_ind << "," << R_vis << "," << R_exitSign << ","
            << N_guide << "," << N_evacuee << "\n";

        ofs << "部屋のサイズ：x" << "," << "部屋のサイズ：y" << "," << "出口幅：w" << ","
            << "エージェント半径" << "," << "誘導半径" << "," << "視界半径" << "," << "避難口誘導灯の音声半径" << ","
            << "誘導者数" << "," << "避難者数" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << ","
            << radius << "," << R_ind << "," << R_vis << "," << R_exitSign << ","
            << N_guide << "," << N_evacuee << "\n";

        cout << "時間" << "," << "避難完了人数" << ",";
        ofs << "時間" << "," << "避難完了人数" << ",";

        for (int i = 0; i < N_guide; ++i)
        {
            cout << "誘導者" << i + 1 << "のx座標" << "," << "誘導者" << i + 1 << "のy座標" << ","
                << "誘導者" << i + 1 << "のe_x" << "," << "誘導者" << i + 1 << "のe_y" << ","
                << "誘導者" << i + 1 << "のdesiredSpeed" << ","
                << "誘導者" << i + 1 << "のv_x" << "," << "誘導者" << i + 1 << "のv_y" << ","
                << "誘導者" << i + 1 << "の実速度の大きさv" << ","
                << "誘導者" << i + 1 << "のf_driv_x" << "," << "誘導者" << i + 1 << "のf_driv_y" << ","
                << "誘導者" << i + 1 << "のf_ig_x" << "," << "誘導者" << i + 1 << "のf_ig_y" << ","
                << "誘導者" << i + 1 << "のf_ij_x" << "," << "誘導者" << i + 1 << "のf_ij_y" << ","
                << "誘導者" << i + 1 << "のf_iw_x" << "," << "誘導者" << i + 1 << "のf_iw_y" << ","
                << "誘導者" << i + 1 << "のN_inR_ind" << ",";

            ofs << "誘導者" << i + 1 << "のx座標" << "," << "誘導者" << i + 1 << "のy座標" << ","
                << "誘導者" << i + 1 << "のe_x" << "," << "誘導者" << i + 1 << "のe_y" << ","
                << "誘導者" << i + 1 << "のdesiredSpeed" << ","
                << "誘導者" << i + 1 << "のv_x" << "," << "誘導者" << i + 1 << "のv_y" << ","
                << "誘導者" << i + 1 << "の実速度の大きさv" << ","
                << "誘導者" << i + 1 << "のf_driv_x" << "," << "誘導者" << i + 1 << "のf_driv_y" << ","
                << "誘導者" << i + 1 << "のf_ig_x" << "," << "誘導者" << i + 1 << "のf_ig_y" << ","
                << "誘導者" << i + 1 << "のf_ij_x" << "," << "誘導者" << i + 1 << "のf_ij_y" << ","
                << "誘導者" << i + 1 << "のf_iw_x" << "," << "誘導者" << i + 1 << "のf_iw_y" << ","
                << "誘導者" << i + 1 << "のN_inR_ind" << ",";
        }

        for (int i = 0; i < N_evacuee; ++i)
        {           
            cout << "避難者" << i + 1 << "のx座標" << "," << "避難者" << i + 1 << "のy座標" << ","
                << "避難者" << i + 1 << "のe_x" << "," << "避難者" << i + 1 << "のe_y" << ","
                << "避難者" << i + 1 << "のdesiredSpeed" << ","
                << "避難者" << i + 1 << "のv_x" << "," << "避難者" << i + 1 << "のv_y" << ","
                << "避難者" << i + 1 << "の実速度の大きさv" << ","
                << "避難者" << i + 1 << "のf_driv_x" << "," << "避難者" << i + 1 << "のf_driv_y" << ","
                << "避難者" << i + 1 << "のf_ig_x" << "," << "避難者" << i + 1 << "のf_ig_y" << ","
                << "避難者" << i + 1 << "のf_ij_x" << "," << "避難者" << i + 1 << "のf_ij_y" << ","
                << "避難者" << i + 1 << "のf_iw_x" << "," << "避難者" << i + 1 << "のf_iw_y" << ",";

            ofs << "避難者" << i + 1 << "のx座標" << "," << "避難者" << i + 1 << "のy座標" << ","
                << "避難者" << i + 1 << "のe_x" << "," << "避難者" << i + 1 << "のe_y" << ","
                << "避難者" << i + 1 << "のdesiredSpeed" << ","
                << "避難者" << i + 1 << "のv_x" << "," << "避難者" << i + 1 << "のv_y" << ","
                << "避難者" << i + 1 << "の実速度の大きさv" << ","
                << "避難者" << i + 1 << "のf_driv_x" << "," << "避難者" << i + 1 << "のf_driv_y" << ","
                << "避難者" << i + 1 << "のf_ig_x" << "," << "避難者" << i + 1 << "のf_ig_y" << ","
                << "避難者" << i + 1 << "のf_ij_x" << "," << "避難者" << i + 1 << "のf_ij_y" << ","
                << "避難者" << i + 1 << "のf_iw_x" << "," << "避難者" << i + 1 << "のf_iw_y" << ",";
        }       
        cout << "\n";
        ofs << "\n";

        setInitialPosition_g(room, guide);
        setInitialPosition_e(room, guide, evacuee);

        for (int n = 0; n < N_step; ++n) 
        {
            removeAgent(guide, room);
            removeAgent(evacuee, room);

            int N_escapeCurrentGuide = guide.size();
            int N_escapeCurrentEvacuee = evacuee.size();

            for (int i = 0; i < N_escapeCurrentGuide; ++i)
            {
                guide[i].move_g(guide, evacuee, room, stepTime);
            }

            for (int i = 0; i < N_escapeCurrentEvacuee; ++i)
            {
                evacuee[i].move_e(evacuee, guide, room, stepTime);
            }

            if (n % (int)(1 / (stepTime * coeff_dataOutPut)) == 0) //指定時間ごとに避難者の位置を記録する
            {
                //避難完了者数の数え上げ
                int N_escapeCompleteEvacuee = countEscapeCompleteNumber(N_evacuee, evacuee);
                //避難完了者数の記録（添え字１…サンプル数、添え字２…秒数）
                recordEscapeCompleteNumber[N][n * stepTime] = N_escapeCompleteEvacuee;

                //誘導者が部屋を脱出した時刻を記録
                if (recordGuideEscapeTimeFlag == true && guide.size() == 0)
                {
                    recordGuideEscapeTime[N] = n * stepTime;
                    recordGuideEscapeTimeFlag = false;
                }

                //最後の避難者が避難完了した時間を記録
                if (recordLastEvacueeEscapeTimeFlag == true && N_escapeCompleteEvacuee == N_evacuee)
                {
                    recordLastEvacueeEscapeTime[N] = n * stepTime;
                    recordLastEvacueeEscapeTimeFlag = false;
                }

                cout << n * stepTime << "," << N_escapeCompleteEvacuee << ",";
                ofs << n * stepTime << "," << N_escapeCompleteEvacuee << ",";

                //現在避難している誘導者数と初期誘導者数が一致しているとき
                if (N_escapeCurrentGuide == N_guide)
                {
                    for (int i = 0; i < N_escapeCurrentGuide; ++i)
                    {
                        cout << guide[i].getPosition().x << "," << guide[i].getPosition().y << ","
                            << guide[i].getDesiredDirection().x << "," << guide[i].getDesiredDirection().y << ","
                            << guide[i].getDesiredSpeed() << ","
                            << guide[i].getVelocity().x << "," << guide[i].getVelocity().y << ","
                            << sqrt((guide[i].getVelocity().x * guide[i].getVelocity().x) + (guide[i].getVelocity().y * guide[i].getVelocity().y)) << ","
                            << guide[i].getF_driv().x << "," << guide[i].getF_driv().y << ","
                            << guide[i].getF_ig().x << "," << guide[i].getF_ig().y << ","
                            << guide[i].getF_ij().x << "," << guide[i].getF_ij().y << ","
                            << guide[i].getF_iw().x << "," << guide[i].getF_iw().y << ","
                            << countN_inR_ind(guide, evacuee) << ",";

                        ofs << guide[i].getPosition().x << "," << guide[i].getPosition().y << ","
                            << guide[i].getDesiredDirection().x << "," << guide[i].getDesiredDirection().y << ","
                            << guide[i].getDesiredSpeed() << ","
                            << guide[i].getVelocity().x << "," << guide[i].getVelocity().y << ","
                            << sqrt((guide[i].getVelocity().x * guide[i].getVelocity().x) + (guide[i].getVelocity().y * guide[i].getVelocity().y)) << ","
                            << guide[i].getF_driv().x << "," << guide[i].getF_driv().y << ","
                            << guide[i].getF_ig().x << "," << guide[i].getF_ig().y << ","
                            << guide[i].getF_ij().x << "," << guide[i].getF_ij().y << ","
                            << guide[i].getF_iw().x << "," << guide[i].getF_iw().y << ","
                            << countN_inR_ind(guide, evacuee) << ",";
                    }
                }
                
                //避難を完了した誘導者が出てきたとき
                else
                {
                    int N_escapeCompleteGuide = countEscapeCompleteNumber(N_guide, guide);

                    for (int i = 0; i < N_escapeCurrentGuide; ++i)
                    {
                        cout << guide[i].getPosition().x << "," << guide[i].getPosition().y << ","
                            << guide[i].getDesiredDirection().x << "," << guide[i].getDesiredDirection().y << ","
                            << guide[i].getDesiredSpeed() << ","
                            << guide[i].getVelocity().x << "," << guide[i].getVelocity().y << ","
                            << sqrt((guide[i].getVelocity().x * guide[i].getVelocity().x) + (guide[i].getVelocity().y * guide[i].getVelocity().y)) << ","
                            << guide[i].getF_driv().x << "," << guide[i].getF_driv().y << ","
                            << guide[i].getF_ig().x << "," << guide[i].getF_ig().y << ","
                            << guide[i].getF_ij().x << "," << guide[i].getF_ij().y << ","
                            << guide[i].getF_iw().x << "," << guide[i].getF_iw().y << ","
                            << countN_inR_ind(guide, evacuee) << ",";

                        ofs << guide[i].getPosition().x << "," << guide[i].getPosition().y << ","
                            << guide[i].getDesiredDirection().x << "," << guide[i].getDesiredDirection().y << ","
                            << guide[i].getDesiredSpeed() << ","
                            << guide[i].getVelocity().x << "," << guide[i].getVelocity().y << ","
                            << sqrt((guide[i].getVelocity().x * guide[i].getVelocity().x) + (guide[i].getVelocity().y * guide[i].getVelocity().y)) << ","
                            << guide[i].getF_driv().x << "," << guide[i].getF_driv().y << ","
                            << guide[i].getF_ig().x << "," << guide[i].getF_ig().y << ","
                            << guide[i].getF_ij().x << "," << guide[i].getF_ij().y << ","
                            << guide[i].getF_iw().x << "," << guide[i].getF_iw().y << ","
                            << countN_inR_ind(guide, evacuee) << ",";
                    }

                    for (int i = 0; i < N_escapeCompleteGuide; ++i)
                    {
                        cout << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << ","
                            << "" << "," << "" << ","
                            << "" << ","
                            << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << ",";

                        ofs << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << ","
                            << "" << "," << "" << ","
                            << "" << ","
                            << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << "," << "" << ","
                            << "" << ",";
                    }
                }                

                for (int i = 0; i < N_escapeCurrentEvacuee; ++i)
                {
                    cout << evacuee[i].getPosition().x << "," << evacuee[i].getPosition().y << ","
                        << evacuee[i].getDesiredDirection().x << "," << evacuee[i].getDesiredDirection().y << ","
                        << evacuee[i].getDesiredSpeed() << ","
                        << evacuee[i].getVelocity().x << "," << evacuee[i].getVelocity().y << ","
                        << sqrt((evacuee[i].getVelocity().x * evacuee[i].getVelocity().x) + (evacuee[i].getVelocity().y * evacuee[i].getVelocity().y)) << ","
                        << evacuee[i].getF_driv().x << "," << evacuee[i].getF_driv().y << ","
                        << evacuee[i].getF_ig().x << "," << evacuee[i].getF_ig().y << ","
                        << evacuee[i].getF_ij().x << "," << evacuee[i].getF_ij().y << ","
                        << evacuee[i].getF_iw().x << "," << evacuee[i].getF_iw().y << ",";

                    ofs << evacuee[i].getPosition().x << "," << evacuee[i].getPosition().y << ","
                        << evacuee[i].getDesiredDirection().x << "," << evacuee[i].getDesiredDirection().y << ","
                        << evacuee[i].getDesiredSpeed() << ","
                        << evacuee[i].getVelocity().x << "," << evacuee[i].getVelocity().y << ","
                        << sqrt((evacuee[i].getVelocity().x * evacuee[i].getVelocity().x) + (evacuee[i].getVelocity().y * evacuee[i].getVelocity().y)) << ","
                        << evacuee[i].getF_driv().x << "," << evacuee[i].getF_driv().y << ","
                        << evacuee[i].getF_ig().x << "," << evacuee[i].getF_ig().y << ","
                        << evacuee[i].getF_ij().x << "," << evacuee[i].getF_ij().y << ","
                        << evacuee[i].getF_iw().x << "," << evacuee[i].getF_iw().y << ",";
                }

                cout << "\n";
                ofs << "\n";
            }
        }

        //シミュレーションが終了しても誘導者が避難完了できなかった場合、その時点での時間を記録する
        if (recordGuideEscapeTimeFlag == true)
        {
            recordGuideEscapeTime[N] = N_step * stepTime - 1.0;
        }

        //シミュレーションが終了しても全避難者が避難完了できなかった場合、その時点での時間を記録する
        if (recordLastEvacueeEscapeTimeFlag == true)
        {
            recordLastEvacueeEscapeTime[N] = N_step * stepTime - 1.0;
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

    double ave_guide_time = calculateAverage(recordGuideEscapeTime);
    double sd_guide_time = calculateStandardDeviation(recordGuideEscapeTime);
    double ave_time = calculateAverage(recordLastEvacueeEscapeTime);
    double sd_time = calculateStandardDeviation(recordLastEvacueeEscapeTime);

    int N_sample = recordEscapeCompleteNumber.size();
    int N_record = recordEscapeCompleteNumber.at(0).size();

    cout << "サンプル数" << "," << N_sample << "\n" << "時間" << "," << "平均避難者数" << "," << "標準偏差" << "\n";
    ofs << "サンプル数" << "," << N_sample << "\n" << "時間" << "," << "平均避難者数" << "," << "標準偏差" << "\n";

    for (int i = 0; i < N_record; ++i)
    {
        cout << i << "," << ave[i] << "," << sd[i] << "\n";
        ofs << i << "," << ave[i] << "," << sd[i] << "\n";
    }

    cout << "\n" << "平均避難時間" << "," << "標準偏差" << "\n" << ave_time << "," << sd_time << "\n";
    ofs << "\n" << "平均避難時間" << "," << "標準偏差" << "\n" << ave_time << "," << sd_time << "\n";

    for (int i = 0; i < recordLastEvacueeEscapeTime.size(); ++i)
    {
        cout << recordLastEvacueeEscapeTime[i] << "\n";
        ofs << recordLastEvacueeEscapeTime[i] << "\n";
    }

    cout << "\n" << "誘導者の平均避難時間" << "," << "誘導者の標準偏差" << "\n" << ave_guide_time << "," << sd_guide_time << "\n";
    ofs << "\n" << "誘導者の平均避難時間" << "," << "誘導者の標準偏差" << "\n" << ave_guide_time << "," << sd_guide_time << "\n";

    for (int i = 0; i < recordGuideEscapeTime.size(); ++i)
    {
        cout << recordGuideEscapeTime[i] << "\n";
        ofs << recordGuideEscapeTime[i] << "\n";

    }

    ofs.close();
    
    clock_t end = clock();
    cout << "処理時間：" << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";

    return 0;
}

int countN_inR_ind(const vector<Agent>& guide, const vector<Agent>& evacuee)
{
    int N_inR_ind = 0;

    int N_guide = guide.size();
    int N_evacuee = evacuee.size();
    double R_ind = guide[0].getR_ind();
    double R_vis = evacuee[0].getR_vis();

    for (int i = 0; i < N_guide; ++i)
    {
        for (int j = 0; j < N_evacuee; ++j)
        {
            double d_Li = distance(guide[i].getPosition(), evacuee[j].getPosition());

            if (d_Li <= R_ind || d_Li <= R_vis)
            {
                N_inR_ind++;
            }
        }
    }

    return N_inR_ind;
}

int countEscapeCompleteNumber(const int N_initialNumber, vector<Agent>& agents)
{
    int N_escapeCurrent = agents.size();

    return N_initialNumber - N_escapeCurrent;
}

double calculateAverage(const vector<double>& data)
{
    int N_sample = data.size();
    double temp = 0;

    for (int i = 0; i < N_sample; ++i)
    {
        temp += data[i];
    }

    return temp / N_sample;    
}

double calculateStandardDeviation(const vector<double>& data)
{
    int N_sample = data.size();
    double temp = 0;

    double ave = calculateAverage(data);

    for (int i = 0; i < N_sample; ++i)
    {
        temp += (data[i] - ave) * (data[i] - ave);
    }

    return sqrt(temp / N_sample);
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
}