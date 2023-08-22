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

//�����̌`��
const double room_size_x = 10;
const double room_size_y = 10;
const double width_exit = 1;

//�V�~�����[�V��������
const int N_sample = 15;             //�T���v����
const int N_evacuee = 100;          //�������Ґ�
const double stepTime = 0.005;      //���ԕ�
const int N_step = 18200;           //�V�~�����[�V�����X�e�b�v��

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

    //int�^�ɃL���X�g���ׂ��H
    vector<vector<int>> recordEscapeCompleteNumber(N_sample, vector<int>(N_step * stepTime));

    for (int N = 0; N < N_sample; ++N) 
    {
        //�e�T���v�����Ƀf�[�^���o��
        std::string fname("simulation(basic)" + toString(N) + ".csv");
        std::ofstream ofs(fname.c_str());

        if (!ofs)
        {
            cout << "�t�@�C�����J���܂���ł����B" << endl;
            return 1;
        }

        Room room;
        room.setRoom_size_x(room_size_x);
        room.setRoom_size_y(room_size_y);
        room.setWidth_exit(width_exit);        

        cout << "�����̃T�C�Y�Fx" << "," << "�����̃T�C�Y�Fy" << "," << "�o�����Fw" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << "\n";
        
        ofs << "�����̃T�C�Y�Fx" << "," << "�����̃T�C�Y�Fy" << "," << "�o�����Fw" << "\n"
            << room.getRoom_size_x() << "," << room.getRoom_size_y() << "," << room.getWidth_exit() << "\n";

        cout << "����" << ","
            << "�����l��" << ",";

        ofs << "����" << ","
            << "�����l��" << ",";

        for (int i = 0; i < N_evacuee; ++i)
        {           
            cout << "����" << i + 1 << "��x���W" << "," << "����" << i + 1 << "��y���W" << ",";
            ofs << "����" << i + 1 << "��x���W" << "," << "����" << i + 1 << "��y���W" << ",";
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

            if (n % (int)(1 / stepTime) == 0) //1�b���Ƃɔ��҂̈ʒu���L�^����
            {
                int N_escapeComplete = countEscapeCompleteNumber(N_evacuee, evacuee);
                recordEscapeCompleteNumber[N][n * stepTime] = N_escapeComplete;

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

    //���v����
    std::string fname("statistics.csv");
    std::ofstream ofs(fname);

    if (!ofs)
    {
        cout << "�t�@�C�����J���܂���ł����B" << endl;
        return 1;
    }

    vector<double> ave = calculateAverage(recordEscapeCompleteNumber);
    vector<double> sd = calculateStandardDeviation(recordEscapeCompleteNumber);

    int N_record = ave.size();

    cout << "����" << "," << "���ϔ��Ґ�" << "," << "�W���΍�" << "\n";
    ofs << "����" << "," << "���ϔ��Ґ�" << "," << "�W���΍�" << "\n";

    for (int i = 0; i < N_record; ++i)
    {
        cout << i << "," << ave[i] << "," << sd[i] << "\n";
        ofs << i << "," << ave[i] << "," << sd[i] << "\n";
    }

    ofs.close();
    
    clock_t end = clock();
    cout << "�������ԁF" << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";

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

    //�Q�l
    //https://daeudaeu.com/c-statistics/
}