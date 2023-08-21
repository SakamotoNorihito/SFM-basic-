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
const int N_sample = 1;             //�T���v����
const int N_evacuee = 100;          //�������Ґ�
const double stepTime = 0.005;      //���ԕ�
const int N_step = 18200;           //�V�~�����[�V�����X�e�b�v��

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
        //�e�T���v�����Ƀf�[�^���o��
        std::string fname("simulation(basic)" + toString(N) + ".csv");
        std::ofstream ofs(fname.c_str());

        if (!ofs)
        {
            cout << "�t�@�C�����J���܂���ł����B" << endl;
            return 0;
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
    cout << "�������ԁF" << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";

    return 0;
}

int countEscapeCompleteNumber(const int N_initialNumber, vector<Agent>& agents) 
{
    int N_escapeCurrent = agents.size();

    return N_initialNumber - N_escapeCurrent;
}

//C++ this�|�C���^�y�I�u�W�F�N�g���g�������B���ꂽ�|�C���^�z
//https://monozukuri-c.com/langcpp-this-pointer/

//C++ �̊�b : �A�N�Z�X����
//http://www.s-cradle.com/developer/sophiaframework/tutorial/Cpp/access.html

//csv�t�@�C���J���Ȃ��o�O����������