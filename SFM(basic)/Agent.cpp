#include<vector>
#include<random>
#include"Agent.h"
using namespace std;

//���w�萔
const double PI = 3.14159;	//�~����

Agent::Agent() 
{
	mass = 80;			//�G�[�W�F���g�̎���(kg)
	radius = 0.25;		//�G�[�W�F���g���a(m)
	desiredSpeed = 1;	//��]����(m/s)
	R_ind = 0;			//�U���҂̗U�����a(m)
	R_vis = 0;			//�G�[�W�F���g�̎��E���a(m)

	position = Vector2d(0, 0);
	velocity = Vector2d(0, 0);
	desiredDirection = Vector2d(0, 0);
}

void Agent::setMass(const double m)
{
	this->mass = m;
}

void Agent::setRadius(const double r)
{
	this->radius = r;
}

void Agent::setDesiredSpeed(const double v_des)
{
	this->desiredSpeed = v_des;
}

void Agent::setR_ind(const double R_induction)
{
	this->R_ind = R_induction;
}

void Agent::setR_vis(const double R_visibility)
{
	this->R_vis = R_visibility;
}

void Agent::setPosition(const Vector2d p) 
{
	this->position.x = p.x;
	this->position.y = p.y;
}

void Agent::setVelocity(const Vector2d v) 
{
	this->velocity.x = v.x;
	this->velocity.y = v.y;
}

void Agent::setDesiredDirection(const Vector2d e) 
{
	this->desiredDirection.x = e.x;
	this->desiredDirection.y = e.y;
}

Vector2d Agent::drivingForce(const Room roomData) 
{
	const double reactionTime = 0.5;											//��������(s)
	const Vector2d target = Vector2d(roomData.getRoom_size_x() + radius, 0);	//�ړI�n
	Vector2d f_driv;

	desiredDirection = unitVector(position, target);
	f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);

	return f_driv;
}

//guide�pdrivingForce
Vector2d Agent::drivingForce_g(const Room room)
{
	const double reactionTime = 0.5;										//��������(s)
	const Vector2d target = Vector2d(room.getRoom_size_x() + radius, 0);	//�ړI�n
	Vector2d f_driv;

	desiredDirection = unitVector(position, target);
	f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);

	return f_driv;
}

//evacuee�pdrivingForce
Vector2d Agent::drivingForce_e(const Room room, const std::vector<Agent>& guide, const std::vector<Agent>& evacuee)
{
	const double reactionTime = 0.5;										//��������(s)
	const Vector2d target = Vector2d(room.getRoom_size_x() + radius, 0);	//�ړI�n
	Vector2d f_driv;
	
	double d_it = distance(position, target);	//���g�ƖړI�n�܂ł̋���

	//�ړI�n�����E�ɓ����Ă���Ƃ�
	if (d_it < R_vis)
	{
		desiredDirection = unitVector(position, target);
		f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);
	}

	//�ړI�n�����E�ɓ����Ă��Ȃ��Ƃ�
	else
	{
		int N_guide = guide.size();
		vector<double> d_ig(N_guide);

		//���҂Ɗe�U���҂̋������v�Z
		for (int i = 0; i < N_guide; ++i)
		{
			d_ig[i] = distance(position, guide[i].getPosition());
		}

		//�ł��߂��U���҂܂ł̋����ƗU���Ҕԍ����擾�i�Q�l�Fhttps://zenn.dev/reputeless/books/standard-cpp-for-competitive-programming/viewer/library-algorithm#1.5-%E9%85%8D%E5%88%97%E3%81%AE%E4%B8%AD%E3%81%8B%E3%82%89%E6%9C%80%E5%B0%8F%E3%81%AE%E8%A6%81%E7%B4%A0%E3%81%A8%E3%81%9D%E3%81%AE%E4%BD%8D%E7%BD%AE%E3%82%92%E5%BE%97%E3%82%8B�j
		auto it = min_element(d_ig.begin(), d_ig.end());
		double d_ig_min = *it;									//�U���҂܂ł̋���
		int nearestGuideNumber = distance(d_ig.begin(), it);	//�U���Ҕԍ�
		double R_ind = guide[nearestGuideNumber].getR_ind();	//�U���҂̗U�����a

		//�U���҂̗U�����a���ɂ���Ƃ�
		if (d_ig_min < R_ind)
		{
			double rho = exp(-d_ig_min / (2 * R_ind));
			Vector2d O, e_i;
			Vector2d e_g = unitVector(guide[nearestGuideNumber].getPosition(), target);
			Vector2d n_ig = unitVector(guide[nearestGuideNumber].getPosition(), position);

			e_i = rho * e_g - (1 - rho) * n_ig;
			desiredDirection = unitVector(O, e_i);
			f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);
		}

		//�U���҂̗U�����a�O�ɂ���Ƃ�
		else
		{
			const double lambda = 1;	//���E�ٕ����̋�����\���p�����[�^

			int N_aroundAgent = 0;
			Vector2d O, v_total, v_average;

			int N_guide = guide.size();
			int N_evacuee = evacuee.size();

			//���g�̎��E�͈͓��ɂ���U���҂̐����グ
			for (int i = 0; i < N_guide; ++i)
			{
				double d_ig = distance(position, guide[i].getPosition());

				if (d_ig != 0 && d_ig < R_vis)
				{
					N_aroundAgent++;

					//���E�ٕ����W���̌v�Z
					Vector2d n_gi = unitVector(position, guide[i].getPosition());	//i��g�����̒P�ʃx�N�g��
					double cos_phi = dotProduct(desiredDirection, n_gi);
					double anisotropy_coef = lambda + (1 - lambda) * ((1 + cos_phi) / 2);

					v_total = v_total + anisotropy_coef * guide[i].getVelocity();
				}
			}

			//���g�̎��E�͈͓��ɂ�����҂̐����グ
			for (int i = 0; i < N_evacuee; ++i)
			{
				double d_ij = distance(position, evacuee[i].getPosition());

				if (d_ij != 0 && d_ij < R_vis)
				{
					N_aroundAgent++;

					//���E�ٕ����W���̌v�Z
					Vector2d n_ji = unitVector(position, evacuee[i].getPosition());	//i��j�����̒P�ʃx�N�g��
					double cos_phi = dotProduct(desiredDirection, n_ji);
					double anisotropy_coef = lambda + (1 - lambda) * ((1 + cos_phi) / 2);

					v_total = v_total + anisotropy_coef * evacuee[i].getVelocity();
				}
			}

			//���g�̎��E�͈͓��ɑ��̃G�[�W�F���g�����݂��Ȃ��Ƃ�
			if (N_aroundAgent == 0)
			{
				std::random_device seed_gen;
				std::default_random_engine engine(seed_gen());

				// 0�ȏ�1.0�����̒l�𓙊m���Ŕ���������
				std::uniform_real_distribution<> dist(0, 1.0);

				double theta = 2 * PI * dist(engine);
				Vector2d randomDirection = Vector2d(cos(theta), sin(theta));

				desiredDirection = unitVector(O, randomDirection);
			}

			//���g�̎��E�͈͓��ɑ��̃G�[�W�F���g�����݂���Ƃ�
			else
			{
				//���̃G�[�W�F���g�̑��x�x�N�g���̑��a v_total ����x�N�g���̂Ƃ�
				if (v_total.x == 0 && v_total.y == 0)
				{
					std::random_device seed_gen;
					std::default_random_engine engine(seed_gen());

					// 0�ȏ�1.0�����̒l�𓙊m���Ŕ���������
					std::uniform_real_distribution<> dist(0, 1.0);

					double theta = 2 * PI * dist(engine);
					Vector2d randomDirection = Vector2d(cos(theta), sin(theta));

					desiredDirection = unitVector(O, randomDirection);
				}

				//���̃G�[�W�F���g�̑��x�x�N�g���̑��a v_total ����x�N�g���łȂ��Ƃ�
				else
				{
					v_average = v_total / N_aroundAgent;
					desiredDirection = unitVector(O, v_average);
				}
			}

			f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);
		}
	}

	return f_driv;
}

Vector2d Agent::agentInteractForce(const std::vector<Agent>& agents)
{
	const double A = 2000;					//�S���I������(N)
	const double B = 0.08;					//�S���I��������(m)
	const double k = 120000;				//���薀�C�W��(kg/s^2)
	const double kappa = 240000;			//�̈��k�W��(kg/m�Es)
	const double r_i = radius;				//���g�̔��a(m)
	const int N_agent = agents.size();		//agent�̐l��(�l)

	double r_j = 0;							//agent[j]�̔��a(m)
	double r_ij = 0;						//���g��agent[j]�̔��a�̘a(m)
	double d_ij = 0;						//���g��agent[j]�̒��S�ԋ���(m)
	double v_ji = 0;
	Vector2d n_ij, t_ij, f_soc, f_rep, f_fric, f_ij;

	for (int j = 0; j < N_agent; ++j) 
	{
		d_ij = distance(position, agents[j].getPosition());

		if (d_ij != 0) //�������g�����O
		{
			r_j = agents[j].getRadius();
			r_ij = r_i + r_j;
			n_ij = unitVector(agents[j].getPosition(), position);		//j��i�����̒P�ʃx�N�g��
			t_ij = tangentialVector(n_ij);
			v_ji = dotProduct(agents[j].getVelocity() - velocity, t_ij);

			f_soc = A * exp((r_ij - d_ij) / B) * n_ij;
			f_rep = k * g(r_ij - d_ij) * n_ij;
			f_fric = kappa * g(r_ij - d_ij) * v_ji * t_ij;

			f_ij = f_ij + f_soc + f_rep + f_fric;
		}
	}

	return f_ij;
}

Vector2d Agent::wallInteractForce(Room room) 
{
	const vector<vector<Vector2d>> wallCornerPoint = room.createWall();

	const double A = 2000;							//�S���I������(N)
	const double B = 0.08;							//�S���I��������(m)
	const double k = 120000;						//���薀�C�W��(kg/s^2)
	const double kappa = 240000;					//�̈��k�W��(kg/m�Es)
	const double r_i = radius;						//���g�̔��a(m)
	const int N_wall = wallCornerPoint.size();		//�ǂ̐�(��)

	double d_iw = 0;								//���g�ƕǂƂ̋���(m)
	Vector2d nearestPoint, n_iw, t_iw, f_soc, f_rep, f_fric, f_iw;

	for (int n = 0; n < N_wall; ++n) 
	{
		nearestPoint = getNearestPoint(wallCornerPoint[n][0], wallCornerPoint[n][1], position);
		d_iw = distance(position, nearestPoint);
		n_iw = unitVector(nearestPoint, position);		//w��i�����̒P�ʃx�N�g��
		t_iw = tangentialVector(n_iw);

		f_soc = A * exp((r_i - d_iw) / B) * n_iw;
		f_rep = k * g(r_i - d_iw) * n_iw;
		f_fric = kappa * g(r_i - d_iw) * dotProduct(velocity, t_iw) * t_iw;

		f_iw = f_iw + f_soc + f_rep - f_fric;
	}

	return f_iw;
}

void Agent::move(vector<Agent>& agents, Room roomData, const double stepTime) 
{
	Vector2d acceleration;

	acceleration = drivingForce(roomData) + agentInteractForce(agents) + wallInteractForce(roomData);

	velocity = velocity + stepTime * acceleration;
	position = position + stepTime * velocity;
}

void setInitialPosition(vector<Agent>& agents, const Room roomData) 
{
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	
	const double room_size_x = roomData.getRoom_size_x();
	const double room_size_y = roomData.getRoom_size_y();
	const int N_agent = agents.size();

	int i = 0;	

	//���҂̍��W���点�Ȃ����߂̏���
	do 
	{
		double r = agents[i].getRadius();

		// r �ȏ� room_size_x - r �����̎�������l�����Ŕ���������
		std::uniform_real_distribution<> dist_x(r, room_size_x - r);
		// (-room_size_y / 2) + r �ȏ� (room_size_y / 2) - r �����̎�������l�����Ŕ���������
		std::uniform_real_distribution<> dist_y((-room_size_y / 2) + r, (room_size_y / 2) - r);
		
		agents[i].setPosition(Vector2d(dist_x(engine), dist_y(engine)));
		int j = 0;

		do 
		{
			if (i == j) 
			{
				i++;
				break;
			}

			else 
			{
				double d_ij = distance(agents[i].getPosition(), agents[j].getPosition());
				double r_i = agents[i].getRadius();
				double r_j = agents[j].getRadius();

				if (d_ij < r_i + r_j) 
				{
					break;
				}

				else 
				{
					j++;
				}
			}

		} while (1);

	} while (i < N_agent);
}

void setInitialPosition_g(const Room room, std::vector<Agent>& guide)
{
	const double room_size_x = room.getRoom_size_x();
	const double room_size_y = room.getRoom_size_y();
	const int N_guide = guide.size();

	for (int i = 0; i < N_guide; ++i)
	{
		switch (i)	//�U���Җ������z�u���w�肷��
		{
		case 0:
			guide[i].setPosition(Vector2d(guide[i].getRadius(), 0));	//�����̍��ǒ���
			break;

		case 1:
			break;
		}
	}
}

void setInitialPosition_e(const Room room, const std::vector<Agent>& guide, std::vector<Agent>& evacuee)
{
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());

	const double room_size_x = room.getRoom_size_x();
	const double room_size_y = room.getRoom_size_y();
	const int N_guide = guide.size();
	const int N_evacuee = evacuee.size();	

	int i = 0;
	vector<bool> isOverlap(N_guide);	//���҂Ɗe�U���Җ��̍��W���̔��茋�ʂ��i�[����z��
	bool judge = false;					//�S�Ă̗U���҂Ƃ̍��W���̔��茋�ʂ��i�[����ϐ�

	do
	{
		double r_i = evacuee[i].getRadius();
		// r_i �ȏ� room_size_x - r_i �����̎�������l�����Ŕ���������
		std::uniform_real_distribution<> dist_x(r_i, room_size_x - r_i);
		// (-room_size_y / 2) + r_i �ȏ� (room_size_y / 2) - r_i �����̎�������l�����Ŕ���������
		std::uniform_real_distribution<> dist_y((-room_size_y / 2) + r_i, (room_size_y / 2) - r_i);
		evacuee[i].setPosition(Vector2d(dist_x(engine), dist_y(engine)));

		//���҂ƗU���҂̍��W���̔���
		for (int g = 0; g < N_guide; ++g)
		{
			double r_g = guide[g].getRadius();
			double r_ig = r_i + r_g;
			double d_ig = distance(evacuee[i].getPosition(), guide[g].getPosition());

			//���҂ƗU���҂̑̂��d�Ȃ��Ă���ꍇ
			if (d_ig < r_ig)
			{
				isOverlap[g] = true;
			}

			//���҂ƗU���҂̑̂��d�Ȃ��Ă��Ȃ��ꍇ
			else
			{
				isOverlap[g] = false;
			}
		}

		//�S�Ă̗U���҂ƍ��W�����N�����Ă��Ȃ�������
		judge = all_of(isOverlap.begin(), isOverlap.end(), [](bool t) { return t == false; });

		//�S�Ă̗U���҂ƍ��W�����N�����Ă��Ȃ��Ƃ�
		if (judge)
		{
			int j = 0;

			//���ҊԂō��W�����N�����Ȃ����߂̏���
			do
			{
				if (i == j)
				{
					i++;
					break;
				}

				else
				{
					double r_j = evacuee[j].getRadius();
					double r_ij = r_i + r_j;
					double d_ij = distance(evacuee[i].getPosition(), evacuee[j].getPosition());

					//���ғ��m�ő̂��d�Ȃ��Ă���ꍇ
					if (d_ij < r_ij)
					{
						break;
					}

					//���ғ��m�ő̂��d�Ȃ��Ă��Ȃ��ꍇ
					else
					{
						j++;
					}
				}

			} while (1);
		}
	} while (i < N_evacuee);
}

void removeAgent(std::vector<Agent>& agents, Room roomData) 
{
	double exitPosition_x = roomData.getRoom_size_x();
	erase_if(agents, [exitPosition_x](Agent agent) {return agent.getPosition().x >= exitPosition_x; });
}
