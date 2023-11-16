#include<vector>
#include<random>
#include"Agent.h"
using namespace std;

Agent::Agent() 
{
	mass = 80;			//エージェントの質量(kg)
	radius = 0.25;		//エージェント半径(m)
	desiredSpeed = 1;	//希望速さ(m/s)

	f_driv = Vector2d(0, 0);
	f_ij = Vector2d(0, 0);
	f_iw = Vector2d(0, 0);

	position = Vector2d(0, 0);
	velocity = Vector2d(0, 0);
	desiredDirection = Vector2d(0, 0);
}

void Agent::setRadius(const double r) 
{
	this->radius = r;
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
	const double reactionTime = 0.5;											//反応時間(s)
	const Vector2d target = Vector2d(roomData.getRoom_size_x() + radius, 0);	//目的地
	Vector2d f_driv;

	desiredDirection = unitVector(position, target);
	f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);

	return f_driv;
}

Vector2d Agent::agentInteractForce(const std::vector<Agent>& agents) 
{
	const double A = 2000;					//心理的反発力(N)
	const double B = 0.08;					//心理的反発距離(m)
	const double k = 120000;				//滑り摩擦係数(kg/s^2)
	const double kappa = 240000;			//体圧縮係数(kg/m・s)
	const double r_i = radius;				//自身の半径(m)
	const int N_agent = agents.size();		//agentの人数(人)

	double r_j = 0;							//agent[j]の半径(m)
	double r_ij = 0;						//自身とagent[j]の半径の和(m)
	double d_ij = 0;						//自身とagent[j]の中心間距離(m)
	double v_ji = 0;
	Vector2d n_ij, t_ij, f_soc, f_rep, f_fric, f_ij;

	for (int j = 0; j < N_agent; ++j) 
	{
		d_ij = distance(position, agents[j].getPosition());

		if (d_ij != 0) //自分自身を除外
		{
			r_j = agents[j].getRadius();
			r_ij = r_i + r_j;
			n_ij = unitVector(agents[j].getPosition(), position);		//j→i方向の単位ベクトル
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

	const double A = 2000;							//心理的反発力(N)
	const double B = 0.08;							//心理的反発距離(m)
	const double k = 120000;						//滑り摩擦係数(kg/s^2)
	const double kappa = 240000;					//体圧縮係数(kg/m・s)
	const double r_i = radius;						//自身の半径(m)
	const int N_wall = wallCornerPoint.size();		//壁の数(個)

	double d_iw = 0;								//自身と壁との距離(m)
	Vector2d nearestPoint, n_iw, t_iw, f_soc, f_rep, f_fric, f_iw;

	for (int n = 0; n < N_wall; ++n) 
	{
		nearestPoint = getNearestPoint(wallCornerPoint[n][0], wallCornerPoint[n][1], position);
		d_iw = distance(position, nearestPoint);
		n_iw = unitVector(nearestPoint, position);		//w→i方向の単位ベクトル
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
	Vector2d f_soc;

	f_driv = drivingForce(roomData);
	f_ij = agentInteractForce(agents);
	f_iw = wallInteractForce(roomData);

	f_soc = f_driv + f_ij + f_iw;

	velocity = velocity + (stepTime / mass) * f_soc;	
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

	//避難者の座標を被らせないための処理
	do 
	{
		double r = agents[i].getRadius();

		// r 以上 room_size_x - r 未満の実数を一様乱数で発生させる
		std::uniform_real_distribution<> dist_x(r, room_size_x - r);
		// (-room_size_y / 2) + r 以上 (room_size_y / 2) - r 未満の実数を一様乱数で発生させる
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

void removeAgent(std::vector<Agent>& agents, Room roomData) 
{
	double exitPosition_x = roomData.getRoom_size_x();
	erase_if(agents, [exitPosition_x](Agent agent) {return agent.getPosition().x >= exitPosition_x; });
}
