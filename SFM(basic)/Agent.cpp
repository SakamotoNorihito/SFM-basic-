#include<vector>
#include<random>
#include"Agent.h"
using namespace std;

//数学定数
const double PI = 3.14159;	//円周率

Agent::Agent() 
{
	mass = 80;			//エージェントの質量(kg)
	radius = 0.25;		//エージェント半径(m)
	desiredSpeed = 1;	//希望速さ(m/s)
	R_ind = 0;			//誘導者の誘導半径(m)
	R_vis = 0;			//エージェントの視界半径(m)

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

//guide用drivingForce
Vector2d Agent::drivingForce_g(const Room room)
{
	const double reactionTime = 0.5;										//反応時間(s)
	const Vector2d target = Vector2d(room.getRoom_size_x() + radius, 0);	//目的地
	Vector2d f_driv;

	desiredDirection = unitVector(position, target);
	f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);

	return f_driv;
}

//evacuee用drivingForce
Vector2d Agent::drivingForce_e(const Room room, const std::vector<Agent>& guide, const std::vector<Agent>& evacuee)
{
	const double reactionTime = 0.5;										//反応時間(s)
	const Vector2d target = Vector2d(room.getRoom_size_x() + radius, 0);	//目的地
	Vector2d f_driv;
	
	double d_it = distance(position, target);	//自身と目的地までの距離

	//目的地が視界に入っているとき
	if (d_it < R_vis)
	{
		desiredDirection = unitVector(position, target);
		f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);
	}

	//目的地が視界に入っていないとき
	else
	{
		int N_guide = guide.size();
		vector<double> d_ig(N_guide);

		//避難者と各誘導者の距離を計算
		for (int i = 0; i < N_guide; ++i)
		{
			d_ig[i] = distance(position, guide[i].getPosition());
		}

		//最も近い誘導者までの距離と誘導者番号を取得（参考：https://zenn.dev/reputeless/books/standard-cpp-for-competitive-programming/viewer/library-algorithm#1.5-%E9%85%8D%E5%88%97%E3%81%AE%E4%B8%AD%E3%81%8B%E3%82%89%E6%9C%80%E5%B0%8F%E3%81%AE%E8%A6%81%E7%B4%A0%E3%81%A8%E3%81%9D%E3%81%AE%E4%BD%8D%E7%BD%AE%E3%82%92%E5%BE%97%E3%82%8B）
		auto it = min_element(d_ig.begin(), d_ig.end());
		double d_ig_min = *it;									//誘導者までの距離
		int nearestGuideNumber = distance(d_ig.begin(), it);	//誘導者番号
		double R_ind = guide[nearestGuideNumber].getR_ind();	//誘導者の誘導半径

		//誘導者の誘導半径内にいるとき
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

		//誘導者の誘導半径外にいるとき
		else
		{
			int N_aroundAgent = 0;
			Vector2d O, v_total, v_average;

			int N_guide = guide.size();
			int N_evacuee = evacuee.size();

			//自身の視界範囲内にいる誘導者の数え上げ
			for (int i = 0; i < N_guide; ++i)
			{
				double d_ig = distance(position, guide[i].getPosition());

				if (d_ig != 0 && d_ig < R_vis)
				{
					N_aroundAgent++;
					v_total = v_total + guide[i].getPosition();
				}
			}

			//自身の視界範囲内にいる避難者の数え上げ
			for (int i = 0; i < N_evacuee; ++i)
			{
				double d_ij = distance(position, evacuee[i].getPosition());

				if (d_ij != 0 && d_ij < R_vis)
				{
					N_aroundAgent++;
					v_total = v_total + evacuee[i].getPosition();
				}
			}

			//自身の視界範囲内に他のエージェントが存在しないとき
			if (N_aroundAgent == 0)
			{
				std::random_device seed_gen;
				std::default_random_engine engine(seed_gen());

				// 0以上1.0未満の値を等確率で発生させる
				std::uniform_real_distribution<> dist(0, 1.0);

				double theta = 2 * PI * dist(engine);
				Vector2d randomDirection = Vector2d(cos(theta), sin(theta));

				desiredDirection = unitVector(O, randomDirection);
			}

			//自身の視界範囲内に他のエージェントが存在するとき
			else
			{
				//他のエージェントの速度ベクトルの総和 v_total が零ベクトルのとき
				if (v_total.x == 0 && v_total.y == 0)
				{
					std::random_device seed_gen;
					std::default_random_engine engine(seed_gen());

					// 0以上1.0未満の値を等確率で発生させる
					std::uniform_real_distribution<> dist(0, 1.0);

					double theta = 2 * PI * dist(engine);
					Vector2d randomDirection = Vector2d(cos(theta), sin(theta));

					desiredDirection = unitVector(O, randomDirection);
				}

				//他のエージェントの速度ベクトルの総和 v_total が零ベクトルでないとき
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
