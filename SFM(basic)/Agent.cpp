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
	R_vis = 3;			//エージェントの視界半径(m)

	f_driv = Vector2d(0, 0);
	f_ig = Vector2d(0, 0);
	f_ij = Vector2d(0, 0);
	f_iw = Vector2d(0, 0);

	position = Vector2d(0, 0);
	velocity = Vector2d(0, 0);
	desiredDirection = Vector2d(0, 0);
	desiredVelocity = Vector2d(0, 0);
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

void Agent::setDesiredVelocity(const Vector2d e_v)
{
	this->desiredVelocity.x = e_v.x;
	this->desiredVelocity.y = e_v.y;
}

double Agent::calculateDesiredSpeed(const std::vector<Agent>& guide, const std::vector<Agent>& evacuee)
{
	const double chi1 = 0.6;
	const double chi2 = 0.3;
	const double chi3 = 0.1;
	const double v_min = 0.6;
	const double v_max = 2.4;

	double firstTerm = 0;
	double secondTerm = 0;
	double thirdTerm = 0;
	double e_i = 0;

	//第一項に必要な要素
	double v_id = dotProduct(velocity, desiredDirection);

	//第二項に必要な要素
	int N_guide = guide.size();
	int N_evacuee = evacuee.size();

	const double lambda = 1;	//視界異方性の強さを表すパラメータ
	int N_aroundAgent = 0;
	Vector2d v_total, v_mean;
	const Vector2d O;
	double v_0 = 0, v_i = 0;

	//自身の視界範囲内にいる誘導者の数え上げ
	for (int i = 0; i < N_guide; ++i)
	{
		double d_ig = distance(position, guide[i].getPosition());

		if (d_ig != 0 && d_ig <= R_vis)
		{
			N_aroundAgent++;

			//視界異方性係数の計算
			Vector2d n_gi = unitVector(position, guide[i].getPosition());	//i→g方向の単位ベクトル
			double cos_phi = dotProduct(desiredDirection, n_gi);
			double anisotropy_coef = lambda + (1 - lambda) * ((1 + cos_phi) / 2);

			v_total = v_total + anisotropy_coef * guide[i].getVelocity();
		}
	}

	//自身の視界範囲内にいる避難者の数え上げ
	for (int i = 0; i < N_evacuee; ++i)
	{
		double d_ij = distance(position, evacuee[i].getPosition());

		if (d_ij != 0 && d_ij <= R_vis)
		{
			N_aroundAgent++;

			//視界異方性係数の計算
			Vector2d n_ji = unitVector(position, evacuee[i].getPosition());	//i→j方向の単位ベクトル
			double cos_phi = dotProduct(desiredDirection, n_ji);
			double anisotropy_coef = lambda + (1 - lambda) * ((1 + cos_phi) / 2);

			v_total = v_total + anisotropy_coef * evacuee[i].getVelocity();
		}
	}

	if (N_aroundAgent != 0)
	{
		v_mean = v_total / N_aroundAgent;
	}

	v_0 = distance(O, v_mean);
	v_i = distance(O, velocity);

	//第三項に必要な要素
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());

	// 0以上1.0未満の値を等確率で発生させる
	std::uniform_real_distribution<> dist(0, 1.0);

	double xi = dist(engine);

	/*
	* desiredSpeedが0の時、区分関数g(x)の中で0除算が発生
	* [desiredSpeed]及び[v_0]に注意
	*/

	if (desiredSpeed != 0)
	{
		firstTerm = chi1 * g((desiredSpeed - v_id) / desiredSpeed);
	}

	if (v_0 != 0)
	{
		secondTerm = chi2 * g((v_0 - v_i) / v_0);
	}

	thirdTerm = chi3 * xi;
	e_i = firstTerm + secondTerm + thirdTerm;

	return (1 - e_i) * v_min + e_i * v_max;
}

bool Agent::canSeeExit(const Room room)
{
	const Vector2d destination = Vector2d(room.getRoom_size_x(), 0);
	double delta = 0;	//確率が変化する領域の幅
	double p = 0;		//出口が見える確率
	double d_ie = distance(position, destination);		// 自身と目的地までの間の距離

	//出口が確実に視認できるとき
	if (d_ie <= R_vis - delta)
	{
		p = 1;
	}
	//出口が確実に視認できないとき
	else if (d_ie >= R_vis + delta)
	{
		p = 0;
	}
	else
	{
		p = (-1 / (2 * delta)) * (d_ie - (R_vis + delta));	// 距離に応じた出口が見える確率を計算
	}	

	//出口視認判定を行うための乱数を生成する
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());	
	std::uniform_real_distribution<> dist(0, 1.0);		// 0以上1.0未満の値を等確率で発生させる
	double N_random = dist(engine);

	//乱数N_randomが出口が見える確率p以下の場合
	if (N_random <= p)
	{
		return true;	// 出口が見える
	}
	//乱数N_randomが出口が見える確率pより大きい場合
	else
	{
		return false;	// 出口が見えない
	}
}

//guide用drivingForce
Vector2d Agent::drivingForce_g(const Room room)
{
	const double reactionTime = 0.5;										//反応時間(s)
	const Vector2d target = Vector2d(room.getRoom_size_x() + radius, 0);	//目的地（出口）
	Vector2d f_driv;

	//desiredSpeed = 1;
	desiredDirection = unitVector(position, target);
	f_driv = (mass / reactionTime) * ((desiredSpeed * desiredDirection) - velocity);

	return f_driv;
}

//evacuee用drivingForce
Vector2d Agent::drivingForce_e(Room room, const std::vector<Agent>& guide, const std::vector<Agent>& evacuee)
{
	const double reactionTime = 0.5;										//反応時間(s)
	const Vector2d O, destination = Vector2d(room.getRoom_size_x(), 0);		//目的地
	int N_guide = guide.size();
	int N_evacuee = evacuee.size();
	Vector2d f_driv;

	double d_id = distance(position, destination);

	//出口が見えるとき、出口を目指す
	if (d_id <= R_vis)
	{
		desiredDirection = unitVector(position, destination);
		desiredVelocity = desiredSpeed * desiredDirection;
	}

	//誘導者が1人以上いるとき
	else if (N_guide > 0)
	{
		vector<double> d_iL(N_guide);

		//避難者と各誘導者の距離を計算
		for (int i = 0; i < N_guide; ++i)
		{
			d_iL[i] = distance(position, guide[i].getPosition());
		}

		//最も近い誘導者までの距離と誘導者番号を取得（参考：https://zenn.dev/reputeless/books/standard-cpp-for-competitive-programming/viewer/library-algorithm#1.5-%E9%85%8D%E5%88%97%E3%81%AE%E4%B8%AD%E3%81%8B%E3%82%89%E6%9C%80%E5%B0%8F%E3%81%AE%E8%A6%81%E7%B4%A0%E3%81%A8%E3%81%9D%E3%81%AE%E4%BD%8D%E7%BD%AE%E3%82%92%E5%BE%97%E3%82%8B）
		auto it = min_element(d_iL.begin(), d_iL.end());
		double d_iL_min = *it;									//誘導者までの距離
		int nearestGuideNumber = distance(d_iL.begin(), it);	//誘導者番号
		double R_ind = guide[nearestGuideNumber].getR_ind();	//誘導者の誘導半径

		//誘導者の誘導半径内にいるとき、誘導に従う
		if (d_iL_min <= R_ind)
		{
			double rho_L = exp(-d_iL_min / R_vis);
			Vector2d e_L = guide[nearestGuideNumber].getDesiredDirection();
			Vector2d n_iL = unitVector(position, guide[nearestGuideNumber].getPosition());

			desiredDirection = unitVector(O, rho_L * e_L + (1 - rho_L) * n_iL);
			desiredVelocity = desiredSpeed * desiredDirection;
		}
	}

	//出口も誘導者も見えないとき、周囲の避難者に追従するor壁に沿って移動する（壁が見えないときはランダム方向に前進する）
	else
	{
		const double nu = 1;			//追従確率P(N)を特徴づけるパラメータν
		const double sigma_th = 0;		//速度の方向の標準偏差の閾値

		int N_aroundAgent = 0;
		vector<double> theta;
		Vector2d totalPosition, totalVelocity;

		//自身の視界範囲内にいる誘導者の情報を参照
		for (int i = 0; i < N_guide; ++i)
		{
			double d_ig = distance(position, guide[i].getPosition());

			if (d_ig != 0 && d_ig <= R_vis)
			{
				N_aroundAgent++;

				totalPosition = totalPosition + guide[i].getPosition();
				totalVelocity = totalVelocity + guide[i].getVelocity();

				double theta_i = atan2(guide[i].getVelocity().y, guide[i].getVelocity().x);
				theta.push_back(theta_i);
			}			
		}

		//自身の視界範囲内にいる避難者の情報を参照
		for (int i = 0; i < N_evacuee; ++i)
		{
			double d_ij = distance(position, evacuee[i].getPosition());

			if (d_ij != 0 && d_ij <= R_vis)
			{
				N_aroundAgent++;

				totalPosition = totalPosition + evacuee[i].getPosition();
				totalVelocity = totalVelocity + evacuee[i].getVelocity();

				double theta_i = atan2(evacuee[i].getVelocity().y, evacuee[i].getVelocity().x);
				theta.push_back(theta_i);
			}
		}

		//周囲の避難者数に応じた追従確率を計算
		double followingProbability = 1 - exp(-N_aroundAgent / nu);

		//周囲にの避難者に追従するか判定するための乱数を生成
		std::random_device seed_gen;
		std::default_random_engine engine(seed_gen());
		std::uniform_real_distribution<> dist(0, 1.0);	// 0以上1.0未満の値を等確率で発生させる
		double p = dist(engine);

		//追従するとき
		if (p < followingProbability)
		{
			Vector2d centerOfGravity = totalPosition / N_aroundAgent;
			double d_iG = distance(position, centerOfGravity);

			double rho_f = exp(-d_iG / R_vis);
			//double rho_f = 0.1;
			Vector2d e_crowd = totalVelocity / N_aroundAgent;
			Vector2d n_iG = unitVector(position, centerOfGravity);

			desiredDirection = unitVector(O, rho_f * e_crowd + (1 - rho_f) * n_iG);
			desiredVelocity = desiredSpeed * desiredDirection;
		}

		else
		{
			desiredDirection = O;
			desiredVelocity = desiredSpeed * desiredDirection;
		}










		//if (N_aroundAgent != 0)
		//{
		//	Vector2d centerOfGravity = totalPosition / N_aroundAgent;
		//	double d_iG = distance(position, centerOfGravity);

		//	double rho_f = exp(-d_iG / R_vis);
		//	//double rho_f = 0.1;
		//	Vector2d e_crowd = totalVelocity / N_aroundAgent;
		//	Vector2d n_iG = unitVector(position, centerOfGravity);

		//	desiredDirection = unitVector(O, rho_f * e_crowd + (1 - rho_f) * n_iG);
		//	desiredVelocity = desiredSpeed * desiredDirection;
		//}

		







		



		



		////目的地が視界に入っていないとき
		//else
		//{
		//	//誘導者が1人以上存在するとき
		//	if (N_guide > 0)
		//	{
		//		vector<double> d_ig(N_guide);

		//		//避難者と各誘導者の距離を計算
		//		for (int i = 0; i < N_guide; ++i)
		//		{
		//			d_ig[i] = distance(position, guide[i].getPosition());
		//		}

		//		//最も近い誘導者までの距離と誘導者番号を取得（参考：https://zenn.dev/reputeless/books/standard-cpp-for-competitive-programming/viewer/library-algorithm#1.5-%E9%85%8D%E5%88%97%E3%81%AE%E4%B8%AD%E3%81%8B%E3%82%89%E6%9C%80%E5%B0%8F%E3%81%AE%E8%A6%81%E7%B4%A0%E3%81%A8%E3%81%9D%E3%81%AE%E4%BD%8D%E7%BD%AE%E3%82%92%E5%BE%97%E3%82%8B）
		//		auto it = min_element(d_ig.begin(), d_ig.end());
		//		double d_ig_min = *it;									//誘導者までの距離
		//		int nearestGuideNumber = distance(d_ig.begin(), it);	//誘導者番号
		//		double R_ind = guide[nearestGuideNumber].getR_ind();	//誘導者の誘導半径

		//		//誘導者の誘導半径内にいるとき
		//		if (d_ig_min <= R_ind)
		//		{
		//			/*
		//			[A social force evacuation model with the leadership effect, Hou, L et al.]より
		//			double rho = exp(-d_ig_min / (2 * R_ind));
		//			Vector2d e_g = unitVector(guide[nearestGuideNumber].getPosition(), target);
		//			*/
		//			
		//			double xi = 3;		//誘導者に並走する成分と接近する成分を重み付けするためのパラメータ
		//			double rho = exp(-d_ig_min / xi);
		//			Vector2d e_g_i;
		//			Vector2d e_g = guide[nearestGuideNumber].getDesiredDirection();
		//			Vector2d n_ig = unitVector(guide[nearestGuideNumber].getPosition(), position);

		//			e_g_i = rho * e_g - (1 - rho) * n_ig;
		//			e_i = unitVector(O, e_g_i);
		//		}

		//		//誘導者の誘導半径外にいるとき
		//		else
		//		{
		//			//個人的希望方向ベクトルが零ベクトルのとき、希望方向をランダムに初期化する
		//			if (e_i.x == 0 && e_i.y == 0)
		//			{
		//				std::random_device seed_gen;
		//				std::default_random_engine engine(seed_gen());

		//				// 0以上1.0未満の値を等確率で発生させる
		//				std::uniform_real_distribution<> dist(0, 1.0);

		//				double theta = 2 * PI * dist(engine);
		//				Vector2d randomDirection = Vector2d(cos(theta), sin(theta));

		//				e_i = unitVector(O, randomDirection);
		//			}

		//			//個人的希望方向ベクトルが零ベクトルでないとき
		//			else
		//			{
		//				const vector<vector<Vector2d>> wallCornerPoint = room.createWall();
		//				const int N_wall = wallCornerPoint.size();		//壁の数(個)					

		//				double d_iw = 0;								//自身と壁との距離(m)
		//				static bool changeDirection = true;				//壁が2つ見えている時に1度だけ希望方向を変更するための判定用変数
		//				vector<int> isVisibleWallNumber;				//視認できる壁の番号を格納する配列						

		//				for (int n = 0; n < N_wall; ++n)
		//				{
		//					Vector2d nearestPoint = getNearestPoint(wallCornerPoint[n][0], wallCornerPoint[n][1], position);
		//					d_iw = distance(position, nearestPoint);

		//					//自身の視界範囲内に壁が見えるとき
		//					if (d_iw <= R_vis)
		//					{
		//						isVisibleWallNumber.push_back(n);
		//					}
		//				}

		//				//見える壁がない時、個人的希望方向を上書きしない
		//				if (isVisibleWallNumber.size() == 0)
		//				{

		//				}

		//				//見える壁が１つの時、見える壁に沿って移動する
		//				else if (isVisibleWallNumber.size() == 1)
		//				{
		//					Vector2d alongWallDirection = unitVector(wallCornerPoint[isVisibleWallNumber[0]][0], wallCornerPoint[isVisibleWallNumber[0]][1]);

		//					//個人的希望方向と壁に沿う単位ベクトルの内積を計算し、それまでの希望方向に沿った方向に進行する
		//					if (dotProduct(desiredDirection, alongWallDirection) >= 0)
		//					{
		//						e_i.x = alongWallDirection.x;
		//						e_i.y = alongWallDirection.y;
		//					}
		//					else
		//					{
		//						e_i.x = -alongWallDirection.x;
		//						e_i.y = -alongWallDirection.y;
		//					}

		//					changeDirection = true;		//壁が2つ見えた時に希望方向を切り替えられるようにフラグをONにする
		//				}

		//				//見える壁が２つの時、沿う壁を変更する
		//				else if (isVisibleWallNumber.size() == 2)
		//				{
		//					//個人的希望方向変更フラグがONの時
		//					if (changeDirection == true)
		//					{
		//						Vector2d alongWallDirection1 = unitVector(wallCornerPoint[isVisibleWallNumber[0]][0], wallCornerPoint[isVisibleWallNumber[0]][1]);
		//						Vector2d alongWallDirection2 = unitVector(wallCornerPoint[isVisibleWallNumber[1]][0], wallCornerPoint[isVisibleWallNumber[1]][1]);

		//						//壁１が進行方向に存在するとき、壁２から遠ざかる
		//						if (dotProduct(desiredDirection, alongWallDirection1) == 0)
		//						{
		//							Vector2d nearestPoint = getNearestPoint(wallCornerPoint[isVisibleWallNumber[1]][0], wallCornerPoint[isVisibleWallNumber[1]][1], position);
		//							Vector2d n_iw = unitVector(nearestPoint, position);

		//							e_i.x = n_iw.x;
		//							e_i.y = n_iw.y;
		//						}
		//						//壁２が進行方向に存在するとき、壁１から遠ざかる
		//						else if (dotProduct(desiredDirection, alongWallDirection2) == 0)
		//						{
		//							Vector2d nearestPoint = getNearestPoint(wallCornerPoint[isVisibleWallNumber[0]][0], wallCornerPoint[isVisibleWallNumber[0]][1], position);
		//							Vector2d n_iw = unitVector(nearestPoint, position);

		//							e_i.x = n_iw.x;
		//							e_i.y = n_iw.y;
		//						}

		//						isVisibleWallNumber.clear();		//見えている壁の記憶を破棄する
		//						changeDirection = false;			//個人的希望方向変更フラグをOFFにする
		//					}
		//				}
		//			}		
		//		}
		//	}

		//	//誘導者が存在しないとき
		//	else
		//	{
		//		//個人的希望方向ベクトルが零ベクトルのとき、希望方向をランダムに初期化する
		//		if (e_i.x == 0 && e_i.y == 0)
		//		{
		//			std::random_device seed_gen;
		//			std::default_random_engine engine(seed_gen());

		//			// 0以上1.0未満の値を等確率で発生させる
		//			std::uniform_real_distribution<> dist(0, 1.0);

		//			double theta = 2 * PI * dist(engine);
		//			Vector2d randomDirection = Vector2d(cos(theta), sin(theta));

		//			e_i = unitVector(O, randomDirection);
		//		}

		//		//個人的希望方向ベクトルが零ベクトルでないとき
		//		else
		//		{
		//			const vector<vector<Vector2d>> wallCornerPoint = room.createWall();
		//			const int N_wall = wallCornerPoint.size();		//壁の数(個)					

		//			double d_iw = 0;								//自身と壁との距離(m)
		//			static bool changeDirection = true;				//壁が2つ見えている時に1度だけ希望方向を変更するための判定用変数
		//			vector<int> isVisibleWallNumber;				//視認できる壁の番号を格納する配列						

		//			for (int n = 0; n < N_wall; ++n)
		//			{
		//				Vector2d nearestPoint = getNearestPoint(wallCornerPoint[n][0], wallCornerPoint[n][1], position);
		//				d_iw = distance(position, nearestPoint);

		//				//自身の視界範囲内に壁が見えるとき
		//				if (d_iw <= R_vis)
		//				{
		//					isVisibleWallNumber.push_back(n);
		//				}
		//			}

		//			//見える壁がない時、個人的希望方向を上書きしない
		//			if (isVisibleWallNumber.size() == 0)
		//			{

		//			}

		//			//見える壁が１つの時、見える壁に沿って移動する
		//			else if (isVisibleWallNumber.size() == 1)
		//			{
		//				Vector2d alongWallDirection = unitVector(wallCornerPoint[isVisibleWallNumber[0]][0], wallCornerPoint[isVisibleWallNumber[0]][1]);

		//				//個人的希望方向と壁に沿う単位ベクトルの内積を計算し、それまでの希望方向に沿った方向に進行する
		//				if (dotProduct(desiredDirection, alongWallDirection) >= 0)
		//				{
		//					e_i.x = alongWallDirection.x;
		//					e_i.y = alongWallDirection.y;
		//				}
		//				else
		//				{
		//					e_i.x = -alongWallDirection.x;
		//					e_i.y = -alongWallDirection.y;
		//				}

		//				changeDirection = true;		//壁が2つ見えた時に希望方向を切り替えられるようにフラグをONにする
		//			}

		//			//見える壁が２つの時、沿う壁を変更する
		//			else if (isVisibleWallNumber.size() == 2)
		//			{
		//				//個人的希望方向変更フラグがONの時
		//				if (changeDirection == true)
		//				{
		//					Vector2d alongWallDirection1 = unitVector(wallCornerPoint[isVisibleWallNumber[0]][0], wallCornerPoint[isVisibleWallNumber[0]][1]);
		//					Vector2d alongWallDirection2 = unitVector(wallCornerPoint[isVisibleWallNumber[1]][0], wallCornerPoint[isVisibleWallNumber[1]][1]);

		//					//壁１が進行方向に存在するとき、壁２から遠ざかる
		//					if (dotProduct(desiredDirection, alongWallDirection1) == 0)
		//					{
		//						Vector2d nearestPoint = getNearestPoint(wallCornerPoint[isVisibleWallNumber[1]][0], wallCornerPoint[isVisibleWallNumber[1]][1], position);
		//						Vector2d n_iw = unitVector(nearestPoint, position);

		//						e_i.x = n_iw.x;
		//						e_i.y = n_iw.y;
		//					}
		//					//壁２が進行方向に存在するとき、壁１から遠ざかる
		//					else if (dotProduct(desiredDirection, alongWallDirection2) == 0)
		//					{
		//						Vector2d nearestPoint = getNearestPoint(wallCornerPoint[isVisibleWallNumber[0]][0], wallCornerPoint[isVisibleWallNumber[0]][1], position);
		//						Vector2d n_iw = unitVector(nearestPoint, position);

		//						e_i.x = n_iw.x;
		//						e_i.y = n_iw.y;
		//					}

		//					isVisibleWallNumber.clear();		//見えている壁の記憶を破棄する
		//					changeDirection = false;			//個人的希望方向変更フラグをOFFにする
		//				}
		//			}
		//		}
		//	}
		//}

		///* e_j_iの計算項 */
		//const double lambda = 1;	//視界異方性の強さを表すパラメータ
		//int N_aroundAgent = 0;
		//Vector2d e_total;

		////自身の視界範囲内にいる誘導者の数え上げ
		//for (int i = 0; i < N_guide; ++i)
		//{
		//	double d_ig = distance(position, guide[i].getPosition());

		//	if (d_ig != 0 && d_ig <= R_vis)
		//	{
		//		N_aroundAgent++;

		//		//視界異方性係数の計算
		//		Vector2d n_gi = unitVector(position, guide[i].getPosition());	//i→g方向の単位ベクトル
		//		double cos_phi = dotProduct(desiredDirection, n_gi);
		//		double anisotropy_coef = lambda + (1 - lambda) * ((1 + cos_phi) / 2);

		//		e_total = e_total + anisotropy_coef * guide[i].getVelocity();
		//	}
		//}

		////自身の視界範囲内にいる避難者の数え上げ
		//for (int i = 0; i < N_evacuee; ++i)
		//{
		//	double d_ij = distance(position, evacuee[i].getPosition());

		//	if (d_ij != 0 && d_ij <= R_vis)
		//	{
		//		N_aroundAgent++;

		//		//視界異方性係数の計算
		//		Vector2d n_ji = unitVector(position, evacuee[i].getPosition());	//i→j方向の単位ベクトル
		//		double cos_phi = dotProduct(desiredDirection, n_ji);
		//		double anisotropy_coef = lambda + (1 - lambda) * ((1 + cos_phi) / 2);

		//		e_total = e_total + anisotropy_coef * evacuee[i].getVelocity();
		//	}
		//}

		//if (N_aroundAgent != 0)
		//{
		//	e_j_i = e_total / N_aroundAgent;
		//}	

		///* drivingForceの計算項 */
		//desiredDirection = unitVector(O, (1 - panicParameter) * e_i + panicParameter * e_j_i);	//e_j_iが0かつpanicParameterが1の時、0除算が発生
		//desiredSpeed = calculateDesiredSpeed(guide, evacuee);
		//desiredVelocity = desiredSpeed * desiredDirection;
		//f_driv = (mass / reactionTime) * (desiredVelocity - velocity);

		//return f_driv;
	}

	f_driv = (mass / reactionTime) * (desiredVelocity - velocity);

	return f_driv;

}

Vector2d Agent::agentInteractForce(const std::vector<Agent>& agents)
{
	const double A = 2000;					//心理的反発力(N)
	const double B = 0.08;					//心理的反発距離(m)
	const double k = 120000;				//体圧縮係数(kg/s^2)
	const double kappa = 240000;			//滑り摩擦係数(kg/m・s)
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

		//自分自身を除外　かつ　エージェントが視界範囲内にいるとき
		if (d_ij != 0 && d_ij < R_vis)
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
	const double k = 120000;						//体圧縮係数(kg/s^2)
	const double kappa = 240000;					//滑り摩擦係数(kg/m・s)
	const double r_i = radius;						//自身の半径(m)
	const int N_wall = wallCornerPoint.size();		//壁の数(個)

	double d_iw = 0;								//自身と壁との距離(m)
	Vector2d nearestPoint, n_iw, t_iw, f_soc, f_rep, f_fric, f_iw;

	for (int n = 0; n < N_wall; ++n) 
	{
		nearestPoint = getNearestPoint(wallCornerPoint[n][0], wallCornerPoint[n][1], position);
		d_iw = distance(position, nearestPoint);

		//壁が自身の視界範囲内にあるとき
		if (d_iw < R_vis)
		{
			n_iw = unitVector(nearestPoint, position);	//w→i方向の単位ベクトル
			t_iw = tangentialVector(n_iw);

			f_soc = A * exp((r_i - d_iw) / B) * n_iw;
			f_rep = k * g(r_i - d_iw) * n_iw;
			f_fric = kappa * g(r_i - d_iw) * dotProduct(velocity, t_iw) * t_iw;

			f_iw = f_iw + f_soc + f_rep - f_fric;
		}		
	}

	return f_iw;
}

//guide用move関数
void Agent::move_g(std::vector<Agent>& guide, const std::vector<Agent>& evacuee, const Room room, const double stepTime)
{
	Vector2d f_soc;

	f_driv = drivingForce_g(room);
	f_ig = agentInteractForce(guide);
	f_ij = agentInteractForce(evacuee);
	f_iw = wallInteractForce(room);

	f_soc = f_driv + f_ig + f_ij + f_iw;

	velocity = velocity + (stepTime / mass) * f_soc;
	position = position + stepTime * velocity;
}

//evacuee用move関数
void Agent::move_e(std::vector<Agent>& evacuee, const std::vector<Agent>& guide, const Room room, const double stepTime)
{
	Vector2d f_soc;

	f_driv = drivingForce_e(room, guide, evacuee);
	f_ig = agentInteractForce(guide);
	f_ij = agentInteractForce(evacuee);
	f_iw = wallInteractForce(room);

	f_soc = f_driv + f_ig + f_ij + f_iw;

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

void setInitialPosition_g(const Room room, std::vector<Agent>& guide)
{
	const double room_size_x = room.getRoom_size_x();
	const double room_size_y = room.getRoom_size_y();
	const int N_guide = guide.size();

	//誘導者の初期位置の候補	
	const Vector2d p1 = Vector2d(room_size_x / 4, room_size_y / 4);
	const Vector2d p2 = Vector2d(room_size_x / 4, 0);
	const Vector2d p3 = Vector2d(room_size_x / 4, -room_size_y / 4);
	const Vector2d p4 = Vector2d(room_size_x / 2, room_size_y / 4);
	const Vector2d p5 = Vector2d(room_size_x / 2, 0);
	const Vector2d p6 = Vector2d(room_size_x / 2, -room_size_y / 4);
	const Vector2d p7 = Vector2d(3 * room_size_x / 4, room_size_y / 4);
	const Vector2d p8 = Vector2d(3 * room_size_x / 4, 0);
	const Vector2d p9 = Vector2d(3 * room_size_x / 4, -room_size_y / 4);
	const Vector2d outOfRoom = Vector2d(1000000, 0);

	//Yang(2014)再現用初期位置
	//const Vector2d p_Yang_2014_1 = Vector2d(3, (-room_size_y / 2) + 3);
	//const Vector2d p_Yang_2014_2 = Vector2d(12, (-room_size_y / 2) + 12);

	for (int i = 0; i < N_guide; ++i)
	{
		const Vector2d p_deepCenter = Vector2d(guide[0].getRadius(), 0);

		switch (i)	//誘導者毎初期配置を指定する
		{
		case 0:
			guide[i].setPosition(outOfRoom);	//部屋の左壁中央
			break;
		case 1:
			guide[i].setPosition(p3);
			break;
		case 2:
			guide[i].setPosition(p4);
			break;
		case 3:
			guide[i].setPosition(p6);
			break;
		case 4:
			guide[i].setPosition(Vector2d(guide[i].getRadius(), -3));
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
	vector<bool> isOverlap(N_guide);	//避難者と各誘導者毎の座標被りの判定結果を格納する配列
	bool judge = false;					//全ての誘導者との座標被りの判定結果を格納する変数

	do
	{
		double r_i = evacuee[i].getRadius();

		// r_i 以上 room_size_x - r_i 未満の実数を一様乱数で発生させる
		std::uniform_real_distribution<> dist_x(r_i, room_size_x - r_i);
		// (-room_size_y / 2) + r_i 以上 (room_size_y / 2) - r_i 未満の実数を一様乱数で発生させる
		std::uniform_real_distribution<> dist_y((-room_size_y / 2) + r_i, (room_size_y / 2) - r_i);

		evacuee[i].setPosition(Vector2d(dist_x(engine), dist_y(engine)));

		//避難者と誘導者の座標被りの判定
		for (int g = 0; g < N_guide; ++g)
		{
			double r_g = guide[g].getRadius();
			double r_ig = r_i + r_g;
			double d_ig = distance(evacuee[i].getPosition(), guide[g].getPosition());

			//避難者と誘導者の体が重なっている場合
			if (d_ig < r_ig)
			{
				isOverlap[g] = true;
			}

			//避難者と誘導者の体が重なっていない場合
			else
			{
				isOverlap[g] = false;
			}
		}

		//全ての誘導者と座標被りを起こしていないか判定
		judge = all_of(isOverlap.begin(), isOverlap.end(), [](bool t) { return t == false; });

		//全ての誘導者と座標被りを起こしていないとき
		if (judge)
		{
			int j = 0;

			//避難者間で座標被りを起こさないための処理
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

					//避難者同士で体が重なっている場合
					if (d_ij < r_ij)
					{
						break;
					}

					//避難者同士で体が重なっていない場合
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

double calculationSD(const std::vector<double>& data)
{
	int N_data = data.size();
	double data_mean = 0;
	double temp = 0;

	for (int i = 0; i < N_data; ++i)
	{
		temp += data[i];
	}

	data_mean = temp / N_data;

	temp = 0;

	for (int i = 0; i < N_data; ++i)
	{
		temp += (data[i] - data_mean) * (data[i] - data_mean);
	}	

	return sqrt(temp / N_data);
}
