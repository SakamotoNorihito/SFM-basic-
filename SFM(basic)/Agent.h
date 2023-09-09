#pragma once
#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include"Vector2d_math.h"
#include "Room.h"

class Agent : public Vector2d {
private:
	double mass;
	double radius;
	double desiredSpeed;
	double R_ind;
	double R_vis;

	Vector2d position;
	Vector2d velocity;
	Vector2d desiredDirection;

	Vector2d drivingForce(const Room roomData);

	//guide用drivingForce
	Vector2d drivingForce_g(const Room room);
	//evacuee用drivingForce
	Vector2d drivingForce_e(const Room room, const std::vector<Agent>& guide, const std::vector<Agent>& evacuee);

	Vector2d agentInteractForce(const std::vector<Agent>& agents);
	Vector2d wallInteractForce(Room roomData);

public:
	Agent();

	void setMass(const double m);
	void setRadius(const double r);
	void setDesiredSpeed(const double v_des);
	void setR_ind(const double R_induction);
	void setR_vis(const double R_visibility);
	void setPosition(const Vector2d p);
	void setVelocity(const Vector2d v);
	void setDesiredDirection(const Vector2d e);
	
	double getMass() const { return mass; }
	double getRadius() const { return radius; }
	double getDesiredSpeed() const { return desiredSpeed; }
	double getR_ind() const { return R_ind; }
	double getR_vis() const { return R_vis; }
	Vector2d getPosition() const { return position; }
	Vector2d getVelocity() const { return velocity; }
	Vector2d getDesiredDirection() const { return desiredDirection; }	
	
	void move(std::vector<Agent>& agents, Room roomData, const double stepTime);	
};

void setInitialPosition(std::vector<Agent>& agents, Room roomData);

//guide用初期配置関数
void setInitialPosition_g(const Room room, std::vector<Agent>& guide);
//evacuee用初期配置関数
void setInitialPosition_e(const Room room, const std::vector<Agent>& guide, std::vector<Agent>& evacuee);


void removeAgent(std::vector<Agent>& agents, Room roomData);

#endif


/*
* ヘッダファイル内に using namepsace，using を記述することを禁止する．ヘッダにそれらを記述するとヘッダ読み込んだソースにまで影響され，思わぬ障害が出る．ヘッダに using namespace を記述すると名前空間の意味が薄れる．
*/
