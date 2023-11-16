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

	Vector2d f_driv;
	Vector2d f_ij;
	Vector2d f_iw;

	Vector2d position;
	Vector2d velocity;
	Vector2d desiredDirection;

	Vector2d drivingForce(const Room roomData);
	Vector2d agentInteractForce(const std::vector<Agent>& agents);
	Vector2d wallInteractForce(Room roomData);

public:
	Agent();

	void setRadius(const double r);
	void setPosition(const Vector2d p);
	void setVelocity(const Vector2d v);
	void setDesiredDirection(const Vector2d e);
	
	double getRadius() const { return radius; }
	Vector2d getF_driv() const { return f_driv; }
	Vector2d getF_ij() const { return f_ij; }
	Vector2d getF_iw() const { return f_iw; }
	Vector2d getPosition() const { return position; }
	Vector2d getVelocity() const { return velocity; }
	Vector2d getDesiredDirection() const { return desiredDirection; }	
	
	void move(std::vector<Agent>& agents, Room roomData, const double stepTime);	
};

void setInitialPosition(std::vector<Agent>& agents, Room roomData);
void removeAgent(std::vector<Agent>& agents, Room roomData);

#endif


/*
* ヘッダファイル内に using namepsace，using を記述することを禁止する．ヘッダにそれらを記述するとヘッダ読み込んだソースにまで影響され，思わぬ障害が出る．ヘッダに using namespace を記述すると名前空間の意味が薄れる．
*/
