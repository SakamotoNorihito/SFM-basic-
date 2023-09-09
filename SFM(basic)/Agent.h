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

	//guide�pdrivingForce
	Vector2d drivingForce_g(const Room room);
	//evacuee�pdrivingForce
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

//guide�p�����z�u�֐�
void setInitialPosition_g(const Room room, std::vector<Agent>& guide);
//evacuee�p�����z�u�֐�
void setInitialPosition_e(const Room room, const std::vector<Agent>& guide, std::vector<Agent>& evacuee);


void removeAgent(std::vector<Agent>& agents, Room roomData);

#endif


/*
* �w�b�_�t�@�C������ using namepsace�Cusing ���L�q���邱�Ƃ��֎~����D�w�b�_�ɂ������L�q����ƃw�b�_�ǂݍ��񂾃\�[�X�ɂ܂ŉe������C�v��ʏ�Q���o��D�w�b�_�� using namespace ���L�q����Ɩ��O��Ԃ̈Ӗ��������D
*/
