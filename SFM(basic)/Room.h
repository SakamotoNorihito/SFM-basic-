#ifndef Room_H
#define Room_H

#include<vector>
#include"Vector2d_math.h"

class Room {
private:
	double room_size_x;
	double room_size_y;
	double width_exit;

public:
	Room();

	void setRoom_size_x(const double x);
	void setRoom_size_y(const double y);
	void setWidth_exit(const double w);

	double getRoom_size_x() const { return room_size_x; }
	double getRoom_size_y() const { return room_size_y; }
	double getWidth_exit() const { return width_exit; }

	std::vector<std::vector<Vector2d>> createWall();
};

#endif