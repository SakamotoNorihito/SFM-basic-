#include"Room.h"
#include <vector>
using namespace std;

Room::Room()
{
	room_size_x = 0;
	room_size_y = 0;
	width_exit = 0;
}

void Room::setRoom_size_x(const double x)
{
	this->room_size_x = x;
}

void Room::setRoom_size_y(const double y)
{
	this->room_size_y = y;
}

void Room::setWidth_exit(const double w)
{
	this->width_exit = w;
}

vector<vector<Vector2d>> Room::createWall()
{
	vector<vector<Vector2d>> wallCornerPoint(5, vector<Vector2d>(2));

	wallCornerPoint[0][0] = Vector2d(room_size_x, width_exit / 2);
	wallCornerPoint[0][1] = Vector2d(room_size_x, room_size_y / 2);

	wallCornerPoint[1][0] = Vector2d(room_size_x, room_size_y / 2);
	wallCornerPoint[1][1] = Vector2d(0, room_size_y / 2);

	wallCornerPoint[2][0] = Vector2d(0, room_size_y / 2);
	wallCornerPoint[2][1] = Vector2d(0, -room_size_y / 2);

	wallCornerPoint[3][0] = Vector2d(0, -room_size_y / 2);
	wallCornerPoint[3][1] = Vector2d(room_size_x, -room_size_y / 2);

	wallCornerPoint[4][0] = Vector2d(room_size_x, -room_size_y / 2);
	wallCornerPoint[4][1] = Vector2d(room_size_x, -width_exit / 2);

	return wallCornerPoint;
}
