#ifndef VECTOR2D_MATH_H
#define VECTOR2D_MATH_H

class Vector2d //2����double�^�x�N�g��
{
public:
	double x, y;
	Vector2d();
	Vector2d(double a, double b);
	friend Vector2d operator*(const double& a, const Vector2d& obj);
	friend Vector2d operator+(const Vector2d& obj1, const Vector2d& obj2);
	friend Vector2d operator-(const Vector2d& obj1, const Vector2d& obj2);
	friend Vector2d operator/(const Vector2d& obj, const double& a);
};

//�֐��̃v���g�^�C�v�錾
double g(const double& x);
double distance(const Vector2d& P1, const Vector2d& P2);
double dotProduct(const Vector2d& P1, const Vector2d& P2);
Vector2d unitVector(const Vector2d& P1, const Vector2d& P2);
Vector2d tangentialVector(const Vector2d& obj);
Vector2d getNearestPoint(const Vector2d& A, const Vector2d& B, const Vector2d& P);

//�������Z�q�i==�j�̎���
//�x�N�g���ƃX�J���[�̏��Z�i/�j�̎���
#endif
