#ifndef VECTOR2D_MATH_H
#define VECTOR2D_MATH_H

class Vector2d //2次元double型ベクトル
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

//関数のプロトタイプ宣言
double g(const double& x);
double distance(const Vector2d& P1, const Vector2d& P2);
double dotProduct(const Vector2d& P1, const Vector2d& P2);
Vector2d unitVector(const Vector2d& P1, const Vector2d& P2);
Vector2d tangentialVector(const Vector2d& obj);
Vector2d getNearestPoint(const Vector2d& A, const Vector2d& B, const Vector2d& P);

//等価演算子（==）の実装
//ベクトルとスカラーの除算（/）の実装
#endif
