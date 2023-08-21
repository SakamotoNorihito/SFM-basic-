#include<iostream>
#include"Vector2d_math.h"
using namespace std;

//コンストラクタ
Vector2d::Vector2d()
{
	x = 0;
	y = 0;
}

//コンストラクタ
Vector2d::Vector2d(double a, double b)
{
	x = a;
	y = b;
}

//scalar * Vector
Vector2d operator*(const double& a, const Vector2d& obj)
{
	return Vector2d(a * obj.x, a * obj.y);
}

//Vector + Vector
Vector2d operator+(const Vector2d& obj1, const Vector2d& obj2)
{
	return Vector2d(obj1.x + obj2.x, obj1.y + obj2.y);
}

//Vector - Vector
Vector2d operator-(const Vector2d& obj1, const Vector2d& obj2)
{
	return Vector2d(obj1.x - obj2.x, obj1.y - obj2.y);
}

//g(x) = x * Heaviside(x)
double g(const double& x)
{
	return x < 0 ? 0 : x;
}

//点p1-点p2間の距離
double distance(const Vector2d& p1, const Vector2d& p2)
{
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

//内積
double dotProduct(const Vector2d& p1, const Vector2d& p2) 
{
	return (p1.x * p2.x) + (p1.y * p2.y);
}

//p1→p2方向の単位Vectorを生成
Vector2d unitVector(const Vector2d& p1, const Vector2d& p2) 
{
	double D = distance(p1, p2);

	if (D == 0)
	{
		cout << "単位ベクトルが0除算されています";
		exit(EXIT_FAILURE);		//異常終了
	}
	else
	{
		double L = 1 / D;
		return Vector2d(L * (p2.x - p1.x), L * (p2.y - p1.y));
	}
}

//Vectorを反時計回りに90°回転
Vector2d tangentialVector(const Vector2d& obj)
{
	return Vector2d(-obj.y, obj.x);
}

//線分AB-点P間の最近傍点Iの座標の算出
Vector2d getNearestPoint(const Vector2d& A, const Vector2d& B, const Vector2d& P)
{
	if (distance(A, B) == 0)
	{
		cout << "壁の端点の座標が重なっています";
		exit(EXIT_FAILURE);		//異常終了
	}
	else
	{
		Vector2d AP = P - A;
		Vector2d AB = B - A;
		Vector2d BP = P - B;
		Vector2d BA = -1 * AB;

		if (dotProduct(AP, AB) < 0)
		{
			return A;	//端点Aの座標
		}
		else if (dotProduct(BP, BA) < 0)
		{
			return B;	//端点Bの座標
		}
		else
		{
			Vector2d I;
			double L_AI = dotProduct(AP, AB) / distance(A, B);	//端点A-最近傍点Iの長さ
			I = A + (L_AI * unitVector(A, B));					//最近傍点Iの座標の算出
			return I;
		}
	}
}