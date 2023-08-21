#include<iostream>
#include"Vector2d_math.h"
using namespace std;

//�R���X�g���N�^
Vector2d::Vector2d()
{
	x = 0;
	y = 0;
}

//�R���X�g���N�^
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

//�_p1-�_p2�Ԃ̋���
double distance(const Vector2d& p1, const Vector2d& p2)
{
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

//����
double dotProduct(const Vector2d& p1, const Vector2d& p2) 
{
	return (p1.x * p2.x) + (p1.y * p2.y);
}

//p1��p2�����̒P��Vector�𐶐�
Vector2d unitVector(const Vector2d& p1, const Vector2d& p2) 
{
	double D = distance(p1, p2);

	if (D == 0)
	{
		cout << "�P�ʃx�N�g����0���Z����Ă��܂�";
		exit(EXIT_FAILURE);		//�ُ�I��
	}
	else
	{
		double L = 1 / D;
		return Vector2d(L * (p2.x - p1.x), L * (p2.y - p1.y));
	}
}

//Vector�𔽎��v����90����]
Vector2d tangentialVector(const Vector2d& obj)
{
	return Vector2d(-obj.y, obj.x);
}

//����AB-�_P�Ԃ̍ŋߖT�_I�̍��W�̎Z�o
Vector2d getNearestPoint(const Vector2d& A, const Vector2d& B, const Vector2d& P)
{
	if (distance(A, B) == 0)
	{
		cout << "�ǂ̒[�_�̍��W���d�Ȃ��Ă��܂�";
		exit(EXIT_FAILURE);		//�ُ�I��
	}
	else
	{
		Vector2d AP = P - A;
		Vector2d AB = B - A;
		Vector2d BP = P - B;
		Vector2d BA = -1 * AB;

		if (dotProduct(AP, AB) < 0)
		{
			return A;	//�[�_A�̍��W
		}
		else if (dotProduct(BP, BA) < 0)
		{
			return B;	//�[�_B�̍��W
		}
		else
		{
			Vector2d I;
			double L_AI = dotProduct(AP, AB) / distance(A, B);	//�[�_A-�ŋߖT�_I�̒���
			I = A + (L_AI * unitVector(A, B));					//�ŋߖT�_I�̍��W�̎Z�o
			return I;
		}
	}
}