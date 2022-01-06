#pragma once

#include <iostream>
#include <vector>
#include <list>
// Libraries
#include <cmath>
#include <cassert>
#include <limits>

using namespace std;


class Vector3
{
public:

	// -------------------- Attributes -------------------- //

	// Components of the vector
	float x, y, z;

	// -------------------- Methods -------------------- //

	// Constructor
	Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

	// Constructor
	Vector3(const Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {}

	// Constructor
	~Vector3() {}

	// = operator
	Vector3& operator=(const Vector3& vector) {
		if (&vector != this) {
			x = vector.x;
			y = vector.y;
			z = vector.z;
		}
		return *this;
	}

	// + operator
	Vector3 operator+(const Vector3 &v) const {
		return Vector3(x + v.x, y + v.y, z + v.z);
	}

	// += operator
	Vector3& operator+=(const Vector3 &v) {
		x += v.x; y += v.y; z += v.z;
		return *this;
	}

	// - operator
	Vector3 operator-(const Vector3 &v) const {
		return Vector3(x - v.x, y - v.y, z - v.z);
	}

	// -= operator
	Vector3& operator-=(const Vector3 &v) {
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}

	// == operator
	bool operator==(const Vector3 &v) const {
		return x == v.x && y == v.y && z == v.z;
	}

	// != operator
	bool operator!=(const Vector3 &v) const {
		return !(*this == v);
	}

	// * operator
	Vector3 operator*(float f) const {
		return Vector3(f*x, f*y, f*z);
	}

	// *= operator
	Vector3 &operator*=(float f) {
		x *= f; y *= f; z *= f;
		return *this;
	}

	// / operator
	Vector3 operator/(float f) const {
		assert(f > std::numeric_limits<float>::epsilon());
		float inv = 1.f / f;
		return Vector3(x * inv, y * inv, z * inv);
	}

	// /= operator
	Vector3 &operator/=(float f) {
		assert(f > std::numeric_limits<float>::epsilon());
		float inv = 1.f / f;
		x *= inv; y *= inv; z *= inv;
		return *this;
	}

	// - operator
	Vector3 operator-() const {
		return Vector3(-x, -y, -z);
	}

	// [] operator
	float &operator[](int i) {
		assert(i >= 0 && i <= 2);
		switch (i) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		}
		return z;
	}

	// [] operator
	const float &operator[](int i) const {
		assert(i >= 0 && i <= 2);
		switch (i) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		}
		return z;
	}

	// Cross product operator
	Vector3 cross(const Vector3 &v) const {
		return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

	// Dot product operator
	float dot(const Vector3 &v) const {
		return x * v.x + y * v.y + z * v.z;
	}

	// Normalize the vector and return it
	Vector3 normalize() {
		float l = length();
		if (l < std::numeric_limits<float>::epsilon()) {
			assert(false);
		}
		x /= l;
		y /= l;
		z /= l;
		return *this;
	}

	bool isNull() const {
		return(x == 0. && y == 0. && z == 0.);
	}

	// Clamp the values between 0 and 1
	Vector3 clamp01() {
		if (x > 1.f) x = 1.f;
		else if (x < 0.f) x = 0.f;
		if (y > 1.f) y = 1.f;
		else if (y < 0.f) y = 0.f;
		if (z > 1.f) z = 1.f;
		else if (z < 0.f) z = 0.f;
		return *this;
	}

	// Return the squared length of the vector
	float lengthSquared() const { return x * x + y * y + z * z; }

	// Return the length of the vector
	float length() const { return sqrt(lengthSquared()); }
};

class BoundingBox
{
public:
	float m_top;
	float m_bottom;
	float m_left;
	float m_right;

	float m_radius;

public:
	void  SetRadius(float radius);
	float GetRadius() { return m_radius; }
};

#define LOOSE_FACTOR 1
class OctreeNode {

public:
	enum eValue {
		Depth_Limit = 4,                     //노드의 세부 수준을 4단계로 제한 합니다.
		Width = 1000
	};                       //최초 노드의 가로 크기


	BoundingBox*                   m_pBoundBox;                          //현재 노드의 바운딩 박스
	Vector3                        m_Pos;                        

	vector< OctreeNode* >       m_vChildren;                            //자식 노드

	//list< Object* >                   m_listObjects;                          //현재 노드에 포함되는 정적 오브젝트 목록
	bool                               m_bCulled;                               // 이 노드가 컬링 되었는지 여부 
	OctreeNode*                        m_pParent;                                 //부모 노드

	void AddChildNode(OctreeNode* node);                                  // 이 노드에 자식 노드를 추가합니다.
	//bool AddObject(Object*);                                               // 이 노드에 오브젝트를 추가합니다.
	void CullNode(bool);                                                     // 이 노드를 컬링합니다.
	vector<OctreeNode*> const GetChildNode(size_t);                          //자식 노드를 가져옵니다.
	void SetParent(OctreeNode* const);                                 //부모 노드를 설정합니다.
	OctreeNode* const GetParent();                                          //부모 노드를 반환합니다.

	//bool IsInNode(const GLTVector3);                                  //현재 노드에 해당 정점이 포함되는지 판단합니다.
	void Render();                                                                 //이 노드의 물체들을 렌더링합니다.
	bool IsCulled();                                                             // 이 노드가 컬링됬는지 판단합니다.

	OctreeNode* BuildOctree(Vector3 vCenter, float fHalfWidth, int depthLimit);

public:
	BoundingBox* GetBoundingBox();
	Vector3 GetPosition() {
		return m_Pos;
	}
	void SetPosition(Vector3 vCenter) {
		m_Pos = vCenter;
	}

};
