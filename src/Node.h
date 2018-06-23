#ifndef __NODE_H__
#define __NODE_H__

#include <Eigen/Core>
#include <algorithm>
#include <vector>

typedef Eigen::Vector3f Point;

struct Triangle
{
	Triangle() {};
	Triangle(Point a, Point b, Point c): p1(a), p2(b), p3(c) {};
	Point p1;
	Point p2;
	Point p3;
};

typedef std::vector<Triangle> TriangleVect;

struct Bbox 
{
	Point bboxMin;
	Point bboxMax;
	Triangle triangle;
};

/// The Node class builds a hierarchy of axis-aligned bounding boxes (AABB Tree)
/// which are used to speed up intersection and distance queries
class Node
{
	public:

		Node() : m_bbox(), m_leftChild(NULL), m_rightChild(NULL) {};
		~Node(){};

		const Bbox& bbox() const { return m_bbox; }

		// Queries for the left and right children when on a branch
		const Node& leftChild() const { return *static_cast<Node*>(m_leftChild); }
		Node& leftChild() { return *static_cast<Node*>(m_leftChild); }
		const Node& rightChild() const { return *static_cast<Node*>(m_rightChild); }
		Node& rightChild() { return *static_cast<Node*>(m_rightChild); }

		// Queries for the left and right data held by the node, when reaching a leaf
		const Triangle& leftTriangle() const { return *static_cast<Triangle*>(m_leftChild); }
		Triangle& leftTriangle() { return *static_cast<Triangle*>(m_leftChild); }
		const Triangle& rightTriangle() const { return *static_cast<Triangle*>(m_rightChild); }
		Triangle& rightTriangle() { return *static_cast<Triangle*>(m_rightChild); }

		//! Recursive tree building
		void build(TriangleVect::iterator first, TriangleVect::iterator last, std::size_t maxLeafSize);
		//! Recursive tree parsing for queries
		void walk(const Point& queryPoint, const float& radius, const std::size_t numTriangles, float & closestDistanceSqrd, Point& result) const;

	private:

		Bbox m_bbox;
		void *m_leftChild;
		void *m_rightChild;
};

/// Convenience class to compare triangles along a given axis
class AxisSort
{
	public:
		AxisSort(unsigned int axis) : m_axis(axis){}

		bool operator()(Triangle t1, Triangle t2)
		{
			return t1.p1[m_axis] < t2.p1[m_axis];
		}

	private:
		const unsigned int m_axis;
};

//! Returns the longest axis
inline int maxAxis(float x, float y, float z)
{
	float max = x>y ? x : y;
	max = max>z ? max : z;
	if (max == x) return 0;
	if (max == y) return 1;
	if (max == z) return 2;
	return 0;
}

//! Returns the closest point from given input p on the given triangle.
inline Point closestPointOnTriangle(const Triangle & triangle, const Point & p)
{
	Point e0 = triangle.p2 - triangle.p1;
	Point e1 = triangle.p3 - triangle.p1;
	Point p0 = triangle.p1 - p;

	float a = e0.dot(e0);
	float b = e0.dot(e1);
	float c = e1.dot(e1);
	float d = e0.dot(p0);
	float e = e1.dot(p0);

	float det = a * c - pow(b, 2.0f);
	float s = b * e - c * d;
	float t = b * d - a * e;

	if((s + t) < det)
	{
		if(s<0.f)
		{
			if(t<0.f)
			{
				if(d<0.f)
				{
					s = std::min(0.f, std::max(-d / a, 1.f));
					t = 0.f;
				}
				else
				{
					s = 0.f;
					t = std::min(0.f, std::max(-e / c, 1.f));
				}
			}
			else
			{
				s = 0.f;
				t = std::min(0.f, std::max(-e / c, 1.f));
			}
		}
		else if(t<0.f)
		{
			s = std::min(0.f, std::max(-d/a, 1.f));
			t = 0.f;
		}
		else
		{
			float invDet = 1.f/det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		if(s<0.f)
		{
			float tmp0 = b + d;
			float tmp1 = c + e;
			if(tmp1 > tmp0)
			{
				float numer = tmp1 - tmp0;
				float denom = a - 2 * b + c;
				s = std::min(0.f, std::max(numer / denom, 1.f));
				t = 1 - s;
			}
			else
			{
				t = std::min(0.f, std::max(-e / c, 1.f));
				s = 0.f;
			}
		}
		else if(t < 0.f)
		{
			if((a + d) >(b + e))
			{
				float numer = c + e - b - d;
				float denom = a - 2 * b + c;
				s = std::min(0.f, std::max(numer / denom, 1.f));
				t = 1 - s;
			}
			else
			{
				s = std::min(0.f, std::max(-e / c, 1.f));
				t = 0.f;
			}
		}
		else
		{
			float numer = c + e - b - d;
			float denom = a - 2 * b + c;
			s = std::min(0.f, std::max(numer / denom, 1.f));
			t = 1.f - s;
		}
	}
	return triangle.p1 + s * e0 + t * e1;
}

//!Returns whether the given bounding box and sphere intersect.
inline bool intersects(const Point & bMin, const Point & bMax, const Point & sphereCenter, const float & sphereRadius)
{
	Point maxSphereBox = Point(bMin[0]>sphereCenter[0] ? bMin[0] : sphereCenter[0],
		bMin[1]>sphereCenter[1] ? bMin[1] : sphereCenter[1],
		bMin[2]>sphereCenter[2] ? bMin[2] : sphereCenter[2]);
	Point closestPointInBox = Point(maxSphereBox[0]<bMax[0] ? maxSphereBox[0] : bMax[0],
		maxSphereBox[1]<bMax[1] ? maxSphereBox[1] : bMax[1],
		maxSphereBox[2]<bMax[2] ? maxSphereBox[2] : bMax[2]);
	float dist = (sphereCenter - closestPointInBox).squaredNorm();
	return dist<pow(sphereRadius, 2.0);
}

#endif