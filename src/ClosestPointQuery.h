#include "Node.h"
#include "Mesh.h"

class ClosestPointQuery
{
	public:

		ClosestPointQuery(const Mesh& mesh);
	
		//!finds the closest point to mesh within the specified maximum search distance.
		//!returns whether the search was successful or not.
		bool operator()(const Point& queryPoint, float maxDist, Point& closestPoint) const;

	private:
		Node * m_root; // root of the tree built to make fast distance queries
		int m_numFaces;
		TriangleVect m_triangles;
};