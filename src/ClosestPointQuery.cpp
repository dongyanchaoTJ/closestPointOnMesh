#include "ClosestPointQuery.h"
#include "Mesh.h"
#include <iostream>

ClosestPointQuery::ClosestPointQuery(const Mesh & mesh)
{
	m_numFaces = mesh.numFaces();
	for(int i = 0; i<m_numFaces; i++)
	{
		Triangle triangle(mesh.V(i, 0), mesh.V(i, 1), mesh.V(i, 2));
		m_triangles.push_back(triangle);
	}
	// Build a tree to accelerate searches
	m_root = new Node[m_numFaces - 1]();
	if(m_numFaces>1)
		m_root->build(m_triangles.begin(), m_triangles.end(), m_numFaces);
}

bool ClosestPointQuery::operator()(const Point & queryPoint, float maxDist, Point& closestPoint) const
{
	float squaredMinDist = pow(maxDist, 2.0f) + 2.0f;
	// Walk down the tree to find the closest point within the max search distance
	m_root->walk(queryPoint, maxDist, m_numFaces, squaredMinDist, closestPoint);
	// Check if the search was successful
	if ((closestPoint - queryPoint).squaredNorm() > squaredMinDist)
		return false;
	return true;
}
