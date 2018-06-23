#include <algorithm>
#include <iostream>
#include "Node.h"

void Node::build(TriangleVect::iterator first, TriangleVect::iterator last, std::size_t maxLeafSize)
{
	// Gather all points within the given range
	std::vector<Point> allPoints;
	for (TriangleVect::iterator it = first; it != last; it++)
	{
		allPoints.push_back(it->p1); 
		allPoints.push_back(it->p2); 
		allPoints.push_back(it->p3);
	}

	// compute bounding box of all points
	Point bMin = allPoints[0];
	Point bMax = allPoints[0];
	for(unsigned i = 1; i < allPoints.size(); i++)
	{
		bMin = bMin.cwiseMin(allPoints[i]);
		bMax = bMax.cwiseMax(allPoints[i]);
	}

	m_bbox.bboxMin = bMin;
	m_bbox.bboxMax = bMax;
	
	// Find longest axis
	int sortAxis = maxAxis(bMax[0]-bMin[0], bMax[1]-bMin[1], bMax[2]-bMin[2]);

	// Order triangles along longest axis
	TriangleVect::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last, AxisSort(sortAxis));

	if(maxLeafSize == 2)
	{
		// This is a leaf. Children now hold the actual triangles.
		m_leftChild = &(*first);
		m_rightChild = &(*(++first));
	}
	else if(maxLeafSize == 3)
	{
		// Left is leaf. Keep building on the right.
		m_leftChild = &(*first);
		m_rightChild = static_cast<Node*>(this) + 1;
		rightChild().build(first + 1, last, 2);
	}
	else
	{
		// Keep building both the left and right children
		const std::size_t newMaxLeafSize = maxLeafSize / 2;
		m_leftChild = static_cast<Node*>(this) + 1;
		m_rightChild = static_cast<Node*>(this) + newMaxLeafSize;
		leftChild().build(first, first + newMaxLeafSize, newMaxLeafSize);
		rightChild().build(first + newMaxLeafSize, last, maxLeafSize - newMaxLeafSize);
	}
}

void Node::walk(const Point & queryPoint, const float & maxDist, const std::size_t numTriangles, float &closestDistanceSqrd, Point& result) const
{
	Point tmp;
	if (numTriangles == 2)
	{
		// At a leaf. Check for a closest point within range first on the left side, then right.
		tmp = closestPointOnTriangle(leftTriangle(), queryPoint);
		if (closestDistanceSqrd >= (tmp - queryPoint).squaredNorm())
		{
			closestDistanceSqrd = (tmp - queryPoint).squaredNorm();
			result = tmp;
		}
		else
		{
			tmp = closestPointOnTriangle(rightTriangle(), queryPoint);
			if (closestDistanceSqrd >= (tmp - queryPoint).squaredNorm())
			{
				closestDistanceSqrd = (tmp - queryPoint).squaredNorm();
				result = tmp;
			}
		}
	}
	else if (numTriangles == 3)
	{
		// At a leaf on the left side, check for closest point.
		tmp = closestPointOnTriangle(leftTriangle(), queryPoint);
		if (closestDistanceSqrd >= (tmp - queryPoint).squaredNorm())
		{
			closestDistanceSqrd = (tmp - queryPoint).squaredNorm();
			result = tmp;
		}
		else
		{
			// Keep walking down the right box if the query sphere intersects with it.
			if (intersects(rightChild().m_bbox.bboxMin, rightChild().m_bbox.bboxMax, queryPoint, maxDist))
				rightChild().walk(queryPoint, maxDist, 2, closestDistanceSqrd, result);
		}
	}
	else
	{
		// Check for an intersection between the left box and the query sphere. If not found, look to the other side.
		if(intersects(leftChild().m_bbox.bboxMin, leftChild().m_bbox.bboxMax, queryPoint, maxDist))
		{
			leftChild().walk(queryPoint, maxDist, numTriangles / 2, closestDistanceSqrd, result);
			if(intersects(rightChild().m_bbox.bboxMin, rightChild().m_bbox.bboxMax, queryPoint, maxDist))
				rightChild().walk(queryPoint, maxDist, numTriangles - numTriangles / 2, closestDistanceSqrd, result);
		}
		else if(intersects(rightChild().m_bbox.bboxMin, rightChild().m_bbox.bboxMax, queryPoint, maxDist))
			rightChild().walk(queryPoint, maxDist, numTriangles - numTriangles / 2, closestDistanceSqrd, result);
	}
}