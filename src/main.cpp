#include <fstream>
#include <iostream>

#include "Mesh.h"
#include "ClosestPointQuery.h"
#include "Node.h"

int main(int argc, char *argv[])
{
	// Get query inputs
	const char * filename = argv[1]; // mesh obj file
	const char * pointValues = argv[2]; // txt file containing point values to query
	float maxDist = std::atof(argv[3]); // maximum search distance
	Mesh * mesh = new Mesh(filename);
	ClosestPointQuery * query = new ClosestPointQuery(*mesh);
	
	// Go through each query point inside the given file.
	std::ifstream input(pointValues);
	float x, y, z;
	char sep;
	while(input >> x >> sep >> y >> sep >> z)
	{
		Point queryPoint(x, y, z);
		Point result;
		bool found = (*query)(queryPoint, maxDist, result);
		if(!found)
			fprintf(stdout, "Could not find closest point to %f %f %f within distance %f\n",
				queryPoint[0], queryPoint[1], queryPoint[2], maxDist);
		else
			fprintf(stdout, "Closest point to %f %f %f within distance %f is: %f %f %f\n",
				queryPoint[0], queryPoint[1], queryPoint[2], maxDist,
				result[0], result[1], result[2]);
	}

	return 0;
}
