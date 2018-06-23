#include "Mesh.h"
#include "igl/readOBJ.h"

Mesh::Mesh(const char* filename)
{
	igl::readOBJ(filename, m_vertices, m_faces);
	m_numFaces = m_faces.rows();
}
