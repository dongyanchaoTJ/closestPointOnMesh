#ifndef __MESH_H__
#define __MESH_H__

#include <Eigen/Core>

/// Simple Mesh class initialized using an obj file.
class Mesh 
{
	public:

		Mesh(const char* filename);
		~Mesh() {};
		const int& numFaces() const { return m_numFaces; };
		Eigen::Vector3f V(int t, int vt) const {return m_vertices.row(m_faces(t, vt));}

	private:

		Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> m_vertices;
		Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> m_faces;
		int m_numFaces;
};

#endif