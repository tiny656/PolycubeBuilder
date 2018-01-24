/*
* Author: tiny656
* Date: 2015-10-12
*/
#pragma once

#include "utility.h"


class Point;
class Face;
class Vertex;

#pragma region Mesh����ģ��
/* mesh����ģ�� */
class TriMesh {
public:
	/*
	* Constructor
	*/
	TriMesh() {}

	/*
	* Method
	*/
	void addVertex(int vertexId, const Eigen::Vector3d &point); // ��Ӷ���
	void addFace(int faceId, const Eigen::Vector3i &faceIndex, const Eigen::Vector3d& faceNormal); // �����

	size_t nFaces() const; // �����������
	size_t nVertices() const; // ���ض��������

	const concurrent_unordered_map<int, shared_ptr<Vertex>>& allVertexs() const;
	concurrent_unordered_map<int, shared_ptr<Vertex>>& allVertexs();
	const concurrent_unordered_map<int, shared_ptr<Face>>& allFaces() const;
	concurrent_unordered_map<int, shared_ptr<Face>>& allFaces();

	const shared_ptr<Vertex> getVertex(int index) const; // ����idΪindex�Ķ���
	const shared_ptr<Face> getFace(int index) const; // ����idΪindex����

private:
	/*
	* Variables
	*/
	concurrent_unordered_map<int, shared_ptr<Vertex>> m_vertex; // ����
	concurrent_unordered_map<int, shared_ptr<Face>> m_face; // ��
};
#pragma endregion

#pragma region ��
/* �� */
class Face {
public:
	/*
	* Constructor
	*/
	Face(int faceId, const Eigen::Vector3i &faceIndex, const Eigen::Vector3d& faceNormal, TriMesh *mesh);

	/*
	* Method
	*/
	int id() const; // �������id
	const Eigen::Vector3i& getVertexIndex() const; // ����������Ƭ�������������
	const set<int>& getAdjacentFace() const; // ����������
	void addAdjacentFace(int faceId); // ���������id
	const Eigen::Vector3d& normal() const; // ������ĵ�Ԫ���򣬲����޸�
	Eigen::Vector3d& normal(); // ������ĵ�Ԫ���򣬿��޸�
	double area() const; // ����������Ƭ�����

	const TriMesh* getTriMesh() const;

private:
	/*
	* Internal Method
	*/
	set<int> internalFindCommonFace(const Eigen::Vector3i &faceVertexId) const; // Ѱ�����������������

	/*
	* Variables
	*/
	TriMesh *m_mesh; // ������Mesh
	int m_faceId; // ��ID
	Eigen::Vector3i m_faceVertexId; // ����������������
	Eigen::Vector3d m_faceNormal; // ��ķ��򣬳�������
	set<int> m_adjacentFaceId; // �������id
};
#pragma endregion

#pragma region ����
/* ���� */
class Vertex {
public:
	/*
	* Constructor
	*/
	Vertex(int vertexId, const Eigen::Vector3d &point, TriMesh *mesh);

	/*
	* Method
	*/
	int id() const; // ���ض����id

	const concurrent_vector<int>& getAdjancentVertexId(); // ���ص�ǰ��������ڵ�id - һ�������
	const set<int>& getFaceId() const; // ���ص�ǰ���������漯�ϵ�id

	void addAdjacentFace(int faceId); // ����ڽ���
	const Eigen::Vector3d& point() const; // ���ؼ������꣬�����޸�
	Eigen::Vector3d& point(); // ���ؼ�������

	const TriMesh* getTriMesh() const;

private:
	/*
	* Variables
	*/
	TriMesh *m_mesh; // ������mesh
	Eigen::Vector3d m_point; // ���㼸��λ��
	int m_id; // ����ID
	set<int> m_adjacentFaceId; // �������id
	concurrent_vector<int> m_adjacentVertexId; // �����һ������

	
};
#pragma endregion