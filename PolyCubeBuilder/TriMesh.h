/*
* Author: tiny656
* Date: 2015-10-12
*/
#pragma once

#include "utility.h"


class Point;
class Face;
class Vertex;

#pragma region Mesh网格模型
/* mesh网格模型 */
class TriMesh {
public:
	/*
	* Constructor
	*/
	TriMesh() {}

	/*
	* Method
	*/
	void addVertex(int vertexId, const Eigen::Vector3d &point); // 添加顶点
	void addFace(int faceId, const Eigen::Vector3i &faceIndex, const Eigen::Vector3d& faceNormal); // 添加面

	size_t nFaces() const; // 返回面的数量
	size_t nVertices() const; // 返回顶点的数量

	const concurrent_unordered_map<int, shared_ptr<Vertex>>& allVertexs() const;
	concurrent_unordered_map<int, shared_ptr<Vertex>>& allVertexs();
	const concurrent_unordered_map<int, shared_ptr<Face>>& allFaces() const;
	concurrent_unordered_map<int, shared_ptr<Face>>& allFaces();

	const shared_ptr<Vertex> getVertex(int index) const; // 返回id为index的顶点
	const shared_ptr<Face> getFace(int index) const; // 返回id为index的面

private:
	/*
	* Variables
	*/
	concurrent_unordered_map<int, shared_ptr<Vertex>> m_vertex; // 顶点
	concurrent_unordered_map<int, shared_ptr<Face>> m_face; // 面
};
#pragma endregion

#pragma region 面
/* 面 */
class Face {
public:
	/*
	* Constructor
	*/
	Face(int faceId, const Eigen::Vector3i &faceIndex, const Eigen::Vector3d& faceNormal, TriMesh *mesh);

	/*
	* Method
	*/
	int id() const; // 返回面的id
	const Eigen::Vector3i& getVertexIndex() const; // 返回三角面片三个顶点的索引
	const set<int>& getAdjacentFace() const; // 返回相邻面
	void addAdjacentFace(int faceId); // 添加相邻面id
	const Eigen::Vector3d& normal() const; // 返回面的单元法向，不可修改
	Eigen::Vector3d& normal(); // 返回面的单元法向，可修改
	double area() const; // 返回三角面片的面积

	const TriMesh* getTriMesh() const;

private:
	/*
	* Internal Method
	*/
	set<int> internalFindCommonFace(const Eigen::Vector3i &faceVertexId) const; // 寻找两个顶点的相邻面

	/*
	* Variables
	*/
	TriMesh *m_mesh; // 所属的Mesh
	int m_faceId; // 面ID
	Eigen::Vector3i m_faceVertexId; // 面上三个顶点索引
	Eigen::Vector3d m_faceNormal; // 面的法向，朝向体外
	set<int> m_adjacentFaceId; // 相邻面的id
};
#pragma endregion

#pragma region 顶点
/* 顶点 */
class Vertex {
public:
	/*
	* Constructor
	*/
	Vertex(int vertexId, const Eigen::Vector3d &point, TriMesh *mesh);

	/*
	* Method
	*/
	int id() const; // 返回顶点的id

	const concurrent_vector<int>& getAdjancentVertexId(); // 返回当前顶点的相邻点id - 一环邻域点
	const set<int>& getFaceId() const; // 返回当前顶点所属面集合的id

	void addAdjacentFace(int faceId); // 添加邻近面
	const Eigen::Vector3d& point() const; // 返回几何坐标，不可修改
	Eigen::Vector3d& point(); // 返回几何坐标

	const TriMesh* getTriMesh() const;

private:
	/*
	* Variables
	*/
	TriMesh *m_mesh; // 所属的mesh
	Eigen::Vector3d m_point; // 顶点几何位置
	int m_id; // 顶点ID
	set<int> m_adjacentFaceId; // 相邻面的id
	concurrent_vector<int> m_adjacentVertexId; // 有序的一环领域

	
};
#pragma endregion