/*
* Author: tiny656
* Date: 2015-11-24
*/
#pragma once

#include "utility.h"
#include "TriMesh.h"

class TetNode; // 四面体网格节点
class Tetrahedra; // 四面体网格单元

class TetMesh {
public:
	TetMesh() {}

	void addNode(int nodeId, const Eigen::Vector3d& point);
	void addTetElement(int tetId, const Eigen::Vector4i& tetNodeId);

	const concurrent_unordered_map<int, shared_ptr<TetNode>>& allNodes() const;
	concurrent_unordered_map<int, shared_ptr<TetNode>>& allNodes();
	const concurrent_unordered_map<int, shared_ptr<Tetrahedra>>& allTetElements() const;
	concurrent_unordered_map<int, shared_ptr<Tetrahedra>>& allTetElements();

	const shared_ptr<Tetrahedra> getTetElement(int tetId) const;
	const shared_ptr<TetNode> getNode(int nodeId) const;
	void buildSurfaceTriMesh(TriMesh &surfaceTriMesh); // 生成表面TriMesh

	size_t nNodes() const; // 返回所有四面体网格顶点数量
	size_t nTets() const; // 返回所有的四面体数量

private:
	concurrent_unordered_map<int, shared_ptr<TetNode>> m_node; // 所有的坐标点
	concurrent_unordered_map<int, shared_ptr<Tetrahedra>> m_tetElement; // 所有的四面体单元
};

class TetNode {
public:
	TetNode(int nodeId, const Eigen::Vector3d &point, TetMesh *tetMesh);
	const Eigen::Vector3d& point() const; // 返回几何坐标，不可修改
	Eigen::Vector3d& point(); // 返回几何坐标
	int id() const; // 返回顶点id
	bool onSurface() const; // 是否在表面
	void setSurfaceState(bool state); // 设置是否在表面
	const concurrent_vector<int>& getAdjTetId() const; // 返回当前点的相邻四面体的id
	void addAdjTetId(int tetId); // 添加相邻的四面体的Id

private:
	TetMesh *m_tetMesh;
	bool m_onSurface; // 当前点是否在表面
	int m_id; // 点的id
	Eigen::Vector3d m_point; // 顶点坐标
	concurrent_vector<int> m_adjTetId; // 相邻四面体的id
};

class Tetrahedra {
public:
	Tetrahedra(int tetId, const Eigen::Vector4i &tetNodeIdVec, TetMesh *tetMesh);
	Eigen::Vector3i getTriangleId(int nodeId); // 返回除去这个点的三个点形成的三角面片的点的id
	Eigen::Vector4i getTetNodeId() const; // 返回四面体的四个点
	int id() const; // 返回四面体的id
private:
	TetMesh *m_tetMesh; 
	int m_id; // 四面体的编号
	Eigen::Vector4i m_tetNodeIdVec; // 四面体上四个点的id
};