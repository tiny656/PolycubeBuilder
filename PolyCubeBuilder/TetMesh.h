/*
* Author: tiny656
* Date: 2015-11-24
*/
#pragma once

#include "utility.h"
#include "TriMesh.h"

class TetNode; // ����������ڵ�
class Tetrahedra; // ����������Ԫ

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
	void buildSurfaceTriMesh(TriMesh &surfaceTriMesh); // ���ɱ���TriMesh

	size_t nNodes() const; // �����������������񶥵�����
	size_t nTets() const; // �������е�����������

private:
	concurrent_unordered_map<int, shared_ptr<TetNode>> m_node; // ���е������
	concurrent_unordered_map<int, shared_ptr<Tetrahedra>> m_tetElement; // ���е������嵥Ԫ
};

class TetNode {
public:
	TetNode(int nodeId, const Eigen::Vector3d &point, TetMesh *tetMesh);
	const Eigen::Vector3d& point() const; // ���ؼ������꣬�����޸�
	Eigen::Vector3d& point(); // ���ؼ�������
	int id() const; // ���ض���id
	bool onSurface() const; // �Ƿ��ڱ���
	void setSurfaceState(bool state); // �����Ƿ��ڱ���
	const concurrent_vector<int>& getAdjTetId() const; // ���ص�ǰ��������������id
	void addAdjTetId(int tetId); // ������ڵ��������Id

private:
	TetMesh *m_tetMesh;
	bool m_onSurface; // ��ǰ���Ƿ��ڱ���
	int m_id; // ���id
	Eigen::Vector3d m_point; // ��������
	concurrent_vector<int> m_adjTetId; // �����������id
};

class Tetrahedra {
public:
	Tetrahedra(int tetId, const Eigen::Vector4i &tetNodeIdVec, TetMesh *tetMesh);
	Eigen::Vector3i getTriangleId(int nodeId); // ���س�ȥ�������������γɵ�������Ƭ�ĵ��id
	Eigen::Vector4i getTetNodeId() const; // ������������ĸ���
	int id() const; // �����������id
private:
	TetMesh *m_tetMesh; 
	int m_id; // ������ı��
	Eigen::Vector4i m_tetNodeIdVec; // ���������ĸ����id
};