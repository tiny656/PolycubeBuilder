/*
* Author: tiny656
* Date: 2015-11-24
*/

#include "TetMesh.h"

void TetMesh::buildSurfaceTriMesh(TriMesh &surfaceTriMesh) {
	// �������е������������ҵ����治�ظ���
	concurrent_unordered_map<string, set<int>> faceCount; // ��¼ÿ��������Ƭ�����ڵ�����������,ת����string����hash
	#pragma omp parallel for
	for (int tetId = 1; tetId <= this->nTets(); tetId++) {
		auto tetElement = this->getTetElement(tetId); // ��ǰ�����嵥Ԫ
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId(); // ������������ĸ�
		
		concurrent_vector<int> tmpVec;
		tmpVec.push_back(tetNodeId.x());
		tmpVec.push_back(tetNodeId.y());
		tmpVec.push_back(tetNodeId.z());
		tmpVec.push_back(tetNodeId.w());

		sort(tmpVec.begin(), tmpVec.end());
		string faceA = to_string(tmpVec[0]) + "," + to_string(tmpVec[1]) + "," + to_string(tmpVec[2]); faceCount[faceA].insert(tetId);
		string faceB = to_string(tmpVec[0]) + "," + to_string(tmpVec[1]) + "," + to_string(tmpVec[3]); faceCount[faceB].insert(tetId);
		string faceC = to_string(tmpVec[0]) + "," + to_string(tmpVec[2]) + "," + to_string(tmpVec[3]); faceCount[faceC].insert(tetId);
		string faceD = to_string(tmpVec[1]) + "," + to_string(tmpVec[2]) + "," + to_string(tmpVec[3]); faceCount[faceD].insert(tetId);
	}

	for (const auto &it : faceCount) {
		// �ҵ�set��С����1���棬���Ǳ���
		if (it.second.size() == 1) {
			// ����õ�����������
			string faceStr = it.first;
			string::size_type pos1 = faceStr.find(','), pos2 = faceStr.find_last_of(',');
			Eigen::Vector3i faceNodeId(stoi(faceStr.substr(0, pos1)), stoi(faceStr.substr(pos1 + 1, pos2)), stoi(faceStr.substr(pos2 + 1)));

			// ��ӵ�����
			#pragma omp parallel for
			for (int i = 0; i < 3; i++) {
				int nodeId = faceNodeId(i);
				auto node = this->getNode(nodeId);
				// ���Ϊ�����
				node->setSurfaceState(true);
				if (surfaceTriMesh.getVertex(nodeId) == NULL)
					surfaceTriMesh.addVertex(nodeId, node->point());
			}

			// ���㵱ǰ����ķ���
			auto tetElement = this->getTetElement(*(it.second.begin()));
			auto tmpNodeId = tetElement->getTetNodeId().sum() - faceNodeId.sum();
			auto A = this->getNode(faceNodeId(0))->point(), B = this->getNode(faceNodeId(1))->point(),
				C = this->getNode(faceNodeId(2))->point(), D = this->getNode(tmpNodeId)->point();
			auto AB = B - A, AC = C - A, AD = D - A;
			auto faceNormal = AB.cross(AC);
			faceNormal.normalize();
			if (!(AD.dot(faceNormal) >= 0 || fabs(AD.dot(faceNormal)) <= 1e-8)) faceNormal = -faceNormal;

			// ��ӵ�Surafece TriMesh
			surfaceTriMesh.addFace(surfaceTriMesh.nFaces() + 1, faceNodeId, faceNormal);
		}
	}
}

const shared_ptr<Tetrahedra> TetMesh::getTetElement(int tetId) const {
	if (this->m_tetElement.find(tetId) == this->m_tetElement.end()) return nullptr;
	return this->m_tetElement.at(tetId);
}

const shared_ptr<TetNode> TetMesh::getNode(int nodeId) const {
	if (this->m_node.find(nodeId) == this->m_node.end()) return nullptr;
	return this->m_node.at(nodeId);
}

size_t TetMesh::nNodes() const {
	return this->m_node.size();
}

size_t TetMesh::nTets() const {
	return this->m_tetElement.size();
}

void TetMesh::addNode(int nodeId, const Eigen::Vector3d& point) {
	this->m_node.insert({ nodeId, shared_ptr<TetNode>(new TetNode(nodeId, point, this)) });
}

void TetMesh::addTetElement(int tetId, const Eigen::Vector4i& tetNodeId) {
	this->m_tetElement.insert({ tetId, shared_ptr<Tetrahedra>(new Tetrahedra(tetId, tetNodeId, this)) });
}

const concurrent_unordered_map<int, shared_ptr<TetNode>>& TetMesh::allNodes() const {
	return this->m_node;
}

concurrent_unordered_map<int, shared_ptr<TetNode>>& TetMesh::allNodes() {
	return this->m_node;
}

const concurrent_unordered_map<int, shared_ptr<Tetrahedra>>& TetMesh::allTetElements() const {
	return this->m_tetElement;
}

concurrent_unordered_map<int, shared_ptr<Tetrahedra>>& TetMesh::allTetElements() {
	return this->m_tetElement;
}



TetNode::TetNode(int nodeId, const Eigen::Vector3d &point, TetMesh *tetMesh) {
	this->m_id = nodeId;
	this->m_point = Eigen::Vector3d(point);
	this->m_tetMesh = tetMesh;
	this->m_onSurface = false;
}

const Eigen::Vector3d& TetNode::point() const {
	return this->m_point;
}

Eigen::Vector3d& TetNode::point() {
	return this->m_point;
}

int TetNode::id() const {
	return this->m_id;
}

bool TetNode::onSurface() const {
	return this->m_onSurface;
}

const concurrent_vector<int>& TetNode::getAdjTetId() const {
	return this->m_adjTetId;
}

void TetNode::setSurfaceState(bool state) {
	this->m_onSurface = state;
}

void TetNode::addAdjTetId(int tetId) {
	this->m_adjTetId.push_back(tetId);
}

Tetrahedra::Tetrahedra(int tetId, const Eigen::Vector4i &tetNodeIdVec, TetMesh *tetMesh) {
	this->m_id = tetId;
	this->m_tetMesh = tetMesh;
	this->m_tetNodeIdVec = Eigen::Vector4i(tetNodeIdVec);
	// ��ӵ����������������id���
	this->m_tetMesh->getNode(tetNodeIdVec.x())->addAdjTetId(tetId);
	this->m_tetMesh->getNode(tetNodeIdVec.y())->addAdjTetId(tetId);
	this->m_tetMesh->getNode(tetNodeIdVec.z())->addAdjTetId(tetId);
	this->m_tetMesh->getNode(tetNodeIdVec.w())->addAdjTetId(tetId);;
}

Eigen::Vector3i Tetrahedra::getTriangleId(int nodeId) {
	Eigen::Vector3i ret;
	int j = 0;
	for (int i = 0; i < 4; i++) {
		if (this->m_tetNodeIdVec(i, 0) != nodeId)
			ret(j++, 0) = this->m_tetNodeIdVec(i, 0);
	}
	return ret;
}

Eigen::Vector4i Tetrahedra::getTetNodeId() const {
	return this->m_tetNodeIdVec;
}

int Tetrahedra::id() const {
	return this->m_id;
}
