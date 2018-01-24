/*
* Author: tiny656
* Date: 2015-10-12
*/

#include "TriMesh.h"
#include "TinyLogger.h"

/* TriMesh Start*/
void TriMesh::addVertex(int vertexId, const Eigen::Vector3d &point) {
	if (this->m_vertex.find(vertexId) != m_vertex.end())
		LOGGER_WRITE_CONSOLE(TinyLogger::WARNING, "TriMesh -> addVertex --- 原有顶点存在，将被覆盖");
	this->m_vertex.insert({ vertexId, shared_ptr<Vertex>(new Vertex(vertexId, point, this)) });
}

void TriMesh::addFace(int faceId, const Eigen::Vector3i &faceIndex, const Eigen::Vector3d& faceNormal) {
	if (this->m_vertex.find(faceIndex[0]) == this->m_vertex.end()
		|| this->m_vertex.find(faceIndex[1]) == this->m_vertex.end()
		|| this->m_vertex.find(faceIndex[2]) == this->m_vertex.end()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "TriMesh -> addFace --- 索引顶点不存在，添加失败");
		return;
	} else this->m_face.insert({ faceId, shared_ptr<Face>(new Face(faceId, faceIndex, faceNormal, this)) });
}

size_t TriMesh::nFaces() const {
	return this->m_face.size();
}

size_t TriMesh::nVertices() const {
	return this->m_vertex.size();
}

const shared_ptr<Vertex> TriMesh::getVertex(int index) const {
	if (this->m_vertex.find(index) == this->m_vertex.end()) return NULL;
	else return this->m_vertex.at(index);
}

const shared_ptr<Face> TriMesh::getFace(int index) const {
	if (this->m_face.find(index) == this->m_face.end()) return NULL;
	else return this->m_face.at(index);
}

const concurrent_unordered_map<int, shared_ptr<Vertex>>& TriMesh::allVertexs() const {
	return this->m_vertex;
}

concurrent_unordered_map<int, shared_ptr<Vertex>>& TriMesh::allVertexs() {
	return this->m_vertex;
}

const concurrent_unordered_map<int, shared_ptr<Face>>& TriMesh::allFaces() const {
	return this->m_face;
}

concurrent_unordered_map<int, shared_ptr<Face>>& TriMesh::allFaces() {
	return this->m_face;
}

/* Face */
Face::Face(int faceId, const Eigen::Vector3i &faceVertexId, const Eigen::Vector3d& faceNormal, TriMesh *mesh) {
	this->m_faceId = faceId;
	this->m_mesh = mesh;
	this->m_faceVertexId = faceVertexId;
	this->m_faceNormal = faceNormal;
	
	auto vertex0 = this->m_mesh->getVertex(faceVertexId[0]);
	auto vertex1 = this->m_mesh->getVertex(faceVertexId[1]);
	auto vertex2 = this->m_mesh->getVertex(faceVertexId[2]);

	//  每个点添加邻近面
	vertex0->addAdjacentFace(faceId);
	vertex1->addAdjacentFace(faceId);
	vertex2->addAdjacentFace(faceId);

	// 每个面添加邻近面
	auto adjancentFaceId = internalFindCommonFace(faceVertexId);
	for (auto &adjFaceId : adjancentFaceId) {
		if (adjFaceId != faceId) {
			this->addAdjacentFace(adjFaceId);
			this->m_mesh->getFace(adjFaceId)->addAdjacentFace(faceId);
		}
	}
}

int Face::id() const {
	return this->m_faceId;
}

const Eigen::Vector3i& Face::getVertexIndex() const {
	return this->m_faceVertexId;
}

double Face::area() const {
	Eigen::Vector3i faceVertexId = this->getVertexIndex();
	Eigen::Vector3d a = this->m_mesh->getVertex(faceVertexId.x())->point();
	Eigen::Vector3d b = this->m_mesh->getVertex(faceVertexId.y())->point();
	Eigen::Vector3d c = this->m_mesh->getVertex(faceVertexId.z())->point();
	Eigen::Vector3d AB(b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
	Eigen::Vector3d AC(c.x() - a.x(), c.y() - a.y(), c.z() - a.z());
	return 0.5 * (AB.cross(AC)).norm();
}

const TriMesh* Face::getTriMesh() const {
	return this->m_mesh;
}

const Eigen::Vector3d& Face::normal() const {
	return this->m_faceNormal;
}

Eigen::Vector3d& Face::normal() {
	return this->m_faceNormal;
}

set<int> Face::internalFindCommonFace(const Eigen::Vector3i &faceVertexId) const {
	set<int> intersectResult;
	for (int i = 0; i < 3; i++) {
		auto vertexFaceSet = this->m_mesh->getVertex(faceVertexId(i % 3, 0))->getFaceId();
		auto otherVertexFaceSet = this->m_mesh->getVertex(faceVertexId((i + 1) % 3, 0))->getFaceId();
		set_intersection(vertexFaceSet.begin(), vertexFaceSet.end(),
			otherVertexFaceSet.begin(), otherVertexFaceSet.end(),
			inserter(intersectResult, intersectResult.begin()));
	}
	return intersectResult;
}

void Face::addAdjacentFace(int faceId) {
	this->m_adjacentFaceId.insert(faceId);
}

const set<int>& Face::getAdjacentFace() const {
	return this->m_adjacentFaceId;
}


/* Vertex */
Vertex::Vertex(int vertexId, const Eigen::Vector3d &point, TriMesh *mesh) {
	this->m_mesh = mesh;
	this->m_id = vertexId;
	this->m_point = Eigen::Vector3d(point);
}

int Vertex::id() const {
	return this->m_id;
}

void Vertex::addAdjacentFace(int faceId) {
	this->m_adjacentFaceId.insert(faceId);
}


const Eigen::Vector3d& Vertex::point() const {
	return this->m_point;
}

Eigen::Vector3d& Vertex::point() {
	return this->m_point;
}

const TriMesh* Vertex::getTriMesh() const {
	return this->m_mesh;
}

const concurrent_vector<int>& Vertex::getAdjancentVertexId() {
	if (!this->m_adjacentVertexId.empty()) return this->m_adjacentVertexId;

	auto mod3 = [](int v){return v % 3; };

	unordered_map<int, vector<int>> nextVertexId;

	// 遍历所有的相邻面, 构建邻接关系
	for (const auto &faceId : this->m_adjacentFaceId) {
		shared_ptr<Face> face = this->m_mesh->getFace(faceId);
		Eigen::Vector3i faceVertexId = face->getVertexIndex();
		for (int i = 0; i < 3; i++) {
			if (faceVertexId(i, 0) == this->id()) {
				nextVertexId[faceVertexId(mod3(i + 1), 0)].push_back(faceVertexId(mod3(i + 2), 0));
				nextVertexId[faceVertexId(mod3(i + 2), 0)].push_back(faceVertexId(mod3(i + 1), 0));
				break;
			}
		}
	}

	int seedVertexId = nextVertexId.begin()->first;
	bool findNextVertex = true;
	this->m_adjacentVertexId.push_back(seedVertexId);
	while (findNextVertex) {
		findNextVertex = false;
		vector<int> nextVertexIdVec = nextVertexId[seedVertexId];
		assert(nextVertexIdVec.size() == 2 && nextVertexIdVec[0] != nextVertexIdVec[1]); // 三角面片除去环点只有两个其他点
		for (int i = 0; i < nextVertexIdVec.size(); i++) {
			int vertexId = nextVertexIdVec[i];
			if (nextVertexId.find(vertexId) != nextVertexId.end()) {
				this->m_adjacentVertexId.push_back(vertexId);
				nextVertexId.erase(seedVertexId);
				seedVertexId = vertexId;
				findNextVertex = true;
				break;
			}
		} 
	}
	return this->m_adjacentVertexId;
}

const set<int>& Vertex::getFaceId() const {
	return this->m_adjacentFaceId;
}