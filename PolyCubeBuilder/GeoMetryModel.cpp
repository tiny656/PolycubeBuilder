/*
* Author: tiny656
* Date: 2015-12-01
*/
#pragma once

#include "GeometryModel.h"
#include "TinyLogger.h"

void GeometryModel::buildGeometryModel() {
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- 开始构造几何模型");
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- FloodFill进行平面划分");
	// 1. 对三角面片依据法向关系进行floodfill染色, 并在染色的过程中，建立点面关系
	bool *visFace = new bool[this->m_surfaceTriMesh->nFaces()+1];
	fill(visFace, visFace + this->m_surfaceTriMesh->nFaces() + 1, false);
	int planeId = 1; // 平面的id
	for (const auto f : this->m_surfaceTriMesh->allFaces()) {
		// 当前三角面未被染色
		if (!visFace[f.first]) {
			shared_ptr<Plane> plane = shared_ptr<Plane>(new Plane(planeId++, this));
			// 开始floodfill算法
			floodfill(f.first, this->m_surfaceTriMesh, visFace, plane);
			// 添加plane
			this->m_plane.insert({ plane->id(), plane });
		}
	}
	delete[] visFace;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- 平面划分完毕，共" + to_string(this->nPlanes()) + "个平面");

	// 端点-边的索引集合
	concurrent_unordered_map<string, int> endPointToBoundryHash;

	// 2. 遍历所有plane
	int boundryId = 1; // 边的id
	for (const auto &planeIt : this->allPlane()) {

		int planeId = planeIt.first;
		shared_ptr<Plane> plane = planeIt.second;

		// 点进行分类 a.端点 b. 内部点 c. 边上点分类
		for (const auto &pointId : plane->allPointId()) {
			shared_ptr<Point> point = this->getPoint(pointId);
			int hasFacesNum = point->allPlaneId().size();
			if (hasFacesNum == 1) // 内部点
				plane->addInnerPointId(pointId);
			else if (hasFacesNum == 2) { // 边上点
				plane->addBoundryPointId(pointId);
				this->m_boundryPointId.insert(pointId);
			} else if (hasFacesNum > 2) {// 端点
				plane->addEndPointId(pointId);
				this->m_endPointId.insert(pointId);
			}
		}

		LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "GeometryModel -> buildGeometryModel --- 面" + to_string(planeId) + ", 点分类完毕，"
			+ to_string(plane->nTriMeshFace()) + "个TriMesh面，"
			+ to_string(plane->nInnerPoint()) + "个内部点，"
			+ to_string(plane->nEndPoint()) + "个端点，"
			+ to_string(plane->nBoundryPoint()) + "个边界上点");
	}

	for (const auto &planeIt : this->allPlane()) {
		int planeId = planeIt.first;
		shared_ptr<Plane> plane = planeIt.second;
		// 两重循环遍历端点，构造边(注意判重复) 
		concurrent_vector<int> endPointIdVec = plane->allEndPointId();
		for (int i = 0; i < endPointIdVec.size(); i++) {
			for (int j = i + 1; j < endPointIdVec.size(); j++) {
				int endPointId_1 = endPointIdVec[i], endPointId_2 = endPointIdVec[j];

				// 判断endpointid1 + endpointid2的Boundry是否出现过, 如果出现过则跳过，开始构造下一个
				string endPointStr = to_string(min(endPointId_1, endPointId_2)) + "," + to_string(max(endPointId_1, endPointId_2));
				if (endPointToBoundryHash.find(endPointStr) != endPointToBoundryHash.end()) continue; 

				shared_ptr<Point> point_1 = this->getPoint(endPointId_1), point_2 = this->getPoint(endPointId_2);
				set<int> planes_1 = point_1->allPlaneId(), planes_2 = point_2->allPlaneId();

				// 如果两个点相交，并且交集大小恰好等于2，那么这两个点就是边的两个端点
				set<int> planeResult;
				set_intersection(planes_1.begin(), planes_1.end(), planes_2.begin(), planes_2.end(), inserter(planeResult, planeResult.begin()));
				if (planeResult.size() == 2) {
					vector<int> intersectPlanes;
					shared_ptr<Boundry> boundry = shared_ptr<Boundry>(new Boundry(boundryId++, this));
					endPointToBoundryHash.insert({ endPointStr, boundry->id() }); // 对加入的边界边进行hash，防止重复添加
					for (int pId : planeResult) {
						intersectPlanes.push_back(pId);
						boundry->addPlaneId(pId); // 添加边界上平面id的索引
					}
					boundry->addEndPointId({ endPointId_1, endPointId_2 }); // 添加端点id
					point_1->addBoundryId(boundry->id()); // 添加第一个端点的边界id
					point_2->addBoundryId(boundry->id()); // 添加第二个端点的边界id
		

					// 两个面的BoundryPointId求交得到，Boundry上的点, 非端点
					shared_ptr<Plane> plane_1 = this->getPlane(intersectPlanes[0]), plane_2 = this->getPlane(intersectPlanes[1]);
					plane_1->addBoundryId(boundry->id()); // 平面添加所属边界
					plane_2->addBoundryId(boundry->id()); // 平面添加所属边界
					set<int> boundryPoint_1 = plane_1->allBoundryPointId(), boundryPoint_2 = plane_2->allBoundryPointId();
					set<int> boundryPointResult;
					set_intersection(boundryPoint_1.begin(), boundryPoint_1.end(), boundryPoint_2.begin(), boundryPoint_2.end(), inserter(boundryPointResult, boundryPointResult.begin()));
					for (const auto &boundryPointId : boundryPointResult)
						boundry->addBoundryPointId(boundryPointId);

					// 将当前边界记录到GeometryModel
					this->m_boundry.insert({boundry->id(), boundry});
				}
			}
		}
		LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "GeometryModel -> buildGeometryModel --- 面" + to_string(planeId) + ","
			+ "共构造出" + to_string(plane->nBoundry()) + "条边界");
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- 模型共包含" + to_string(this->nBoundrys()) + "条边界");
}

void GeometryModel::floodfill(int faceId, const TriMesh *surfaceTriMesh, bool *visFace, shared_ptr<Plane> &plane) {
	queue<int> q;
	q.push(faceId);
	visFace[faceId] = true;
	while ( !q.empty() ) {
		int curFaceId = q.front();  q.pop();
		// 处理相关信息
		plane->addTriMeshFaceId(curFaceId); // 添加triMesh的id
		shared_ptr<Face> triMeshFace = surfaceTriMesh->getFace(curFaceId);
		Eigen::Vector3i vertexIdOnFace = triMeshFace->getVertexIndex();
		for (int i = 0; i < 3; i++) {
			int vertexId = vertexIdOnFace(i, 0);
			if (this->m_point.find(vertexId) == this->m_point.end()) // Point不存在，建立新的Point，并赋值trimesh中点的id
				this->m_point.insert({ vertexId, shared_ptr<Point>(new Point(vertexId, this)) });
			this->m_point.at(vertexId)->addPlaneId(plane->id()); // point添加他所属于的面
			plane->addPointId(vertexIdOnFace(i, 0)); // Plane 添加它所涵盖的点
		}

		// 遍历相邻的面片，判断如果法向是相同的就加入队列
		set<int> adjTriMeshFace = triMeshFace->getAdjacentFace();
		for (int adjFaceId : adjTriMeshFace) {
			if (!visFace[adjFaceId]) {
				shared_ptr<Face> adjFace = surfaceTriMesh->getFace(adjFaceId);
				// 计算相邻面片法向的差值
				double diff = triMeshFace->normal().dot(adjFace->normal()) / (triMeshFace->normal().norm() * adjFace->normal().norm());
				if (fabs(diff - 1) < 0.1) {
					q.push(adjFaceId);
					visFace[adjFaceId] = true; // 标记当前trimesh面已经被访问
				}
			}
		}
	}
}

GeometryModel::GeometryModel(const TriMesh *surfaceTriMesh) {
	this->m_surfaceTriMesh = surfaceTriMesh;
}

const shared_ptr<Plane>& GeometryModel::getPlane(int planeId) const {
	return this->m_plane.at(planeId);
}

const shared_ptr<Boundry>& GeometryModel::getBoundry(int boundryId) const {
	return this->m_boundry.at(boundryId);
}

const shared_ptr<Point>& GeometryModel::getPoint(int pointId) const {
	return this->m_point.at(pointId);
}

const concurrent_unordered_map<int, shared_ptr<Plane>>& GeometryModel::allPlane() const {
	return this->m_plane;
}

concurrent_unordered_map<int, shared_ptr<Plane>>& GeometryModel::allPlane() {
	return this->m_plane;
}

const concurrent_unordered_map<int, shared_ptr<Boundry>>& GeometryModel::allBoundry() const {
	return this->m_boundry;
}

concurrent_unordered_map<int, shared_ptr<Boundry>>& GeometryModel::allBoundry() {
	return this->m_boundry;
}

const concurrent_unordered_map<int, shared_ptr<Point>>& GeometryModel::allPoint() const {
	return this->m_point;
}

concurrent_unordered_map<int, shared_ptr<Point>>& GeometryModel::allPoint() {
	return this->m_point;
}

const concurrent_unordered_set<int>& GeometryModel::allEndPointId() const {
	return this->m_endPointId;
}

concurrent_unordered_set<int>& GeometryModel::allEndPointId() {
	return this->m_endPointId;
}

const concurrent_unordered_set<int>& GeometryModel::allBoundryPointId() const {
	return this->m_boundryPointId;
}

concurrent_unordered_set<int>& GeometryModel::allBoundryPointId() {
	return this->m_boundryPointId;
}

const TriMesh* GeometryModel::getSurfaceTriMesh() const {
	return this->m_surfaceTriMesh;
}

size_t GeometryModel::nPlanes() const {
	return this->m_plane.size();
}

size_t GeometryModel::nBoundrys() const {
	return this->m_boundry.size();
}

size_t GeometryModel::nPoints() const {
	return this->m_point.size();
}

size_t GeometryModel::nEndPoints() const {
	return this->m_endPointId.size();
}

Plane::Plane(int planeId, GeometryModel *geometryModel) {
	this->m_id = planeId;	
	this->m_geometryModel = geometryModel;
}

void Plane::addTriMeshFaceId(int triMeshFaceId) {
	this->m_triMeshFaceId.push_back(triMeshFaceId);
}

void Plane::addPointId(int pointId) {
	this->m_allPointId.insert(pointId);
}

int Plane::id() const {
	return this->m_id;
}

const GeometryModel* Plane::getGeometryModel() const {
	return this->m_geometryModel;
}

void Plane::addInnerPointId(int innerPointId) {
	this->m_innerPointId.push_back(innerPointId);
}

void Plane::addEndPointId(int endPointId) {
	this->m_endPointId.push_back(endPointId);
}

void Plane::addBoundryPointId(int boundryPointId) {
	this->m_boundryPointId.insert(boundryPointId);
}

void Plane::addBoundryId(int boundryId) {
	this->m_boundryId.push_back(boundryId);
}

const concurrent_vector<int>& Plane::allBoundryId() const {
	return this->m_boundryId;
}

const concurrent_vector<int>& Plane::allTriMeshFaceId() const {
	return this->m_triMeshFaceId;
}

concurrent_vector<int>& Plane::allTriMeshFaceId() {
	return this->m_triMeshFaceId;
}

const set<int>& Plane::allPointId() const {
	return this->m_allPointId;
}

set<int>& Plane::allPointId() {
	return this->m_allPointId;
}

const concurrent_vector<int>& Plane::allInnerPointId() const {
	return this->m_innerPointId;
}

concurrent_vector<int>& Plane::allInnerPointId() {
	return this->m_innerPointId;
}

concurrent_vector<int>& Plane::allBoundryId() {
	return this->m_boundryId;
}

const concurrent_vector<int>& Plane::allEndPointId() const {
	return this->m_endPointId;
}

concurrent_vector<int>& Plane::allEndPointId() {
	return this->m_endPointId;
}

const set<int>& Plane::allBoundryPointId() const {
	return this->m_boundryPointId;
}

set<int>& Plane::allBoundryPointId() {
	return this->m_boundryPointId;
}

size_t Plane::nTriMeshFace() const {
	return this->m_triMeshFaceId.size();
}

size_t Plane::nInnerPoint() const {
	return this->m_innerPointId.size();
}

size_t Plane::nEndPoint() const {
	return this->m_endPointId.size();
}

size_t Plane::nBoundryPoint() const {
	return this->m_boundryPointId.size();
}

size_t Plane::nBoundry() const {
	return this->m_boundryId.size();
}

Boundry::Boundry(int boundryId, GeometryModel* geometryModel) {
	this->m_id = boundryId;
	this->m_geometryModel = geometryModel;
}

int Boundry::id() const {
	return this->m_id;
}

void Boundry::addPlaneId(int planeId) {
	this->m_planeId.push_back(planeId);
}

void Boundry::addEndPointId(pair<int, int> endPointId) {
	this->m_endPointId = endPointId;
}

void Boundry::addBoundryPointId(int boundryPointId) {
	this->m_boundryPointId.push_back(boundryPointId);
}

const concurrent_vector<int>& Boundry::allPlaneId() const {
	return this->m_planeId;
}

concurrent_vector<int>& Boundry::allPlaneId() {
	return this->m_planeId;
}

const concurrent_vector<int>& Boundry::allBoundryPointId() const {
	return this->m_boundryPointId;
}

concurrent_vector<int>& Boundry::allBoundryPointId() {
	return this->m_boundryPointId;
}

const pair<int, int>& Boundry::endPointId() const {
	return this->m_endPointId;
}

pair<int, int>& Boundry::endPointId() {
	return this->m_endPointId;
}

pair<Eigen::Vector3d, int> Boundry::getDirection(int endPointId) {
	assert(endPointId == this->endPointId().first || endPointId == this->endPointId().second);
	
	int otherPointId = (endPointId == this->endPointId().first) ? this->endPointId().second : this->endPointId().first;
	shared_ptr<Vertex> vertex = this->m_geometryModel->getSurfaceTriMesh()->getVertex(endPointId);
	shared_ptr<Vertex> otherVertex = this->m_geometryModel->getSurfaceTriMesh()->getVertex(otherPointId);

	return { otherVertex->point() - vertex->point(), otherPointId};
}

const GeometryModel* Boundry::getGeometryModel() const {
	return this->m_geometryModel;
}

Point::Point(int pointId, GeometryModel *geometryModel) {
	this->m_id = pointId;
	this->m_geometryModel = geometryModel;
}

int Point::id() const {
	return this->m_id;
}

void Point::addPlaneId(int planeId) {
	this->m_planeId.insert(planeId);
}

void Point::addBoundryId(int boundryId) {
	this->m_boundtryId.push_back(boundryId);
}

const set<int>& Point::allPlaneId() const {
	return this->m_planeId;
}

set<int>& Point::allPlaneId() {
	return this->m_planeId;
}

const concurrent_vector<int>& Point::allBoundryId() const {
	return this->m_boundtryId; 
}

concurrent_vector<int>& Point::allBoundryId() {
	return this->m_boundtryId;
}

const GeometryModel* Point::getGeometryModel() const {
	return this->m_geometryModel;
}
