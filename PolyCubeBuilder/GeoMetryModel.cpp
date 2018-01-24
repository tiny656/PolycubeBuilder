/*
* Author: tiny656
* Date: 2015-12-01
*/
#pragma once

#include "GeometryModel.h"
#include "TinyLogger.h"

void GeometryModel::buildGeometryModel() {
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- ��ʼ���켸��ģ��");
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- FloodFill����ƽ�滮��");
	// 1. ��������Ƭ���ݷ����ϵ����floodfillȾɫ, ����Ⱦɫ�Ĺ����У����������ϵ
	bool *visFace = new bool[this->m_surfaceTriMesh->nFaces()+1];
	fill(visFace, visFace + this->m_surfaceTriMesh->nFaces() + 1, false);
	int planeId = 1; // ƽ���id
	for (const auto f : this->m_surfaceTriMesh->allFaces()) {
		// ��ǰ������δ��Ⱦɫ
		if (!visFace[f.first]) {
			shared_ptr<Plane> plane = shared_ptr<Plane>(new Plane(planeId++, this));
			// ��ʼfloodfill�㷨
			floodfill(f.first, this->m_surfaceTriMesh, visFace, plane);
			// ���plane
			this->m_plane.insert({ plane->id(), plane });
		}
	}
	delete[] visFace;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- ƽ�滮����ϣ���" + to_string(this->nPlanes()) + "��ƽ��");

	// �˵�-�ߵ���������
	concurrent_unordered_map<string, int> endPointToBoundryHash;

	// 2. ��������plane
	int boundryId = 1; // �ߵ�id
	for (const auto &planeIt : this->allPlane()) {

		int planeId = planeIt.first;
		shared_ptr<Plane> plane = planeIt.second;

		// ����з��� a.�˵� b. �ڲ��� c. ���ϵ����
		for (const auto &pointId : plane->allPointId()) {
			shared_ptr<Point> point = this->getPoint(pointId);
			int hasFacesNum = point->allPlaneId().size();
			if (hasFacesNum == 1) // �ڲ���
				plane->addInnerPointId(pointId);
			else if (hasFacesNum == 2) { // ���ϵ�
				plane->addBoundryPointId(pointId);
				this->m_boundryPointId.insert(pointId);
			} else if (hasFacesNum > 2) {// �˵�
				plane->addEndPointId(pointId);
				this->m_endPointId.insert(pointId);
			}
		}

		LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "GeometryModel -> buildGeometryModel --- ��" + to_string(planeId) + ", �������ϣ�"
			+ to_string(plane->nTriMeshFace()) + "��TriMesh�棬"
			+ to_string(plane->nInnerPoint()) + "���ڲ��㣬"
			+ to_string(plane->nEndPoint()) + "���˵㣬"
			+ to_string(plane->nBoundryPoint()) + "���߽��ϵ�");
	}

	for (const auto &planeIt : this->allPlane()) {
		int planeId = planeIt.first;
		shared_ptr<Plane> plane = planeIt.second;
		// ����ѭ�������˵㣬�����(ע�����ظ�) 
		concurrent_vector<int> endPointIdVec = plane->allEndPointId();
		for (int i = 0; i < endPointIdVec.size(); i++) {
			for (int j = i + 1; j < endPointIdVec.size(); j++) {
				int endPointId_1 = endPointIdVec[i], endPointId_2 = endPointIdVec[j];

				// �ж�endpointid1 + endpointid2��Boundry�Ƿ���ֹ�, ������ֹ�����������ʼ������һ��
				string endPointStr = to_string(min(endPointId_1, endPointId_2)) + "," + to_string(max(endPointId_1, endPointId_2));
				if (endPointToBoundryHash.find(endPointStr) != endPointToBoundryHash.end()) continue; 

				shared_ptr<Point> point_1 = this->getPoint(endPointId_1), point_2 = this->getPoint(endPointId_2);
				set<int> planes_1 = point_1->allPlaneId(), planes_2 = point_2->allPlaneId();

				// ����������ཻ�����ҽ�����Сǡ�õ���2����ô����������Ǳߵ������˵�
				set<int> planeResult;
				set_intersection(planes_1.begin(), planes_1.end(), planes_2.begin(), planes_2.end(), inserter(planeResult, planeResult.begin()));
				if (planeResult.size() == 2) {
					vector<int> intersectPlanes;
					shared_ptr<Boundry> boundry = shared_ptr<Boundry>(new Boundry(boundryId++, this));
					endPointToBoundryHash.insert({ endPointStr, boundry->id() }); // �Լ���ı߽�߽���hash����ֹ�ظ����
					for (int pId : planeResult) {
						intersectPlanes.push_back(pId);
						boundry->addPlaneId(pId); // ��ӱ߽���ƽ��id������
					}
					boundry->addEndPointId({ endPointId_1, endPointId_2 }); // ��Ӷ˵�id
					point_1->addBoundryId(boundry->id()); // ��ӵ�һ���˵�ı߽�id
					point_2->addBoundryId(boundry->id()); // ��ӵڶ����˵�ı߽�id
		

					// �������BoundryPointId�󽻵õ���Boundry�ϵĵ�, �Ƕ˵�
					shared_ptr<Plane> plane_1 = this->getPlane(intersectPlanes[0]), plane_2 = this->getPlane(intersectPlanes[1]);
					plane_1->addBoundryId(boundry->id()); // ƽ����������߽�
					plane_2->addBoundryId(boundry->id()); // ƽ����������߽�
					set<int> boundryPoint_1 = plane_1->allBoundryPointId(), boundryPoint_2 = plane_2->allBoundryPointId();
					set<int> boundryPointResult;
					set_intersection(boundryPoint_1.begin(), boundryPoint_1.end(), boundryPoint_2.begin(), boundryPoint_2.end(), inserter(boundryPointResult, boundryPointResult.begin()));
					for (const auto &boundryPointId : boundryPointResult)
						boundry->addBoundryPointId(boundryPointId);

					// ����ǰ�߽��¼��GeometryModel
					this->m_boundry.insert({boundry->id(), boundry});
				}
			}
		}
		LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "GeometryModel -> buildGeometryModel --- ��" + to_string(planeId) + ","
			+ "�������" + to_string(plane->nBoundry()) + "���߽�");
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "GeometryModel -> buildGeometryModel --- ģ�͹�����" + to_string(this->nBoundrys()) + "���߽�");
}

void GeometryModel::floodfill(int faceId, const TriMesh *surfaceTriMesh, bool *visFace, shared_ptr<Plane> &plane) {
	queue<int> q;
	q.push(faceId);
	visFace[faceId] = true;
	while ( !q.empty() ) {
		int curFaceId = q.front();  q.pop();
		// ���������Ϣ
		plane->addTriMeshFaceId(curFaceId); // ���triMesh��id
		shared_ptr<Face> triMeshFace = surfaceTriMesh->getFace(curFaceId);
		Eigen::Vector3i vertexIdOnFace = triMeshFace->getVertexIndex();
		for (int i = 0; i < 3; i++) {
			int vertexId = vertexIdOnFace(i, 0);
			if (this->m_point.find(vertexId) == this->m_point.end()) // Point�����ڣ������µ�Point������ֵtrimesh�е��id
				this->m_point.insert({ vertexId, shared_ptr<Point>(new Point(vertexId, this)) });
			this->m_point.at(vertexId)->addPlaneId(plane->id()); // point����������ڵ���
			plane->addPointId(vertexIdOnFace(i, 0)); // Plane ����������ǵĵ�
		}

		// �������ڵ���Ƭ���ж������������ͬ�ľͼ������
		set<int> adjTriMeshFace = triMeshFace->getAdjacentFace();
		for (int adjFaceId : adjTriMeshFace) {
			if (!visFace[adjFaceId]) {
				shared_ptr<Face> adjFace = surfaceTriMesh->getFace(adjFaceId);
				// ����������Ƭ����Ĳ�ֵ
				double diff = triMeshFace->normal().dot(adjFace->normal()) / (triMeshFace->normal().norm() * adjFace->normal().norm());
				if (fabs(diff - 1) < 0.1) {
					q.push(adjFaceId);
					visFace[adjFaceId] = true; // ��ǵ�ǰtrimesh���Ѿ�������
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
