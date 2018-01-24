/*
* Author: tiny656
* Date: 2015-12-01
*/
#pragma once

#include "utility.h"
#include "TriMesh.h"

class Plane; // ƽ��
class Boundry; // ��
class Point; // ��

class GeometryModel {

public:

	GeometryModel(const TriMesh *surfaceTriMesh);

	void buildGeometryModel(); // ��Surface TriMesh���켸�νṹģ��

	const shared_ptr<Plane>& getPlane(int planeId) const; // ��ȡƽ��
	const shared_ptr<Boundry>& getBoundry(int boundryId) const; // ��ȡ�߽�
	const shared_ptr<Point>& getPoint(int pointId) const; // ��ȡ��

	const concurrent_unordered_map<int, shared_ptr<Plane>>& allPlane() const;
	concurrent_unordered_map<int, shared_ptr<Plane>>& allPlane();
	const concurrent_unordered_map<int, shared_ptr<Boundry>>& allBoundry() const;
	concurrent_unordered_map<int, shared_ptr<Boundry>>& allBoundry();
	const concurrent_unordered_map<int, shared_ptr<Point>>& allPoint() const;
	concurrent_unordered_map<int, shared_ptr<Point>>& allPoint();
	const concurrent_unordered_set<int>& allEndPointId() const;
	concurrent_unordered_set<int>& allEndPointId();
	const concurrent_unordered_set<int>& allBoundryPointId() const;
	concurrent_unordered_set<int>& allBoundryPointId();

	const TriMesh* getSurfaceTriMesh() const;

	size_t nPlanes() const; // �������еļ���ƽ�������
	size_t nBoundrys() const; // �������бߵ�����
	size_t nPoints() const; // �������ж��������
	size_t nEndPoints() const;


private:
	void floodfill(int faceId, const TriMesh *surfaceTriMesh, bool *visFace, shared_ptr<Plane> &plane); // ��trimesh���水�շ����ϵ����floodfill�㷨

private:
	const TriMesh *m_surfaceTriMesh; // �����ı�����������mesh
	concurrent_unordered_map<int, shared_ptr<Plane>> m_plane; // ���е�Polycubeƽ��
	concurrent_unordered_map<int, shared_ptr<Boundry>> m_boundry; // ���еı�
	concurrent_unordered_map<int, shared_ptr<Point>>  m_point; // ���еĵ�
	concurrent_unordered_set<int> m_endPointId; // ���еĶ˵�id
	concurrent_unordered_set<int> m_boundryPointId; // ���еı߽��id
};


class Plane {
public:
	Plane(int planeId, GeometryModel *geometryModel);

	void addTriMeshFaceId(int triMeshFaceId); // �����ص�Trimesh��Face��Id
	void addPointId(int pointId); // ���ƽ���ϵĵ�Id
	void addInnerPointId(int innerPointId); // ����ڲ���id
	void addEndPointId(int endPointId); // ��Ӷ˵�id
	void addBoundryPointId(int boundryPointId); // ��ӱ��ϵ�id
	void addBoundryId(int boundryId); // ��ӱߵ�id

	const concurrent_vector<int>& allBoundryId() const; // ��ȡ��ǰƽ��ߵ�id
	concurrent_vector<int>& allBoundryId(); // ��ȡ��ǰƽ��ߵ�id
	const concurrent_vector<int>& allTriMeshFaceId() const; // ƽ������������Ƭ��id
	concurrent_vector<int>& allTriMeshFaceId(); // ƽ������������Ƭ��id

	const set<int>& allPointId() const; // ƽ�����е��id
	set<int>& allPointId(); // ƽ�����е��id
	const concurrent_vector<int>& allInnerPointId() const; // ƽ�����ڲ���id
	concurrent_vector<int>& allInnerPointId(); // ƽ�����ڲ���id
	const concurrent_vector<int>& allEndPointId() const; // ��ȡ���ж˵��id
	concurrent_vector<int>& allEndPointId(); // ��ȡ���ж˵��id
	const set<int>& allBoundryPointId() const; // ���б߽���ȥ�˵��id
	set<int>& allBoundryPointId(); // ���б߽���ȥ�˵��id

	size_t nTriMeshFace() const; // ƽ����������Ƭ����
	size_t nInnerPoint() const; // ƽ�����ڲ��������
	size_t nEndPoint() const; // ƽ���ϵĶ˵������
	size_t nBoundryPoint() const; // ƽ���ϱ߽�����������ƶ˵�
	size_t nBoundry() const; // ƽ���ϱ߽������

	int id() const; // ����ƽ���id

	const GeometryModel* getGeometryModel() const; // ����������GeometryModel

private:
	GeometryModel *m_geometryModel;
	int m_id; // ƽ���id
	
	concurrent_vector<int> m_boundryId; // ƽ���ϱߵ�Id
	concurrent_vector<int> m_triMeshFaceId; // ƽ��������������Ƭ��Id
	set<int> m_allPointId; // ���е�ļ���
	concurrent_vector<int> m_innerPointId; // ƽ���ڲ�triMesh�ĵ�Id
	concurrent_vector<int> m_endPointId; // ƽ���϶˵��Id
	set<int> m_boundryPointId; // �߽������ȥ�˵�ĵ�Id
};


class Boundry {
public:
	Boundry(int boundryId, GeometryModel *geometryModel);

	int id() const; // �ߵ�id
	
	void addPlaneId(int planeId); // ��ӱ߽�������id 
	void addEndPointId(pair<int, int> endPointId); // ��Ӷ˵�Id
	void addBoundryPointId(int boundryPointId); // ��ӱ߽��id���Ƕ˵�

	const concurrent_vector<int>& allPlaneId() const; // �������е�ƽ��id
	concurrent_vector<int>& allPlaneId(); // �������е�ƽ��id
	const concurrent_vector<int>& allBoundryPointId() const; // �������еı߽�㣬��ȥ�˵�
	concurrent_vector<int>& allBoundryPointId(); // �������еı߽�㣬��ȥ�˵�
	const pair<int, int>& endPointId() const; // ���ض˵��id
	pair<int, int>& endPointId(); // ���ض˵��id 
	
	pair<Eigen::Vector3d, int> getDirection(int endPointId); // ���뵱ǰ�߽��ϵ�һ����id�����رߵķ���
	
	const GeometryModel* getGeometryModel() const; // ����������GeometryModel


private:
	GeometryModel *m_geometryModel;
	int m_id; // �ߵ�id
	concurrent_vector<int> m_planeId; // ������Id����
	concurrent_vector<int> m_boundryPointId; // ���ϳ�ȥ�˵��id(ָ��surface trimesh�ϵ�id)
	pair<int, int> m_endPointId; // ���ϵ������˵�id(ָ��surface trimesh�ϵ�id)
};


class Point {
public:
	Point(int pointId, GeometryModel *geometryModel);

	int id() const; // ���ص��id
		
	void addPlaneId(int planeId); // ���ƽ��Id
	void addBoundryId(int boundryId); // ��ӱ߽�id

	const set<int>& allPlaneId() const; // �������е�ƽ��Id
	set<int>& allPlaneId(); // �������е�ƽ��id
	const concurrent_vector<int>& allBoundryId() const; // �������еı߽�id
	concurrent_vector<int>& allBoundryId(); // �������еı߽�id

	const GeometryModel* getGeometryModel() const; // ����������GeometryModel

private:
	GeometryModel *m_geometryModel;
	int m_id; // ���id����surface triMesh�϶�Ӧ
	set<int> m_planeId; // ������Id�ļ���
	concurrent_vector<int> m_boundtryId; // �����ߵ�Id����
};
