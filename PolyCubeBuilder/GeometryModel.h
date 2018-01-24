/*
* Author: tiny656
* Date: 2015-12-01
*/
#pragma once

#include "utility.h"
#include "TriMesh.h"

class Plane; // 平面
class Boundry; // 边
class Point; // 点

class GeometryModel {

public:

	GeometryModel(const TriMesh *surfaceTriMesh);

	void buildGeometryModel(); // 从Surface TriMesh构造几何结构模型

	const shared_ptr<Plane>& getPlane(int planeId) const; // 获取平面
	const shared_ptr<Boundry>& getBoundry(int boundryId) const; // 获取边界
	const shared_ptr<Point>& getPoint(int pointId) const; // 获取点

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

	size_t nPlanes() const; // 返回所有的几何平面的数量
	size_t nBoundrys() const; // 返回所有边的数量
	size_t nPoints() const; // 返回所有顶点的数量
	size_t nEndPoints() const;


private:
	void floodfill(int faceId, const TriMesh *surfaceTriMesh, bool *visFace, shared_ptr<Plane> &plane); // 对trimesh的面按照法向关系进行floodfill算法

private:
	const TriMesh *m_surfaceTriMesh; // 关联的表面三角网格mesh
	concurrent_unordered_map<int, shared_ptr<Plane>> m_plane; // 所有的Polycube平面
	concurrent_unordered_map<int, shared_ptr<Boundry>> m_boundry; // 所有的边
	concurrent_unordered_map<int, shared_ptr<Point>>  m_point; // 所有的点
	concurrent_unordered_set<int> m_endPointId; // 所有的端点id
	concurrent_unordered_set<int> m_boundryPointId; // 所有的边界点id
};


class Plane {
public:
	Plane(int planeId, GeometryModel *geometryModel);

	void addTriMeshFaceId(int triMeshFaceId); // 添加相关的Trimesh的Face的Id
	void addPointId(int pointId); // 添加平面上的点Id
	void addInnerPointId(int innerPointId); // 添加内部点id
	void addEndPointId(int endPointId); // 添加端点id
	void addBoundryPointId(int boundryPointId); // 添加边上点id
	void addBoundryId(int boundryId); // 添加边的id

	const concurrent_vector<int>& allBoundryId() const; // 获取当前平面边的id
	concurrent_vector<int>& allBoundryId(); // 获取当前平面边的id
	const concurrent_vector<int>& allTriMeshFaceId() const; // 平面所有三角面片的id
	concurrent_vector<int>& allTriMeshFaceId(); // 平面所有三角面片的id

	const set<int>& allPointId() const; // 平面所有点的id
	set<int>& allPointId(); // 平面所有点的id
	const concurrent_vector<int>& allInnerPointId() const; // 平面所内部点id
	concurrent_vector<int>& allInnerPointId(); // 平面所内部点id
	const concurrent_vector<int>& allEndPointId() const; // 获取所有端点的id
	concurrent_vector<int>& allEndPointId(); // 获取所有端点的id
	const set<int>& allBoundryPointId() const; // 所有边界点除去端点的id
	set<int>& allBoundryPointId(); // 所有边界点除去端点的id

	size_t nTriMeshFace() const; // 平面上三角面片数量
	size_t nInnerPoint() const; // 平面上内部点的数量
	size_t nEndPoint() const; // 平面上的端点的数量
	size_t nBoundryPoint() const; // 平面上边界点数量，不计端点
	size_t nBoundry() const; // 平面上边界的数量

	int id() const; // 返回平面的id

	const GeometryModel* getGeometryModel() const; // 返回所属的GeometryModel

private:
	GeometryModel *m_geometryModel;
	int m_id; // 平面的id
	
	concurrent_vector<int> m_boundryId; // 平面上边的Id
	concurrent_vector<int> m_triMeshFaceId; // 平面上所有三角面片的Id
	set<int> m_allPointId; // 所有点的集合
	concurrent_vector<int> m_innerPointId; // 平面内部triMesh的点Id
	concurrent_vector<int> m_endPointId; // 平面上端点的Id
	set<int> m_boundryPointId; // 边界上面除去端点的点Id
};


class Boundry {
public:
	Boundry(int boundryId, GeometryModel *geometryModel);

	int id() const; // 边的id
	
	void addPlaneId(int planeId); // 添加边界相关面的id 
	void addEndPointId(pair<int, int> endPointId); // 添加端点Id
	void addBoundryPointId(int boundryPointId); // 添加边界点id，非端点

	const concurrent_vector<int>& allPlaneId() const; // 返回所有的平面id
	concurrent_vector<int>& allPlaneId(); // 返回所有的平面id
	const concurrent_vector<int>& allBoundryPointId() const; // 返回所有的边界点，除去端点
	concurrent_vector<int>& allBoundryPointId(); // 返回所有的边界点，除去端点
	const pair<int, int>& endPointId() const; // 返回端点的id
	pair<int, int>& endPointId(); // 返回端点的id 
	
	pair<Eigen::Vector3d, int> getDirection(int endPointId); // 传入当前边界上的一个点id，返回边的方向
	
	const GeometryModel* getGeometryModel() const; // 返回所属的GeometryModel


private:
	GeometryModel *m_geometryModel;
	int m_id; // 边的id
	concurrent_vector<int> m_planeId; // 所属面Id集合
	concurrent_vector<int> m_boundryPointId; // 边上除去端点的id(指向surface trimesh上的id)
	pair<int, int> m_endPointId; // 边上的两个端点id(指向surface trimesh上的id)
};


class Point {
public:
	Point(int pointId, GeometryModel *geometryModel);

	int id() const; // 返回点的id
		
	void addPlaneId(int planeId); // 添加平面Id
	void addBoundryId(int boundryId); // 添加边界id

	const set<int>& allPlaneId() const; // 返回所有的平面Id
	set<int>& allPlaneId(); // 返回所有的平面id
	const concurrent_vector<int>& allBoundryId() const; // 返回所有的边界id
	concurrent_vector<int>& allBoundryId(); // 返回所有的边界id

	const GeometryModel* getGeometryModel() const; // 返回所属的GeometryModel

private:
	GeometryModel *m_geometryModel;
	int m_id; // 点的id，和surface triMesh上对应
	set<int> m_planeId; // 所属面Id的集合
	concurrent_vector<int> m_boundtryId; // 所属边的Id集合
};
