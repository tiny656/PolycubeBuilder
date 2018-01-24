/*
* Author: tiny656
* Date: 2015-12-03
*/

#include "MappingAlgo.h"
#include "TinyLogger.h"
#include "Timer.h"
#include "MeanValueCoordsAlgo.h"
#include "MeshViewer.h"

void MappingAlgo::run(
	const GeometryModel& oriGeometryModel,
	const GeometryModel& refGeometryModel,
	TriMesh *oriSurfaceTriMesh,
	TriMesh *refSurfaceTriMesh) {	
	Timer timer;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MappingAlgo -> run ---  开始模型的参数化映射");

	// 初始化标记数组为未访问
	concurrent_unordered_map<int, bool> visOriEndPoint, visRefEndPoint;
	for_each(oriGeometryModel.allEndPointId().begin(), oriGeometryModel.allEndPointId().end(), [&](const int &endPointId) { visOriEndPoint[endPointId] = false; });
	for_each(refGeometryModel.allEndPointId().begin(), refGeometryModel.allEndPointId().end(), [&](const int &endPointId) { visRefEndPoint[endPointId] = false; });
	
	Eigen::Vector3d colorSet[24] = {
		Eigen::Vector3d(0, 0, 0),
		Eigen::Vector3d(1, 0, 0),
		Eigen::Vector3d(0, 1, 0),
		Eigen::Vector3d(0, 0, 1),
		Eigen::Vector3d(1, 1, 0),
		Eigen::Vector3d(1, 0, 1),
		Eigen::Vector3d(0, 1, 1),
		Eigen::Vector3d(1, 1, 1),
	};
	// 模型对应点显示
	concurrent_unordered_map<int, Eigen::Vector3d> colorOri;
	concurrent_unordered_map<int, Eigen::Vector3d> colorRef;
	int i = 0;
	// ori - ref 端点对应 有问题 BUG
	concurrent_unordered_map<int, int> endPointMapping;
	while (true) {
		// 考虑边不一定完全连通可达，不断重复去查找对应点
		pair<int, int> seedPair = findSeedOfEndPoint(oriGeometryModel, refGeometryModel, visOriEndPoint, visRefEndPoint); // 寻找到端点的种子点
		if (seedPair.first == -1 || seedPair.second == -1) break;
		mappingEndPoint(seedPair, oriGeometryModel, refGeometryModel, visOriEndPoint, visRefEndPoint, endPointMapping);
	}
	
	for (const auto &it : endPointMapping) {
		colorOri[it.first] = colorSet[i % 8];
		colorRef[it.second] = colorSet[i % 8];
		i++;
	}
	MeshViewer::showTriMeshPoint(*oriSurfaceTriMesh, colorOri, "ORI种子端点");
	MeshViewer::showTriMeshPoint(*refSurfaceTriMesh, colorRef, "REF种子端点");
	

	// 遍历所有的边界，构造插值出新的边界点位置
	concurrent_unordered_map<int, Eigen::Vector3d> oriBoundryPointNewPos;
	for (const auto &oriBoundryIt : oriGeometryModel.allBoundry()) {
		shared_ptr<Boundry> oriBoundry = oriBoundryIt.second;
		pair<int, int> oriEndPointId = oriBoundry->endPointId();
		assert(oriEndPointId.first != oriEndPointId.second);
		Eigen::Vector3d oriEndPointA = oriSurfaceTriMesh->getVertex(oriEndPointId.first)->point(); // 源模型上端点坐标值
		Eigen::Vector3d oriEndPointB = oriSurfaceTriMesh->getVertex(oriEndPointId.second)->point();
		Eigen::Vector3d refEndPointA = refSurfaceTriMesh->getVertex(endPointMapping[oriEndPointId.first])->point(); // 中间模型上端点坐标值
		Eigen::Vector3d refEndPointB = refSurfaceTriMesh->getVertex(endPointMapping[oriEndPointId.second])->point();
		// 遍历当前边上的所有边界点，除去端点，进行弦长参数化求解在中间模型上的新坐标
		for (const auto &oriBoundryPointId : oriBoundry->allBoundryPointId()) {
			Eigen::Vector3d oriBoundryPoint = oriSurfaceTriMesh->getVertex(oriBoundryPointId)->point();
			Eigen::Vector3d pri;
			double value = 0;
			pri(0, 0) = fabs(oriEndPointB.x() - oriEndPointA.x()) > 1e-3 ? (oriBoundryPoint.x() - oriEndPointA.x()) / (oriEndPointB.x() - oriEndPointA.x()) : 0;
			pri(1, 0) = fabs(oriEndPointB.y() - oriEndPointA.y()) > 1e-3 ? (oriBoundryPoint.y() - oriEndPointA.y()) / (oriEndPointB.y() - oriEndPointA.y()) : 0;
			pri(2, 0) = fabs(oriEndPointB.z() - oriEndPointA.z()) > 1e-3 ? (oriBoundryPoint.z() - oriEndPointA.z()) / (oriEndPointB.z() - oriEndPointA.z()) : 0;
			oriBoundryPointNewPos[oriBoundryPointId] = (Eigen::MatrixXd::Ones(3, 1) - pri).cwiseProduct(refEndPointA) + pri.cwiseProduct(refEndPointB);
		}
	} 

	// 进行每个平面的内部点，二维均值坐标插值，并更新到ori SurfaceTriMesh
	for (const auto &planeIt : oriGeometryModel.allPlane()) {
		int planeId = planeIt.first;
		shared_ptr<Plane> plane = planeIt.second;
		MeanValueCoordsAlgo::compute2DClosedPolygon(plane, oriSurfaceTriMesh, refSurfaceTriMesh, endPointMapping, oriBoundryPointNewPos);
	}

	// 更新边界点到ori SurfaceTrie1esh
	for (const auto &oriBoundryPointId : oriGeometryModel.allBoundryPointId()) {
		shared_ptr<Vertex> oriSuraceMeshVertex = oriSurfaceTriMesh->getVertex(oriBoundryPointId);
		oriSuraceMeshVertex->point().x() = oriBoundryPointNewPos[oriBoundryPointId].x();
		oriSuraceMeshVertex->point().y() = oriBoundryPointNewPos[oriBoundryPointId].y();
		oriSuraceMeshVertex->point().z() = oriBoundryPointNewPos[oriBoundryPointId].z();
	}

	// 更新端点到ori SurfaceTrie1esh
	for (const auto &oriEndPointId : oriGeometryModel.allEndPointId()) {
		shared_ptr<Vertex> oriSurfaceEndPointVertex = oriSurfaceTriMesh->getVertex(oriEndPointId);
		shared_ptr<Vertex> refSurfaceEndPointVertex = refSurfaceTriMesh->getVertex(endPointMapping[oriEndPointId]);
		oriSurfaceEndPointVertex->point().x() = refSurfaceEndPointVertex->point().x();
		oriSurfaceEndPointVertex->point().y() = refSurfaceEndPointVertex->point().y();
		oriSurfaceEndPointVertex->point().z() = refSurfaceEndPointVertex->point().z();
	}
	
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MappingAlgo -> run --- 模型映射完毕，共耗时 " + to_string(timer.elapsed_seconds()) + " 秒");
}

pair<int, int> MappingAlgo::findSeedOfEndPoint(
	const GeometryModel& oriGeometryModel, 
	const GeometryModel& refGeometryModel,
	concurrent_unordered_map<int, bool> &visOriEndPoint,
	concurrent_unordered_map<int, bool> &visRefEndPoint) {

	const TriMesh *oriSurfaceTriMesh = oriGeometryModel.getSurfaceTriMesh();
	const TriMesh *refSurfaceTriMesh = refGeometryModel.getSurfaceTriMesh();

	concurrent_unordered_set<int> oriEndPointIdSet = oriGeometryModel.allEndPointId(), refEndPointIdSet = refGeometryModel.allEndPointId(); // 记录参考模型和源模型的端点的Id

	assert(oriEndPointIdSet.size() == refEndPointIdSet.size()); // 端点数量应该相同
	auto internalFindEndPoint = [](const concurrent_unordered_set<int> &set, const TriMesh *surfaceTriMesh, concurrent_unordered_map<int, bool> &visEndPoint)
	{
		int ret = -1;
		Eigen::Vector3d sentinel(-1e10, -1e10, -1e10);

		for (const auto &endPointId : set) {
			if (visEndPoint[endPointId]) continue;
			Eigen::Vector3d point = surfaceTriMesh->getVertex(endPointId)->point();
			if (fabs(point.x() - sentinel.x()) < 1e-1) {
				if (fabs(point.y() - sentinel.y()) < 1e-1) {
					if (point.z() > sentinel.z()) {
						sentinel.z() = point.z();
						ret= endPointId;
					}
				} else if (point.y() > sentinel.y()) {
					sentinel.z() = point.z();
					sentinel.y() = point.y();
					ret = endPointId;
				}
			} else if (point.x() > sentinel.x()) {
				sentinel.x() = point.x();
				sentinel.y() = point.y();
				sentinel.z() = point.z();
				ret = endPointId;
			}
		}
		return ret;
	};
	pair<int, int> ret;
	ret.first = internalFindEndPoint(oriEndPointIdSet, oriSurfaceTriMesh, visOriEndPoint);
	ret.second= internalFindEndPoint(refEndPointIdSet, refSurfaceTriMesh, visRefEndPoint);
	return ret;
}

void MappingAlgo::mappingEndPoint(
	const pair<int, int> seedPair, 
	const GeometryModel &oriGeometryModel, 
	const GeometryModel &refGeometryModel, 
	concurrent_unordered_map<int, bool> &visOriEndPoint,
	concurrent_unordered_map<int, bool> &visRefEndPoint,
	concurrent_unordered_map<int, int> &endPointMappingId) {

	// 标记种子点
	endPointMappingId.insert({ seedPair.first, seedPair.second });
	

	// 开始宽搜进行端点映射
	queue<pair<int, int>> q;
	q.push(seedPair);
	visOriEndPoint[seedPair.first] = visRefEndPoint[seedPair.second] = true;

	while ( !q.empty() ) {
		pair<int, int> curMapEndPointId = q.front(); q.pop();
		int oriEndPointId = curMapEndPointId.first, refEndPointId = curMapEndPointId.second;
		shared_ptr<Point> oriPoint = oriGeometryModel.getPoint(oriEndPointId);
		shared_ptr<Point> refPoint = refGeometryModel.getPoint(refEndPointId);
		for (int oriBoundryId : oriPoint->allBoundryId()) {
			shared_ptr<Boundry> oriBoundry = oriGeometryModel.getBoundry(oriBoundryId);
			pair<Eigen::Vector3d, int> oriDirection = oriBoundry->getDirection(oriEndPointId);
			if (visOriEndPoint[oriDirection.second]) continue; // 当前源模型上的点被访问了
			Eigen::Vector3d oriDir = oriDirection.first;
			for (int refBoundryId : refPoint->allBoundryId()) {
				shared_ptr<Boundry> refBoundry = refGeometryModel.getBoundry(refBoundryId);
				pair<Eigen::Vector3d, int> refDirection = refBoundry->getDirection(refEndPointId);
				if (visRefEndPoint[refDirection.second]) continue;
				// 如果两个方向一致
				Eigen::Vector3d refDir = refDirection.first;
				double diff = oriDir.dot(refDir) / (oriDir.norm() * refDir.norm());
				if (fabs(diff - 1) < 1e-3) {
					visOriEndPoint[oriDirection.second] = visRefEndPoint[refDirection.second] = true;
					endPointMappingId[oriDirection.second] = refDirection.second;
					q.push({ oriDirection.second, refDirection.second });
					break;
				}
			}
		}
	}
}
