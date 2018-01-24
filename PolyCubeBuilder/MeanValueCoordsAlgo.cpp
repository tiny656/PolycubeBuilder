/*
* Author: tiny656
* Date: 2015-11-25
*/

#include "MeanValueCoordsAlgo.h"
#include "TriMesh.h"
#include "TetMesh.h"
#include "MeshViewer.h"
#include "Timer.h"
#include "TinyLogger.h"

void MeanValueCoordsAlgo::compute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh) {
	Timer timer; // 计时器
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute3DClosedTriMesh --- 开始进行均值坐标内部点插值计算");
	internalCompute3DClosedTriMesh(surfaceTriMesh, tetMesh); // 计算
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute3DClosedTriMesh --- 插值计算完毕, 共耗时" + to_string(timer.elapsed_seconds()) + "秒");
}

void MeanValueCoordsAlgo::compute2DClosedPolygon(
	const shared_ptr<Plane> plane,
	TriMesh *oriSurfaceTriMesh,
	TriMesh *refSurfaceTriMesh,
	const concurrent_unordered_map<int, int> &endPointMapping,
	const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos) {
	Timer timer; // 计时器
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute2DClosedPolygon --- 开始进行均值坐标内部点插值计算");
	internalCompute2DClosedPolygon(plane, oriSurfaceTriMesh, refSurfaceTriMesh, endPointMapping, oriBoundryPointNewPos); // 计算
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute2DClosedPolygon --- 插值计算完毕, 共耗时" + to_string(timer.elapsed_seconds()) + "秒");
}

#pragma region 计算三维上Polygon内部点坐标
void MeanValueCoordsAlgo::internalCompute2DClosedPolygon(
	shared_ptr<Plane> plane, 
	TriMesh *oriSurfaceTriMesh,
	TriMesh *refSurfaceTriMesh,
	const concurrent_unordered_map<int, int> &endPointMapping,
	const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos) {
	// 遍历所有的节点确定表面节点顺序
	concurrent_unordered_map<int, int> pointPosInMat; // 记录节点在矩阵中的位置，节点id - 矩阵中所在的行
	int pointIdx = 0;
	for (const auto &pointId : plane->allInnerPointId()) pointPosInMat[pointId] = pointIdx++;
	for (const auto &pointId : plane->allEndPointId()) pointPosInMat[pointId] = pointIdx++;
	for (const auto &pointId : plane->allBoundryPointId()) pointPosInMat[pointId] = pointIdx++;

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 表面节点和表面内部节点顺序索引完成");

	int totPointNum = plane->nInnerPoint() + plane->nEndPoint() + plane->nBoundryPoint(); // 所有节点个数
	int innerPointNum = plane->nInnerPoint(), outerNodeNum = totPointNum - innerPointNum; // 内部节点个数, 表面节点个数
	if (innerPointNum == 0) return;

	// 权重稀疏矩阵
	Eigen::SparseMatrix<double> W(innerPointNum, totPointNum); W.setZero();
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 开始构造均值坐标权值矩阵");
	
	// 遍历每个节点，计算所有节点的均值坐标权重
	const GeometryModel* oriGeometryModel = plane->getGeometryModel();

	auto innerPointIdVec = plane->allInnerPointId();
	//#pragma omp parallel for	 
	for (int innerPointIdx = 0; innerPointIdx < innerPointIdVec.size(); innerPointIdx++) {
		int innerPointId = innerPointIdVec[innerPointIdx];
		shared_ptr<Vertex> innerVertex = oriSurfaceTriMesh->getVertex(innerPointId);
		int pos = pointPosInMat[innerVertex->id()];
		// 计算当前点的环绕三角网格的均值坐标权重值，更新稀疏矩阵A和常量矩阵B
		double totalW = 0.0;
		concurrent_unordered_map<int, double> weightInc;
		weightInc.clear();
		internal2DPointMeanValueCompute(innerVertex, pointPosInMat, totalW, weightInc);

		// 更新权值矩阵W
		for (auto &v : weightInc)
			W.insert(pos, v.first) = v.second / totalW;
	}
	
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 均值坐标权值矩阵构造完毕");

	// 构造X的下半部分，平面边界的节点
	Eigen::MatrixXd T(outerNodeNum, 3);
	for (const auto &pointId : plane->allEndPointId()) {
		int pos = pointPosInMat[pointId] - innerPointNum;
		shared_ptr<Vertex> vertex = refSurfaceTriMesh->getVertex(endPointMapping.at(pointId));
		for (int i = 0; i < 3; i++)
			T(pos, i) = vertex->point()(i, 0);
	}
	for (const auto &pointId : plane->allBoundryPointId()) {
		int pos = pointPosInMat[pointId] - innerPointNum;
		// Eigen::Vector3d point = oriSurfaceTriMesh.getVertex(pointId)->point();
		Eigen::Vector3d point = oriBoundryPointNewPos.at(pointId);
		for (int i = 0; i < 3; i++)
			T(pos, i) = point(i, 0);
	}
	
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 构造线性方程AX=B的相关已知参数");
	// 求解AX = B 线性方程组
	Eigen::SparseMatrix<double> A(W.block(0, 0, innerPointNum, innerPointNum)); // 矩阵A

	Eigen::SparseMatrix<double> I(innerPointNum, innerPointNum); I.setIdentity();
	A = A - I;
	Eigen::MatrixXd tmp(W.block(0, innerPointNum, innerPointNum, outerNodeNum));
	Eigen::MatrixXd B(-1 * (tmp * T)); // 矩阵B
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 开始进行线性方程AX=B的求解");
	A.makeCompressed();
	solver.compute(A);
	// 求解线性方程，得到内部点的新坐标
	Eigen::VectorXd Xx = solver.solve(B.block(0, 0, innerPointNum, 1));
	Eigen::VectorXd Xy = solver.solve(B.block(0, 1, innerPointNum, 1));
	Eigen::VectorXd Xz = solver.solve(B.block(0, 2, innerPointNum, 1));

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 线性方程求解完毕");
	
	#pragma region 更新SurfaceTriMesh平面的内部点的坐标
	// 更新原有的surfaceTriMesh的平面内部点
	//#pragma omp parallel for
	for (int innerPointIdx = 0; innerPointIdx < innerPointIdVec.size(); innerPointIdx++)  {
		int innerPointId = innerPointIdVec[innerPointIdx];
		shared_ptr<Vertex> innerVertex = oriSurfaceTriMesh->getVertex(innerPointId);
		innerVertex->point().x() = Xx(pointPosInMat[innerPointId], 0);
		innerVertex->point().y() = Xy(pointPosInMat[innerPointId], 0);
		innerVertex->point().z() = Xz(pointPosInMat[innerPointId], 0);
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- 更新四面体网格模型节点坐标完毕");
	#pragma endregion
}
#pragma endregion

#pragma region 计算三维封闭三角网格均值坐标和求解变形后内部点的坐标
void MeanValueCoordsAlgo::internalCompute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh) {
	// 遍历所有的节点确定表面节点顺序
	concurrent_unordered_map<int, int> nodePosInMat; // 记录节点在矩阵中的位置，节点id - 矩阵中所在的行
	int innerIdx = 0, outerIdx = tetMesh.nNodes() - 1;

	// #pragma omp parallel for
	for (int nodeId = 1; nodeId <= tetMesh.nNodes(); nodeId++) {
		shared_ptr<TetNode> node = tetMesh.getNode(nodeId);
		// #pragma omp critical 
		// {
			if (!node->onSurface())
				nodePosInMat[node->id()] = innerIdx++;
			else
				nodePosInMat[node->id()] = outerIdx--;
		// }
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 表面节点和内部节点顺序索引完成");

	int totNodeNum = tetMesh.nNodes(); // 所有节点个数
	int innerNodeNum = innerIdx, outerNodeNum = tetMesh.nNodes() - innerNodeNum; // 内部节点个数, 表面节点个数
	if (innerNodeNum == 0) return;
	Eigen::SparseMatrix<double> W(innerNodeNum, totNodeNum); W.setZero();// 均值坐标权重矩阵
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 开始构造均值坐标权值矩阵");
	// 遍历每个节点，计算所有节点的均值坐标权重
	// #pragma omp parallel for	 
	for (int nodeId = 1; nodeId <= tetMesh.nNodes(); nodeId++) {
		shared_ptr<TetNode> node = tetMesh.getNode(nodeId);
		if (!node->onSurface()) {
			int pos = nodePosInMat[node->id()];
			
			// 计算当前节点的环绕四面体的均值坐标权重值，更新稀疏矩阵A和常量矩阵B
			double totalW = 0.0;
			concurrent_unordered_map<int, double> weightInc;
			weightInc.clear();

			internalNode3DMeanValueCompute(node, tetMesh, nodePosInMat, totalW, weightInc);
			
			// 更新权值矩阵W
			for (auto &v : weightInc)
				W.insert(pos, v.first) = v.second / totalW;
		}
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 均值坐标权值矩阵构造完毕");

	// 构造X的下半部分，表面的节点
	Eigen::MatrixXd T(outerNodeNum, 3);
	for (const auto &v : surfaceTriMesh.allVertexs()) {
		int pos = nodePosInMat[v.first] - innerNodeNum;
		for (int i = 0; i < 3; i++) 
			T(pos, i) = v.second->point()(i, 0);
	}

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 构造线性方程AX=B的相关已知参数");
	// 求解AX = B 线性方程组
	Eigen::SparseMatrix<double> A(W.block(0, 0, innerNodeNum, innerNodeNum)); // 矩阵A
	
	Eigen::SparseMatrix<double> I(innerNodeNum, innerNodeNum); I.setIdentity();
	A = A - I;
	Eigen::MatrixXd tmp(W.block(0, innerNodeNum, innerNodeNum, outerNodeNum));
	Eigen::MatrixXd B(-1 * (tmp * T)); // 矩阵B
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 开始进行线性方程AX=B的求解");
	A.makeCompressed();
	solver.compute(A);
	// 求解线性方程，得到内部点的新坐标
	Eigen::VectorXd Xx = solver.solve(B.block(0, 0, innerNodeNum, 1));
	Eigen::VectorXd Xy = solver.solve(B.block(0, 1, innerNodeNum, 1));
	Eigen::VectorXd Xz = solver.solve(B.block(0, 2, innerNodeNum, 1));

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 线性方程求解完毕");

	#pragma region 更新TetMesh节点的坐标
	// 更新原有的TetMesh
	// #pragma omp parallel for
	for (int nodeId = 1; nodeId <= totNodeNum; nodeId++) {
		shared_ptr<TetNode> node = tetMesh.getNode(nodeId);
		if (node->onSurface()) {
			Eigen::Vector3d surfaceNode = surfaceTriMesh.getVertex(nodeId)->point();
			node->point().x() = surfaceNode.x();
			node->point().y() = surfaceNode.y();
			node->point().z() = surfaceNode.z();
		} else {
			node->point().x() = Xx(nodePosInMat[nodeId], 0);
			node->point().y() = Xy(nodePosInMat[nodeId], 0);
			node->point().z() = Xz(nodePosInMat[nodeId], 0);
		}
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- 更新四面体网格模型节点坐标完毕");
	#pragma endregion
}
#pragma endregion

#pragma region 计算当前内部点的三维均值坐标权值
void MeanValueCoordsAlgo::internalNode3DMeanValueCompute(
	// 输入参数
	const shared_ptr<TetNode> node,
	const TetMesh &tetMesh,
	const concurrent_unordered_map<int, int> &nodePosInMat,
	// 输出
	double &totalW,
	concurrent_unordered_map<int, double> &weightInc) {

	auto mod3 = [](int i) {return (i + 3) % 3; }; // mod 3函数
	concurrent_vector<int> adjTetIdVec = node->getAdjTetId(); // 与当前点相邻的四面体的id

	// 遍历当前点的所有相邻四面体单元上，对应的三角面片
	// #pragma omp parallel for
	for (int vecIdx = 0; vecIdx < adjTetIdVec.size(); vecIdx++) {
		int adjTetId = adjTetIdVec[vecIdx];
		Eigen::Vector3i triangleNodeId(tetMesh.getTetElement(adjTetId)->getTriangleId(node->id()));
		Eigen::Vector3d p[3], u[3];
		shared_ptr<TetNode> triNode[3];

		for (int i = 0; i < 3; i++) 
			triNode[i] = tetMesh.getNode(triangleNodeId(i, 0));

		double d[3], L[3], theta[3], h = 0, c[3], s[3], w[3];
		for (int i = 0; i < 3; i++) {
			p[i] = triNode[i]->point();
			d[i] = (p[i] - node->point()).norm();
			u[i] = p[i] - node->point();
			u[i].normalize();
		}

		for (int i = 0; i < 3; i++) {
			L[i] = (u[mod3(i + 1)] - u[mod3(i - 1)]).norm();
			theta[i] = 2.0 * asin(L[i] / 2.0);	
			h += theta[i];
		}

		h /= 2;

		for (int i = 0; i < 3; i++){
			c[i] = (2.0 * sin(h) * sin(h - theta[i])) / (sin(theta[mod3(i + 1)]) * sin(theta[mod3(i - 1)])) - 1;
			s[i] = sqrt(1 - c[i] * c[i]);
		}

		for (int i = 0; i < 3; i++) {
			w[i] = (theta[i] - c[mod3(i + 1)] * theta[mod3(i - 1)] - c[mod3(i - 1)] * theta[mod3(i + 1)]) / (d[i] * sin(theta[mod3(i + 1)])*s[mod3(i - 1)]);
			//#pragma omp critical
			//{
				totalW += w[i];
			//}
		}

		for (int i = 0; i < 3; i++) {
			//#pragma omp critical
			//{
				weightInc[nodePosInMat.at(triNode[i]->id())] += w[i];
			//}
		}
	}
}

#pragma endregion

#pragma region 计算内部点二维均值坐标权值
void MeanValueCoordsAlgo::internal2DPointMeanValueCompute(
	const shared_ptr<Vertex> &vertex,
	const concurrent_unordered_map<int, int> &pointPosInMat,
	// 输出
	double &totalW,
	concurrent_unordered_map<int, double> &weightInc) {

	const TriMesh *surfaceTriMesh = vertex->getTriMesh();
	int vertexId = vertex->id();
	Eigen::Vector3d centerV = vertex->point();
	concurrent_vector<int> adjacentVertex = vertex->getAdjancentVertexId();
	assert(adjacentVertex.size() >= 2);
	// #pragma omp parallel for
	for (int i = 0; i < adjacentVertex.size(); i++) {
		int prevVertexId = adjacentVertex[i - 1 < 0 ? adjacentVertex.size() - 1 : i-1];
		int curVertexId = adjacentVertex[i];
		int nextVertexId = adjacentVertex[i + 1 >= adjacentVertex.size() ? 0 : i+1];
		Eigen::Vector3d prevV = surfaceTriMesh->getVertex(prevVertexId)->point();
		Eigen::Vector3d curV = surfaceTriMesh->getVertex(curVertexId)->point();
		Eigen::Vector3d nextV = surfaceTriMesh->getVertex(nextVertexId)->point();
		double tanRes1 = ((curV - centerV).norm() * (prevV - centerV).norm() - (curV - centerV).dot(prevV - centerV)) 
			/ (curV - centerV).cross(prevV - centerV).norm();
		double tanRes2 = ((curV - centerV).norm() * (nextV - centerV).norm() - (curV - centerV).dot(nextV - centerV))
			/ (curV - centerV).cross(nextV - centerV).norm();
		double w = (tanRes1 + tanRes2) / (curV - centerV).norm();
		weightInc[pointPosInMat.at(curVertexId)] = w;
		totalW += w;
	}
}
#pragma endregion