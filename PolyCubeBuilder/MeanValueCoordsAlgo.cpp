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
	Timer timer; // ��ʱ��
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute3DClosedTriMesh --- ��ʼ���о�ֵ�����ڲ����ֵ����");
	internalCompute3DClosedTriMesh(surfaceTriMesh, tetMesh); // ����
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute3DClosedTriMesh --- ��ֵ�������, ����ʱ" + to_string(timer.elapsed_seconds()) + "��");
}

void MeanValueCoordsAlgo::compute2DClosedPolygon(
	const shared_ptr<Plane> plane,
	TriMesh *oriSurfaceTriMesh,
	TriMesh *refSurfaceTriMesh,
	const concurrent_unordered_map<int, int> &endPointMapping,
	const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos) {
	Timer timer; // ��ʱ��
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute2DClosedPolygon --- ��ʼ���о�ֵ�����ڲ����ֵ����");
	internalCompute2DClosedPolygon(plane, oriSurfaceTriMesh, refSurfaceTriMesh, endPointMapping, oriBoundryPointNewPos); // ����
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> compute2DClosedPolygon --- ��ֵ�������, ����ʱ" + to_string(timer.elapsed_seconds()) + "��");
}

#pragma region ������ά��Polygon�ڲ�������
void MeanValueCoordsAlgo::internalCompute2DClosedPolygon(
	shared_ptr<Plane> plane, 
	TriMesh *oriSurfaceTriMesh,
	TriMesh *refSurfaceTriMesh,
	const concurrent_unordered_map<int, int> &endPointMapping,
	const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos) {
	// �������еĽڵ�ȷ������ڵ�˳��
	concurrent_unordered_map<int, int> pointPosInMat; // ��¼�ڵ��ھ����е�λ�ã��ڵ�id - ���������ڵ���
	int pointIdx = 0;
	for (const auto &pointId : plane->allInnerPointId()) pointPosInMat[pointId] = pointIdx++;
	for (const auto &pointId : plane->allEndPointId()) pointPosInMat[pointId] = pointIdx++;
	for (const auto &pointId : plane->allBoundryPointId()) pointPosInMat[pointId] = pointIdx++;

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- ����ڵ�ͱ����ڲ��ڵ�˳���������");

	int totPointNum = plane->nInnerPoint() + plane->nEndPoint() + plane->nBoundryPoint(); // ���нڵ����
	int innerPointNum = plane->nInnerPoint(), outerNodeNum = totPointNum - innerPointNum; // �ڲ��ڵ����, ����ڵ����
	if (innerPointNum == 0) return;

	// Ȩ��ϡ�����
	Eigen::SparseMatrix<double> W(innerPointNum, totPointNum); W.setZero();
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- ��ʼ�����ֵ����Ȩֵ����");
	
	// ����ÿ���ڵ㣬�������нڵ�ľ�ֵ����Ȩ��
	const GeometryModel* oriGeometryModel = plane->getGeometryModel();

	auto innerPointIdVec = plane->allInnerPointId();
	//#pragma omp parallel for	 
	for (int innerPointIdx = 0; innerPointIdx < innerPointIdVec.size(); innerPointIdx++) {
		int innerPointId = innerPointIdVec[innerPointIdx];
		shared_ptr<Vertex> innerVertex = oriSurfaceTriMesh->getVertex(innerPointId);
		int pos = pointPosInMat[innerVertex->id()];
		// ���㵱ǰ��Ļ�����������ľ�ֵ����Ȩ��ֵ������ϡ�����A�ͳ�������B
		double totalW = 0.0;
		concurrent_unordered_map<int, double> weightInc;
		weightInc.clear();
		internal2DPointMeanValueCompute(innerVertex, pointPosInMat, totalW, weightInc);

		// ����Ȩֵ����W
		for (auto &v : weightInc)
			W.insert(pos, v.first) = v.second / totalW;
	}
	
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- ��ֵ����Ȩֵ���������");

	// ����X���°벿�֣�ƽ��߽�Ľڵ�
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
	
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- �������Է���AX=B�������֪����");
	// ���AX = B ���Է�����
	Eigen::SparseMatrix<double> A(W.block(0, 0, innerPointNum, innerPointNum)); // ����A

	Eigen::SparseMatrix<double> I(innerPointNum, innerPointNum); I.setIdentity();
	A = A - I;
	Eigen::MatrixXd tmp(W.block(0, innerPointNum, innerPointNum, outerNodeNum));
	Eigen::MatrixXd B(-1 * (tmp * T)); // ����B
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- ��ʼ�������Է���AX=B�����");
	A.makeCompressed();
	solver.compute(A);
	// ������Է��̣��õ��ڲ����������
	Eigen::VectorXd Xx = solver.solve(B.block(0, 0, innerPointNum, 1));
	Eigen::VectorXd Xy = solver.solve(B.block(0, 1, innerPointNum, 1));
	Eigen::VectorXd Xz = solver.solve(B.block(0, 2, innerPointNum, 1));

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- ���Է���������");
	
	#pragma region ����SurfaceTriMeshƽ����ڲ��������
	// ����ԭ�е�surfaceTriMesh��ƽ���ڲ���
	//#pragma omp parallel for
	for (int innerPointIdx = 0; innerPointIdx < innerPointIdVec.size(); innerPointIdx++)  {
		int innerPointId = innerPointIdVec[innerPointIdx];
		shared_ptr<Vertex> innerVertex = oriSurfaceTriMesh->getVertex(innerPointId);
		innerVertex->point().x() = Xx(pointPosInMat[innerPointId], 0);
		innerVertex->point().y() = Xy(pointPosInMat[innerPointId], 0);
		innerVertex->point().z() = Xz(pointPosInMat[innerPointId], 0);
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute2DClosedPolygon --- ��������������ģ�ͽڵ��������");
	#pragma endregion
}
#pragma endregion

#pragma region ������ά������������ֵ����������κ��ڲ��������
void MeanValueCoordsAlgo::internalCompute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh) {
	// �������еĽڵ�ȷ������ڵ�˳��
	concurrent_unordered_map<int, int> nodePosInMat; // ��¼�ڵ��ھ����е�λ�ã��ڵ�id - ���������ڵ���
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
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- ����ڵ���ڲ��ڵ�˳���������");

	int totNodeNum = tetMesh.nNodes(); // ���нڵ����
	int innerNodeNum = innerIdx, outerNodeNum = tetMesh.nNodes() - innerNodeNum; // �ڲ��ڵ����, ����ڵ����
	if (innerNodeNum == 0) return;
	Eigen::SparseMatrix<double> W(innerNodeNum, totNodeNum); W.setZero();// ��ֵ����Ȩ�ؾ���
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- ��ʼ�����ֵ����Ȩֵ����");
	// ����ÿ���ڵ㣬�������нڵ�ľ�ֵ����Ȩ��
	// #pragma omp parallel for	 
	for (int nodeId = 1; nodeId <= tetMesh.nNodes(); nodeId++) {
		shared_ptr<TetNode> node = tetMesh.getNode(nodeId);
		if (!node->onSurface()) {
			int pos = nodePosInMat[node->id()];
			
			// ���㵱ǰ�ڵ�Ļ���������ľ�ֵ����Ȩ��ֵ������ϡ�����A�ͳ�������B
			double totalW = 0.0;
			concurrent_unordered_map<int, double> weightInc;
			weightInc.clear();

			internalNode3DMeanValueCompute(node, tetMesh, nodePosInMat, totalW, weightInc);
			
			// ����Ȩֵ����W
			for (auto &v : weightInc)
				W.insert(pos, v.first) = v.second / totalW;
		}
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- ��ֵ����Ȩֵ���������");

	// ����X���°벿�֣�����Ľڵ�
	Eigen::MatrixXd T(outerNodeNum, 3);
	for (const auto &v : surfaceTriMesh.allVertexs()) {
		int pos = nodePosInMat[v.first] - innerNodeNum;
		for (int i = 0; i < 3; i++) 
			T(pos, i) = v.second->point()(i, 0);
	}

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- �������Է���AX=B�������֪����");
	// ���AX = B ���Է�����
	Eigen::SparseMatrix<double> A(W.block(0, 0, innerNodeNum, innerNodeNum)); // ����A
	
	Eigen::SparseMatrix<double> I(innerNodeNum, innerNodeNum); I.setIdentity();
	A = A - I;
	Eigen::MatrixXd tmp(W.block(0, innerNodeNum, innerNodeNum, outerNodeNum));
	Eigen::MatrixXd B(-1 * (tmp * T)); // ����B
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- ��ʼ�������Է���AX=B�����");
	A.makeCompressed();
	solver.compute(A);
	// ������Է��̣��õ��ڲ����������
	Eigen::VectorXd Xx = solver.solve(B.block(0, 0, innerNodeNum, 1));
	Eigen::VectorXd Xy = solver.solve(B.block(0, 1, innerNodeNum, 1));
	Eigen::VectorXd Xz = solver.solve(B.block(0, 2, innerNodeNum, 1));

	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- ���Է���������");

	#pragma region ����TetMesh�ڵ������
	// ����ԭ�е�TetMesh
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
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeanValueCoordsAlgo -> internalCompute3DClosedTriMesh --- ��������������ģ�ͽڵ��������");
	#pragma endregion
}
#pragma endregion

#pragma region ���㵱ǰ�ڲ������ά��ֵ����Ȩֵ
void MeanValueCoordsAlgo::internalNode3DMeanValueCompute(
	// �������
	const shared_ptr<TetNode> node,
	const TetMesh &tetMesh,
	const concurrent_unordered_map<int, int> &nodePosInMat,
	// ���
	double &totalW,
	concurrent_unordered_map<int, double> &weightInc) {

	auto mod3 = [](int i) {return (i + 3) % 3; }; // mod 3����
	concurrent_vector<int> adjTetIdVec = node->getAdjTetId(); // �뵱ǰ�����ڵ��������id

	// ������ǰ����������������嵥Ԫ�ϣ���Ӧ��������Ƭ
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

#pragma region �����ڲ����ά��ֵ����Ȩֵ
void MeanValueCoordsAlgo::internal2DPointMeanValueCompute(
	const shared_ptr<Vertex> &vertex,
	const concurrent_unordered_map<int, int> &pointPosInMat,
	// ���
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