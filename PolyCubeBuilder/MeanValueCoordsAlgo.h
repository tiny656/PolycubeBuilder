/*
* Author: tiny656
* Date: 2015-11-25
*/
#pragma once

#include "utility.h"
#include "TriMesh.h"
#include "TetMesh.h"
#include "GeometryModel.h"

class MeanValueCoordsAlgo {
public:
	static void compute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh); // 三维封闭三角网格

	static void compute2DClosedPolygon(
		const shared_ptr<Plane> plane, 
		TriMesh *oriSurfaceTriMesh,
		TriMesh *refSurfaceTriMesh,
		const concurrent_unordered_map<int, int> &endPointMapping,
		const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos); // 计算三维星形网格

private:
	// 在surfaceTriMesh 的表面情况下，计算并更新内部坐标点
	static void internalCompute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh);

	// 对于GeometryModel的plane插值内部点
	static void internalCompute2DClosedPolygon(
		shared_ptr<Plane> plane, 
		TriMesh *oriSurfaceTriMesh,
		TriMesh *refSurfaceTriMesh,
		const concurrent_unordered_map<int, int> &endPointMapping,
		const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos);

	// 三维均值坐标每个点权重求解， 求解权值矩阵
	static void internalNode3DMeanValueCompute(
		// 输入参数
		const shared_ptr<TetNode> node,
		const TetMesh &tetMesh,
		const concurrent_unordered_map<int, int> &nodePosInMat,
		// 输出参数
		double &totalW,
		concurrent_unordered_map<int, double> &weightInc);

	static void internal2DPointMeanValueCompute(
		const shared_ptr<Vertex> &vertex,
		const concurrent_unordered_map<int, int> &pointPosInMat,
		// 输出
		double &totalW,
		concurrent_unordered_map<int, double> &weightInc);

};