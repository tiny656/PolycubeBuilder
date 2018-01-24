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
	static void compute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh); // ��ά�����������

	static void compute2DClosedPolygon(
		const shared_ptr<Plane> plane, 
		TriMesh *oriSurfaceTriMesh,
		TriMesh *refSurfaceTriMesh,
		const concurrent_unordered_map<int, int> &endPointMapping,
		const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos); // ������ά��������

private:
	// ��surfaceTriMesh �ı�������£����㲢�����ڲ������
	static void internalCompute3DClosedTriMesh(const TriMesh &surfaceTriMesh, TetMesh &tetMesh);

	// ����GeometryModel��plane��ֵ�ڲ���
	static void internalCompute2DClosedPolygon(
		shared_ptr<Plane> plane, 
		TriMesh *oriSurfaceTriMesh,
		TriMesh *refSurfaceTriMesh,
		const concurrent_unordered_map<int, int> &endPointMapping,
		const concurrent_unordered_map<int, Eigen::Vector3d> &oriBoundryPointNewPos);

	// ��ά��ֵ����ÿ����Ȩ����⣬ ���Ȩֵ����
	static void internalNode3DMeanValueCompute(
		// �������
		const shared_ptr<TetNode> node,
		const TetMesh &tetMesh,
		const concurrent_unordered_map<int, int> &nodePosInMat,
		// �������
		double &totalW,
		concurrent_unordered_map<int, double> &weightInc);

	static void internal2DPointMeanValueCompute(
		const shared_ptr<Vertex> &vertex,
		const concurrent_unordered_map<int, int> &pointPosInMat,
		// ���
		double &totalW,
		concurrent_unordered_map<int, double> &weightInc);

};