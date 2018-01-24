/*
* Author: tiny656
* Date: 2015-10-12
*/
#pragma once
#include "utility.h"
#include "TriMesh.h"
#include "TetMesh.h"

class GeometryModel;

class MeshViewer {
public:
	static void show(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceColor, const string &windowName = ""); // ��Trimeshģ�ͼ�����ʾ�б�
	static void show(const TetMesh &tetMesh, const string &windowName = ""); //
	
	static void showTriMeshPoint(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor, const string &windowName = ""); // ��Trimeshģ�ͼ�����ʾ�б�
	static void showTetMeshPoint(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor, const string &windowName = "");

	static void showSimulationResult(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData, const string &windowName = ""); // ��ʾ������ֵ

	static void showCompareTetMesh(const TetMesh &tetMesh1, const TetMesh &tetMesh2,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor1,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor2,
		GeometryModel &geometryModel,
		const string &windowName = ""); // ��ʾ����TetMesh

	static void run(); // ��ʾ�����б��е�meshģ��

private:
	MeshViewer() {}
	MeshViewer(const MeshViewer &);
	MeshViewer& operator=(const MeshViewer &);
	
	static osg::Drawable* internalFaceColorBuildDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceColor); // ����faceColor����TriMesh��Drawable���ƶ���
	static osg::Drawable* internalPointColorBuildDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor); // ����pointColor����TriMesh��Drawable���ƶ���
	static osg::Drawable* internalBuildDrawable(const TetMesh &tetMesh);

	static osg::Drawable* internalPointColorBuildDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor); 
	static osg::Drawable* internalSimuationBuildDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData); // ����pointSimulationData����TetMesh�ķ�����Drawable���ƶ���
	static osg::Drawable* internalSimuationLineBuildDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData); // ����pointSimulationData����TetMesh�ķ�����Drawable���ƶ���
	
	static osg::Drawable* internalCompareTetMeshBuildDrawable(const TetMesh &tetMesh1, const TetMesh &tetMesh2,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor1,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor2); // ��������TetMesh�ıȽϽ��Drawable���ƶ���
	
	// ��
	static osg::Drawable* internalLineDrawable(const TriMesh &triMesh);
	static osg::Drawable* internalLineDrawable(const TetMesh &tetMesh);
	

	// ��
	static osg::Drawable* internalPointDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor);
	static osg::Drawable* internalPointDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor);


	// չʾGeometry�߽�
	static osg::Drawable* internalBoundryBuildDrawable(const TetMesh &tetMesh, const GeometryModel &geometryModel);


	static osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer; // Viewer����
};
