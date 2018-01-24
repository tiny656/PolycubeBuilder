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
	static void show(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceColor, const string &windowName = ""); // 将Trimesh模型加入显示列表
	static void show(const TetMesh &tetMesh, const string &windowName = ""); //
	
	static void showTriMeshPoint(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor, const string &windowName = ""); // 将Trimesh模型加入显示列表
	static void showTetMeshPoint(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor, const string &windowName = "");

	static void showSimulationResult(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData, const string &windowName = ""); // 显示仿真结果值

	static void showCompareTetMesh(const TetMesh &tetMesh1, const TetMesh &tetMesh2,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor1,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor2,
		GeometryModel &geometryModel,
		const string &windowName = ""); // 显示两个TetMesh

	static void run(); // 显示所有列表中的mesh模型

private:
	MeshViewer() {}
	MeshViewer(const MeshViewer &);
	MeshViewer& operator=(const MeshViewer &);
	
	static osg::Drawable* internalFaceColorBuildDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceColor); // 按照faceColor生成TriMesh的Drawable绘制对象
	static osg::Drawable* internalPointColorBuildDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor); // 按照pointColor生成TriMesh的Drawable绘制对象
	static osg::Drawable* internalBuildDrawable(const TetMesh &tetMesh);

	static osg::Drawable* internalPointColorBuildDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor); 
	static osg::Drawable* internalSimuationBuildDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData); // 按照pointSimulationData生成TetMesh的仿真结果Drawable绘制对象
	static osg::Drawable* internalSimuationLineBuildDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData); // 按照pointSimulationData生成TetMesh的仿真结果Drawable绘制对象
	
	static osg::Drawable* internalCompareTetMeshBuildDrawable(const TetMesh &tetMesh1, const TetMesh &tetMesh2,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor1,
		const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor2); // 生成两个TetMesh的比较结果Drawable绘制对象
	
	// 线
	static osg::Drawable* internalLineDrawable(const TriMesh &triMesh);
	static osg::Drawable* internalLineDrawable(const TetMesh &tetMesh);
	

	// 点
	static osg::Drawable* internalPointDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor);
	static osg::Drawable* internalPointDrawable(const TetMesh &tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor);


	// 展示Geometry边界
	static osg::Drawable* internalBoundryBuildDrawable(const TetMesh &tetMesh, const GeometryModel &geometryModel);


	static osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer; // Viewer容器
};
