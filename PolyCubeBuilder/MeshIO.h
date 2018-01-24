/*
* Author: tiny656
* Date: 2015-10-12
*/
#pragma once
#include "utility.h"
#include "TriMesh.h"
#include "TetMesh.h"

class MeshIO {
public:
	static bool readTriMesh(TriMesh &triMesh, const string &filename); // 读取三角网格TriMesh
	static bool writeTriMesh(const TriMesh &triMesh, const string &filename); // 写出TriMesh

	static bool readTetMesh(TetMesh &tetMesh, const string &filename); // 从生成的文件中读取四面体网格TetMesh
	static bool readTetMeshFromInpFile(TetMesh &tetMesh, const string &filename); // 从inp文件中, 读取四面体网格TetMesh
	static bool writeTetMesh(TetMesh &tetMesh, const string &filename); // 写出TetMesh

	static bool readPointData(concurrent_unordered_map<int, Eigen::Vector3d> &pointData, const string filename);

	static bool readSimuationData(concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData, const string &filename); // 读取Tet模型每个节点的仿真值

private:
	static concurrent_vector<string> split(string line, char ch);
};