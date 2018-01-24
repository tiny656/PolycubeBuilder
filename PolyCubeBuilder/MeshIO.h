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
	static bool readTriMesh(TriMesh &triMesh, const string &filename); // ��ȡ��������TriMesh
	static bool writeTriMesh(const TriMesh &triMesh, const string &filename); // д��TriMesh

	static bool readTetMesh(TetMesh &tetMesh, const string &filename); // �����ɵ��ļ��ж�ȡ����������TetMesh
	static bool readTetMeshFromInpFile(TetMesh &tetMesh, const string &filename); // ��inp�ļ���, ��ȡ����������TetMesh
	static bool writeTetMesh(TetMesh &tetMesh, const string &filename); // д��TetMesh

	static bool readPointData(concurrent_unordered_map<int, Eigen::Vector3d> &pointData, const string filename);

	static bool readSimuationData(concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData, const string &filename); // ��ȡTetģ��ÿ���ڵ�ķ���ֵ

private:
	static concurrent_vector<string> split(string line, char ch);
};