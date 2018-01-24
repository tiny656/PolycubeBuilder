/*
* Author: tiny656
* Date: 2015-10-12
*/

#include "MeshIO.h"
#include "TinyLogger.h"
#include "TriMesh.h"

bool MeshIO::readTriMesh(TriMesh &triMesh, const string &filename) {
	fstream fin(filename, ios_base::in);
	if (fin.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readTriMesh --- 文件打开失败");
		return false;
	}
	int vertexNum, faceNum, id;
	Eigen::Vector3d point;
	fin >> vertexNum >> faceNum;
	for (int i = 0; i < vertexNum; i++) {
		fin >> id >> point.x() >> point.y() >> point.z();
		triMesh.addVertex(id, point);
	} 
	Eigen::Vector3i faceIndex;
	Eigen::Vector3d faceNormal;
	for (int i = 0; i < faceNum; i++) {
		fin >> id >> faceIndex.x() >> faceIndex.y() >> faceIndex.z();
		fin >> faceNormal.x() >> faceNormal.y() >> faceNormal.z();
		triMesh.addFace(id, faceIndex, faceNormal);
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> readTriMesh --- 文件" + filename + "， 读取成功");
	return true;
}

bool MeshIO::writeTriMesh(const TriMesh &triMesh, const string &filename) {
	fstream fout(filename, ios_base::out);
	if (fout.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> writeTriMesh --- 文件打开失败");
		return false;
	}
	fout << triMesh.nVertices() << " " << triMesh.nFaces() << endl;
	for (const auto &v : triMesh.allVertexs()) {
		auto point = v.second->point();
		fout << v.second->id() << " " << point.x() << " " << point.y() << " " << point.z() << endl;
	}
	for (const auto &f : triMesh.allFaces()) {
		auto vertexIdx = f.second->getVertexIndex();
		auto normal = f.second->normal();
		fout << f.second->id() << " " << vertexIdx.x() << " " << vertexIdx.y() << " " << vertexIdx.z();
		fout << " " << normal.x() << " " << normal.y() << " " << normal.z() << endl;
	} 
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> writeTriMesh --- 文件" + filename + "， 写出成功");
	return true;
}

bool MeshIO::readTetMesh(TetMesh& tetMesh, const string& filename) {
	fstream fin(filename, ios_base::in);
	if (fin.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readTetMesh --- 文件打开失败");
		return false;
	}
	int nodeNum, tetNum, id, isOnSurface;
	Eigen::Vector3d point;
	Eigen::Vector4i tetNodeId;
	fin >> nodeNum >> tetNum;
	for (int i = 0; i < nodeNum; i++) {
		fin >> id >> point.x() >> point.y() >> point.z() >> isOnSurface;
		tetMesh.addNode(id, point);
		tetMesh.getNode(id)->setSurfaceState(isOnSurface == 1);
	}
	for (int i = 0; i < tetNum; i++) {
		fin >> id >> tetNodeId.x() >> tetNodeId.y() >> tetNodeId.z() >> tetNodeId.w();
		tetMesh.addTetElement(id, tetNodeId);
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> readTetMesh --- 文件" + filename + "， 读取成功");
	return true;
}

bool MeshIO::readTetMeshFromInpFile(TetMesh &tetMesh, const string &filename) {
	fstream fin(filename, ios_base::in);
	if (fin.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readTetMeshFromInpFile --- 文件打开失败");
		return false;
	}
	string line;
	enum MODE { EMPTY = 0, NODE_MODE, C3D4_MODE};
	MODE mode = EMPTY;
	while (!fin.eof()) {
		getline(fin, line);
		if (mode == NODE_MODE) {
			// 分解Node字符串格式
			auto result = split(line, ',');
			try {
				if (result.size() == 4) {
					int nodeId = stoi(result[0]);
					Eigen::Vector3d point(stod(result[1]), stod(result[2]), stod(result[3]));
					tetMesh.addNode(nodeId, point);
				} else mode = EMPTY;
			} catch (exception& ex) {
				LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readTetMeshFromInpFile --- 读取和解析Node出错，" + string(ex.what()));
				mode = EMPTY;
			}
		} else if (mode == C3D4_MODE) {
			// 分解C3D4字符串格式
			auto result = split(line, ',');
			try {
				if (result.size() == 5) {
					int tetId = stoi(result[0]);
					Eigen::Vector4i tetNodeId(stoi(result[1]), stoi(result[2]), stoi(result[3]), stoi(result[4]));
					tetMesh.addTetElement(tetId, tetNodeId);
				} else mode = EMPTY;
			} catch (exception& ex) {
				LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readTetMeshFromInpFile --- 读取和解析C3D4出错，" + string(ex.what()));
				mode = EMPTY;
			}
		}
		if (line == "*Node") mode = NODE_MODE;
		else if (line == "*Element, type=C3D4") mode = C3D4_MODE;
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> readTetMeshFromInpFile --- 文件" + filename + "， 读取成功");
	return true;
}

bool MeshIO::writeTetMesh(TetMesh &tetMesh, const string &filename) {
	fstream fout(filename, ios_base::out);
	if (fout.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> writeTetMesh --- 文件打开失败");
		return false;
	}
	fout << tetMesh.nNodes() << " " << tetMesh.nTets() << endl;
	for (const auto node : tetMesh.allNodes()) {
		auto point = node.second->point();
		fout << node.second->id() << " " << point.x() << " " << point.y() << " " << point.z() << " " << node.second->onSurface() << endl;
	}
	for (const auto tet : tetMesh.allTetElements()) {
		auto tetNodeId = tet.second->getTetNodeId();
		fout << tet.second->id() << " " << tetNodeId.x() << " " << tetNodeId.y() << " " << tetNodeId.z() << " " << tetNodeId.w() << endl;
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> writeTetMesh --- 文件"+ filename +"， 写出成功");
	return true;
}

bool MeshIO::readPointData(concurrent_unordered_map<int, Eigen::Vector3d>& pointData, const string filename) {
	fstream fin(filename, ios_base::in);
	if (fin.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readPointData --- 文件打开失败");
		return false;
	}
	int node_id;
	Eigen::Vector3d color;
	while (!fin.eof()) {
		fin >> node_id >> color(0, 0) >> color(1, 0) >> color(2, 0);
		pointData[node_id] = color;
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> readPointData --- 文件" + filename + "， 读取成功");
	return true;

}

bool MeshIO::readSimuationData(concurrent_unordered_map<int, Eigen::VectorXd>& pointSimulationData, const string& filename) {
	fstream fin(filename, ios_base::in);
	if (fin.fail()) {
		LOGGER_WRITE_CONSOLE(TinyLogger::ERROR, "MeshIO -> readSimuationData --- 文件打开失败");
		return false;
	}
	int node_id;
	Eigen::VectorXd simulationData(5);
	while (!fin.eof()) {
		fin >> node_id >> simulationData(0, 0) >> simulationData(1, 0) >> simulationData(2, 0) >> simulationData(3, 0) >> simulationData(4, 0);
		pointSimulationData[node_id] = simulationData;
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshIO -> readSimuationData --- 文件" + filename + "， 读取成功");
	return true;
}

concurrent_vector<string> MeshIO::split(string line, char ch) {
	concurrent_vector<string> ret;
	string str = "";
	for (int i = 0; i < line.size(); i++) {
		if (line[i] == ch) {
			if (str != "") {
				ret.push_back(str);
				str = "";
			}
		} else str += line[i];
	}
	if (str != "") ret.push_back(str);
	return ret;
}

