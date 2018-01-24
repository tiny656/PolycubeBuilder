/*
* author: tiny656
* date: 2015-10-12
*/

#include "utility.h"
#include "TriMesh.h"
#include "TetMesh.h"
#include "MeshIO.h"
#include "MeshViewer.h"
#include "PolycubeAlgo.h"
#include "MeanValueCoordsAlgo.h"
#include "GeometryModel.h"
#include "MappingAlgo.h"

const string INP_DIRECTORY = "model_data/inp_file/";
const string RESULT_DIRECTORY = "model_result/";
const string ORIGIN_SURFACE_DIRECTORY = "origin_surface/";
const string POLYCUBE_SURFACE_DIRECTORY = "polycube_surface/";
const string ORIGIN_TET_DIRECTORY = "origin_tet/";
const string POLYCUBE_TET_DIRECTORY = "polycube_tet/";
const string TOAST_DIRECTORY = "toast_for_this_momnent/";

void polyCubeGenerate(int argc, char **argv) {

	string filename(argv[2]); // 文件名
	TriMesh surfaceTriMesh;
	TetMesh tetMesh;

	// 读入inp中的四面体模型数据
	MeshIO::readTetMeshFromInpFile(tetMesh, INP_DIRECTORY + filename + ".inp");
	MeshIO::writeTetMesh(tetMesh, RESULT_DIRECTORY + ORIGIN_TET_DIRECTORY + filename);


	// 构造表面三角网格模型
	tetMesh.buildSurfaceTriMesh(surfaceTriMesh);
	MeshIO::writeTriMesh(surfaceTriMesh, RESULT_DIRECTORY + ORIGIN_SURFACE_DIRECTORY + filename);

	// 构造法向信息后，才能进行显示
	#ifdef MESH_VIEW
	MeshViewer::show(tetMesh, "原始四面体");
	#endif // MESH_VIEW

	// 对表面三角网格模型进行polycube能量最小化
	PolycubeAlgo::run(surfaceTriMesh);
	MeshIO::writeTriMesh(surfaceTriMesh, RESULT_DIRECTORY + POLYCUBE_SURFACE_DIRECTORY + filename);

	// 三维均值坐标法求解polycube表面下，原始四面体网格内部新坐标
	MeanValueCoordsAlgo::compute3DClosedTriMesh(surfaceTriMesh, tetMesh);
	MeshIO::writeTetMesh(tetMesh, RESULT_DIRECTORY + POLYCUBE_TET_DIRECTORY + filename);
	
	#ifdef MESH_VIEW
	MeshViewer::show(tetMesh, "体参数化");
	#endif

	#ifdef MESH_VIEW
	MeshViewer::run();
	#endif // MESH_VIEW
	
}

void parameterization(int argc, char **argv) {
	// 读入中间模型(参照模型)和其他模型(源模型)
	TetMesh refTetMesh;
	TriMesh refSurfaceTriMesh;
	string refFileName(argv[2]);
	MeshIO::readTetMesh(refTetMesh, RESULT_DIRECTORY + POLYCUBE_TET_DIRECTORY + refFileName);
	MeshIO::readTriMesh(refSurfaceTriMesh, RESULT_DIRECTORY + POLYCUBE_SURFACE_DIRECTORY+ refFileName);
	GeometryModel refGeometryModel(&refSurfaceTriMesh);
	refGeometryModel.buildGeometryModel();

	for (int i = 3; i < argc; i++) {
		//读入源模型
		TetMesh oriTetMesh;
		TriMesh oriSurfaceMesh;
		string oriFileName(argv[i]);
		MeshIO::readTetMesh(oriTetMesh, RESULT_DIRECTORY + POLYCUBE_TET_DIRECTORY + oriFileName);
		MeshIO::readTriMesh(oriSurfaceMesh, RESULT_DIRECTORY + POLYCUBE_SURFACE_DIRECTORY + oriFileName);
		
		#ifdef MESH_VIEW
		MeshViewer::show(oriTetMesh, "体参数化");
		#endif

		GeometryModel oriGeometryModel(&oriSurfaceMesh);
		oriGeometryModel.buildGeometryModel();
		
		#ifdef MESH_VIEW
		MeshViewer::show(oriSurfaceMesh, concurrent_unordered_map<int, Eigen::Vector3d>(), "源模型未处理表面网格");
		#endif

		MappingAlgo::run(oriGeometryModel, refGeometryModel, &oriSurfaceMesh, &refSurfaceTriMesh);

		#ifdef MESH_VIEW
		MeshViewer::show(oriSurfaceMesh, concurrent_unordered_map<int, Eigen::Vector3d>(), "源模型参数化映射后表面网格");
		#endif

		// 三维均值坐标法求解polycube表面下，原始四面体网格内部新坐标, DEBUG Point
		MeanValueCoordsAlgo::compute3DClosedTriMesh(oriSurfaceMesh, oriTetMesh);

		MeshIO::writeTetMesh(oriTetMesh, RESULT_DIRECTORY + TOAST_DIRECTORY + oriFileName);

		#ifdef MESH_VIEW
		MeshViewer::show(oriTetMesh, "源模型体参数化映射到中间模型后的结果");
		#endif
	}

	#ifdef MESH_VIEW
	MeshViewer::run();
	#endif // MESH_VIEW
}


void simulationDisplay(int argc, char **argv) {
	TetMesh tetMesh;
	concurrent_unordered_map<int, Eigen::VectorXd> pointSimulationData;
	string tetMeshfileName(argv[2]);
	string simulationDataFileName(argv[3]);
	MeshIO::readTetMesh(tetMesh, RESULT_DIRECTORY + ORIGIN_TET_DIRECTORY + tetMeshfileName);
	MeshIO::readSimuationData(pointSimulationData, "J:/SimulationDataMining/7_mapping_simulation_data/dengzi_thermal_mapping_result/"+simulationDataFileName);
	//MeshIO::readTetMesh(tetMesh, RESULT_DIRECTORY + TOAST_DIRECTORY + tetMeshfileName);
	//MeshIO::readSimuationData(pointSimulationData, "J:/SimulationDataMining/7_mapping_simulation_data/remapping_result/" + simulationDataFileName);
	assert(tetMesh.nNodes() == pointSimulationData.size());
	MeshViewer::showSimulationResult(tetMesh, pointSimulationData);
	MeshViewer::run();
}

void tetPointDisplay(int argc, char **argv) {
	TetMesh tetMesh;
	concurrent_unordered_map<int, Eigen::Vector3d> pointColor;
	string tetMeshfileName(argv[2]);
	string pointDataFileName(argv[3]);
	MeshIO::readTetMesh(tetMesh, RESULT_DIRECTORY + tetMeshfileName);
	MeshIO::readPointData(pointColor, RESULT_DIRECTORY + "tet_point_show/" + pointDataFileName);
	MeshViewer::showTetMeshPoint(tetMesh, pointColor);
	MeshViewer::run();
}

void compareTetMeshDisplay(int argc, char** argv) {
	TetMesh tetMesh1, tetMesh2;
	

	concurrent_unordered_map<int, Eigen::Vector3d> pointColor1, pointColor2;
	string tetMesh1FileName(argv[2]), tetMesh2FileName(argv[3]);
	// string pointData1FileName(argv[3]), pointData2FileName(argv[5]);
	MeshIO::readTetMesh(tetMesh1, RESULT_DIRECTORY + tetMesh1FileName);
	MeshIO::readTetMesh(tetMesh2, RESULT_DIRECTORY + tetMesh2FileName);
	
	TriMesh triMesh1, triMesh2;
	tetMesh1.buildSurfaceTriMesh(triMesh1);
	//tetMesh2.buildSurfaceTriMesh(triMesh2);
	GeometryModel geometryModel1(&triMesh1);//, geometryModel2(&triMesh2);
	geometryModel1.buildGeometryModel();
	//geometryModel2.buildGeometryModel();

	//MeshIO::readPointData(pointColor1, RESULT_DIRECTORY + "tet_point_show/" + pointData1FileName);
	//MeshIO::readPointData(pointColor2, RESULT_DIRECTORY + "tet_point_show/" + pointData2FileName);
	MeshViewer::showCompareTetMesh(tetMesh1, tetMesh2, pointColor1, pointColor2, geometryModel1);
	MeshViewer::run();
}

void tetMeshDisplay(int argc, char **argv) {
	TetMesh tetMesh;
	string tetMeshFileName(argv[2]);
	MeshIO::readTetMesh(tetMesh, RESULT_DIRECTORY + tetMeshFileName);
	MeshViewer::show(tetMesh);
	MeshViewer::run();
}

void triMeshDisplay(int argc, char **argv) {
	TriMesh triMesh;
	string triMeshFileName(argv[2]);
	MeshIO::readTriMesh(triMesh, RESULT_DIRECTORY + triMeshFileName);
	MeshViewer::show(triMesh, concurrent_unordered_map<int, Eigen::Vector3d>());
	MeshViewer::run();
}

int main(int argc, char** argv) {
	if (argc == 3 && !strcmp(argv[1], "-polycube")) {
		// polycube 生成
		polyCubeGenerate(argc, argv);
	} else if (argc >= 4 && !strcmp(argv[1], "-map")) {
		// 映射结果生成
		parameterization(argc, argv);
	} else if (argc == 4 && !strcmp(argv[1], "-simulate")) {
		simulationDisplay(argc, argv);
	} else if (argc == 4 && !strcmp(argv[1], "-tetpointshow")) {
		tetPointDisplay(argc, argv);
	} else if (argc == 4 && !strcmp(argv[1], "-comparetet")) {
		compareTetMeshDisplay(argc, argv);
	} else if (argc == 3 && !strcmp(argv[1], "-tetshow")) {
		tetMeshDisplay(argc, argv);
	} else if (argc == 3 && !strcmp(argv[1], "-trishow")) {
		triMeshDisplay(argc, argv);
	}
	return 0;
}
