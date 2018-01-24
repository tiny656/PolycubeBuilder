/*
* Author: tiny656
* Date: 2015-12-03
*/
#pragma once

#include "utility.h"
#include "GeometryModel.h"

class MappingAlgo {

public:
	static void run(
		const GeometryModel &oriGeometryModel,
		const GeometryModel &refGeometryModel,
		TriMesh *oriSurfaceTriMesh,
		TriMesh *refSurfaceTriMesh);

private:
	// ���Ҳο�ģ�ͺ�Դģ�͵��������Ӷ�Ӧ��
	static pair<int, int> findSeedOfEndPoint(
		const GeometryModel &oriGeometryModel,
		const GeometryModel &refGeometryModel,
		concurrent_unordered_map<int, bool> &visOriEndPoint,
		concurrent_unordered_map<int, bool> &visRefEndPoint);
	
	static void mappingEndPoint(
		const pair<int, int> seedPair,
		const GeometryModel& oriGeometryModel,
		const GeometryModel& refGeometryModel,
		concurrent_unordered_map<int, bool> &visOriEndPoint,
		concurrent_unordered_map<int, bool> &visRefEndPoint,
		concurrent_unordered_map<int, int> &endPointMappingId);
};