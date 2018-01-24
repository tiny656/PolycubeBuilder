/*
* Author: tiny656
* Date: 2015-10-14
*/

#pragma once

#include "utility.h"
#include "TriMesh.h"

class PolycubeAlgo {
public:
	static void run(TriMesh &mesh); // 算法主函数入口

private:

	// TODO: 添加交互选择Mesh模型进行法向的染色

	static void dye(TriMesh &mesh, concurrent_unordered_map<int, Eigen::Vector3d> &color, concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal); // 对mesh模型进行染色
	
	static double computeEnergy(TriMesh &mesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, double wNor, double wDst); // 计算全局能量大小

	/*
	* 功能:计算全局能量,包括所有面变的法向能量和
	*/
	static void optimizeEnergy(TriMesh &mesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal); // 梯度下降 - 优化能量公式

	/*
	* 功能: 法向能量最优化梯度下降计算
	* 输入参数: 
	*	- const TriMesh &mesh - 当前mesh模型
	*	- const shared_ptr<Face> face -  当前面
	*	- const unordered_map<int, osg::Vec3d> &faceDNormal - 面的坐标轴朝向法向
	* 输出参数:
	*	- unordered_map<int, osg::Vec3d> &partialDerivative - 记录每个点的梯度下降更新值 
	*/
	static void normalGrad(const TriMesh &mesh, const shared_ptr<Face> &face, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, concurrent_unordered_map<int, Eigen::Vector3d> &partialDerivative);

	/*
	* 功能: 扭曲能量最优化梯度下降计算
	* 输入参数:
	**	- const TriMesh &mesh - 当前mesh模型
	*	- const shared_ptr<Face> face -  当前面
	* 输出参数:
	*	- unordered_map<int, osg::Vec3d> &partialDerivative - 记录每个点的梯度下降更新值 
	*/
	static void distortionGrad(const TriMesh &mesh, const shared_ptr<Face> &face, concurrent_unordered_map<int, Eigen::Vector3d> &partialDerivative);
};
