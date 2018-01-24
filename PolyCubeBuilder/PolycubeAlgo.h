/*
* Author: tiny656
* Date: 2015-10-14
*/

#pragma once

#include "utility.h"
#include "TriMesh.h"

class PolycubeAlgo {
public:
	static void run(TriMesh &mesh); // �㷨���������

private:

	// TODO: ��ӽ���ѡ��Meshģ�ͽ��з����Ⱦɫ

	static void dye(TriMesh &mesh, concurrent_unordered_map<int, Eigen::Vector3d> &color, concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal); // ��meshģ�ͽ���Ⱦɫ
	
	static double computeEnergy(TriMesh &mesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, double wNor, double wDst); // ����ȫ��������С

	/*
	* ����:����ȫ������,�����������ķ���������
	*/
	static void optimizeEnergy(TriMesh &mesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal); // �ݶ��½� - �Ż�������ʽ

	/*
	* ����: �����������Ż��ݶ��½�����
	* �������: 
	*	- const TriMesh &mesh - ��ǰmeshģ��
	*	- const shared_ptr<Face> face -  ��ǰ��
	*	- const unordered_map<int, osg::Vec3d> &faceDNormal - ��������ᳯ����
	* �������:
	*	- unordered_map<int, osg::Vec3d> &partialDerivative - ��¼ÿ������ݶ��½�����ֵ 
	*/
	static void normalGrad(const TriMesh &mesh, const shared_ptr<Face> &face, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, concurrent_unordered_map<int, Eigen::Vector3d> &partialDerivative);

	/*
	* ����: Ť���������Ż��ݶ��½�����
	* �������:
	**	- const TriMesh &mesh - ��ǰmeshģ��
	*	- const shared_ptr<Face> face -  ��ǰ��
	* �������:
	*	- unordered_map<int, osg::Vec3d> &partialDerivative - ��¼ÿ������ݶ��½�����ֵ 
	*/
	static void distortionGrad(const TriMesh &mesh, const shared_ptr<Face> &face, concurrent_unordered_map<int, Eigen::Vector3d> &partialDerivative);
};
