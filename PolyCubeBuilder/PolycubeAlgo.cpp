/*
* Author: tiny656
* Date: 2015-10-15
*/
#include "PolycubeAlgo.h"
#include "TinyLogger.h"
#include "MeshViewer.h"
#include "Timer.h"

void PolycubeAlgo::run(TriMesh &triMesh) {
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- ��ʼִ��Polycube����");
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- ģ�͹� " + to_string(triMesh.nFaces()) + " ����, " + to_string(triMesh.nVertices()) + "����");
	
	Timer timer; // ��ʱ��
	concurrent_unordered_map<int, Eigen::Vector3d> color, faceDNormal;

	// 1. ���ݷ������ƬȾɫ
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- ��ʼ��ģ�ͽ���Ⱦɫ");
	dye(triMesh, color, faceDNormal); // dye mesh MeshȾɫ, ���� ��ɫ��Ϣ �� ������Ƭ�ı�׼����
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- Ⱦɫ���, ����ʱ" + to_string(timer.elapsed_seconds()) + "��");
	
	#ifdef MESH_VIEW
	MeshViewer::show(triMesh, color, "Ⱦɫ");
	#endif // MESH_VIEW

	// 2. �����Ż�����αPolycube
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- ��ʼ��ģ�ͽ����������Ż�");
	timer.reset();
	optimizeEnergy(triMesh, faceDNormal);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- �������Ż����, ����ʱ" + to_string(timer.elapsed_seconds()) + "��");

	#ifdef MESH_VIEW
	MeshViewer::show(triMesh, color, "�������Ż�");
	#endif // MESH_VIEW
}

void PolycubeAlgo::dye(TriMesh &triMesh, concurrent_unordered_map<int, Eigen::Vector3d> &color, concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal) {
	#pragma omp parallel for
	for (int i = 1; i <= triMesh.nFaces(); i++) {
		auto normal = triMesh.getFace(i)->normal();
		double x = normal.x(), y = normal.y(), z = normal.z();
		if (x > 0 && fabs(x) >= fabs(y) && fabs(x) >= fabs(z)) {
			color[i] = Eigen::Vector3d(1.0, 0.0, 0.0); // X - red
			faceDNormal[i] = Eigen::Vector3d(1, 0, 0);
		} else if (x < 0 && fabs(x) >= fabs(y) && fabs(x) >= fabs(z)) {
			color[i] = Eigen::Vector3d(1.0, 125.0 / 255.0, 0.0); // -X - red
			faceDNormal[i] = Eigen::Vector3d(-1, 0, 0);
		} else if (y > 0 && fabs(y) > fabs(x) && fabs(y) > fabs(z)) {
			color[i] = Eigen::Vector3d(0.0, 1.0, 0.0); // Y - green  // 
			faceDNormal[i] = Eigen::Vector3d(0, 1, 0);
		} else if (y < 0 && fabs(y) > fabs(x) && fabs(y) > fabs(z)) {
			color[i] = Eigen::Vector3d(1.0, 1.0, 0.0); // -Y - green //
			faceDNormal[i] = Eigen::Vector3d(0, -1, 0);
		} else if (z > 0 && fabs(z) > fabs(x) && fabs(z) > fabs(y)) {
			color[i] = Eigen::Vector3d(0.0, 0.0, 1.0); // Z - blue
			faceDNormal[i] = Eigen::Vector3d(0, 0, 1);
		} else if (z < 0 && fabs(z) > fabs(x) && fabs(z) > fabs(y)) {
			color[i] = Eigen::Vector3d(1.0, 0.0, 1.0); // -Z - blue
			faceDNormal[i] = Eigen::Vector3d(0, 0, -1);
		}
	}
}


void PolycubeAlgo::optimizeEnergy(TriMesh &triMesh, const concurrent_unordered_map<int, Eigen ::Vector3d> &faceDNormal) {
	//TODO: �����Ƿ���Ը��������Ż�ģ��, SGD, ADAGD, ADADELTA��
	#pragma region �ݶ��½�����ģ�Ͳ���
	int max_iter_times = 100000;
	double learnRate = 0.005; // learn rate ѧϰ�ٶ�a
	double wNor = 1, wDst = 0; // ����������Ť��������Ȩ�ء������Ĺ����з���������Ȩ�ر��ֲ��䣬Ť��������Ȩ������Сֱ��0
	#pragma endregion

	const int NUMS_PRINT_ENERGY = 250; // ��ӡ���������Ƶ��
	const int BREAK_CONDITION = 250; // �ж��Ƿ���������
	int iter_times = 1; // max iterator times ��¼��ǰ��������
	double preEnergy = 1e16, curEnergy = computeEnergy(triMesh, faceDNormal, wNor, wDst); // �洢ǰһ�ε����������бȽϿ��Ƿ�����
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> optimizeEnergy --- ��ʼ����ģ�͵�����: " + to_string(curEnergy));
	while ((curEnergy < preEnergy || iter_times < max_iter_times) && curEnergy > 1e-5) {

		concurrent_unordered_map<int, Eigen::Vector3d> prNor, prDst; // ����������Ť���������ݶȵ�������, �ֱ��ʾ��id���㣬��x,y,z�ϵ�����
		for (auto v : triMesh.allVertexs()) {
			prNor[v.first].setZero();
			prDst[v.first].setZero();
		}
		
		#pragma omp parallel for
		for (int i = 1; i <= triMesh.nFaces(); i++) {
			auto face = triMesh.getFace(i);
			normalGrad(triMesh, face, faceDNormal, prNor); // ���㷨�������ݶȸ���ֵ
			distortionGrad(triMesh, face, prDst); // ����Ť�������ݶȸ���ֵ
		}

		// �ݶ��½�����Meshģ���ϵ���������ֵ
		// TODO: ���м���,unordered_map�Ĳ�����Ҫʹ��bucket��ʽ
		for (auto v : triMesh.allVertexs()) {
			auto vertex = v.second;
			for (int j = 0; j < 3; j++) 
				vertex->point()(j, 0) -= 2.0 * learnRate / vertex->getFaceId().size() * ((wNor * prNor[v.first](j, 0)) + (wDst * prDst[v.first](j, 0)));
		}

		#ifdef _DEBUG // ����ģʽ�£���ӡÿһ�ε�����ֵ
		LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "PolycubeAlgo -> optimizeEnergy --- �� " + to_string(iter_times) + "�ε���");
		#endif

		preEnergy = curEnergy; //�洢��ǰ������
		curEnergy = computeEnergy(triMesh, faceDNormal, wNor, wDst);

		// wDst *= 0.95; // ����Ť��������ϵ��
		// if (iter_times >= 510) wDst = 0;
		// ����ʼ��ɢ��ʱ�򣬽���ѧϰ��
		if (curEnergy > preEnergy)  learnRate *= 0.80;
		
		// if (curEnergy < preEnergy) learnRate += learnRate * 0.1;
		#ifdef NDEBUG // �ǵ���ģʽ��,��NUMS_PRINT_ENERGY
		if (iter_times <=  250 || iter_times % NUMS_PRINT_ENERGY == 0)
			LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> optimizeEnergy --- �� " + to_string(iter_times) + "�ε���, " + "������ֵΪ: " + to_string(curEnergy));
		#endif

		// TODO: ��ӵ�������һ�������Ƿ�����
		// �������Ϊ1000,1500,2000,2500...ʱ�򣬽��ܿ���̨�����Ƿ�Ҫ����ѭ��
		// �Ƿ�̬����ѧϰ�ʣ�ÿ�����ٴ�
		if (iter_times % BREAK_CONDITION == 0) {
			string op = "";
			// cout << "�Ƿ���Ҫ�����ݶ��½� - ��:Y(y)";
			// cin >> op;
			if (op == "Y" || op == "y") break;
		}
		iter_times++; // ��������+1
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo->optimizeEnergy --- �����Ż������յ�����ֵΪ: " + to_string(curEnergy));

	// ����ÿ����ķ���,���ò��м���
	#pragma omp parallel for
	for (int faceId = 1; faceId <= triMesh.nFaces(); faceId++) {
		auto face = triMesh.getFace(faceId);
		Eigen::Vector3i vertexIdOfFace = face->getVertexIndex();
		Eigen::Vector3d v[3];
		for (int i = 0; i < 3; i++) v[i] = triMesh.getVertex(vertexIdOfFace(i, 0))->point();
		face->normal() = Eigen::Vector3d((v[2] - v[0]).cross(v[1] - v[0]));
		face->normal().normalize();
		if (face->normal().dot(faceDNormal.at(faceId)) < 0) face->normal() *= -1;
	}
}

#pragma region ģ������������
double PolycubeAlgo::computeEnergy(TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, double wNor, double wDst) {
	double E = 0, normalEnergy = 0, distortionEnergy = 0; // ������,��������,Ť������
	#pragma omp parallel for
	for (int i = 1; i <= triMesh.nFaces(); i++) {
		auto face = triMesh.getFace(i);
		Eigen::Vector3i verIdx = face->getVertexIndex();
		auto fArea = face->area(); // ��ǰ������
		auto A = triMesh.getVertex(verIdx(0, 0))->point(), B = triMesh.getVertex(verIdx(1, 0))->point(), C = triMesh.getVertex(verIdx(2, 0))->point(); // ��ǰ���ϵ�������
		auto AB = B - A, AC = C - A;

		#pragma region ��Ƭ������������
		/* ��Ƭ�������� */
		Eigen::Vector3d D = faceDNormal.at(face->id());
		Eigen::Vector3d N = AB.cross(AC);
		auto N_ = Eigen::Vector3d(N);
		N_.normalize();
		// ���㷨�����
		int sig = N_.dot(D) < 0 ? -1 : 1;
		N_ *= sig;

		// �ۼӷ������� 
		// ���й�����������
		#pragma omp critical
		{
			normalEnergy += fArea * (N_ - D).squaredNorm();
		}
		#pragma endregion

		#pragma region ��ƬŤ����������
		/* ��ƬŤ������ */
		// ������Ƭuvƽ������
		Eigen::Array2d Auv, Buv, Cuv;
		Auv << 0, 0;
		Buv << AB.norm(), 0;
		Cuv << AB.dot(AC) / AB.norm(), (AB.cross(AC)).norm() / AB.norm();
		Eigen::MatrixXd MA(4, 4), MA_inv(4, 4); // ����A
		MA << Auv(0) - Cuv(0), Auv(1) - Cuv(1), 0, 0,
			0, 0, Auv(0) - Cuv(0), Auv(1) - Cuv(1),
			Buv(0) - Cuv(0), Buv(1) - Cuv(1), 0, 0,
			0, 0, Buv(0) - Cuv(0), Buv(1) - Cuv(1);
		MA_inv = MA.inverse();// ����A����
		Eigen::MatrixXd Wxy(4, 1), Wxz(4, 1), Wyz(4, 1);
		Eigen::MatrixXd Bxy(4, 1), Bxz(4, 1), Byz(4, 1);
		Bxy << A.x() - C.x(), A.y() - C.y(), B.x() - C.x(), B.y() - C.y();
		Bxz << A.x() - C.x(), A.z() - C.z(), B.x() - C.x(), B.z() - C.z();
		Byz << A.y() - C.y(), A.z() - C.z(), B.y() - C.y(), B.z() - C.z();
		// ����Wxy, Wxz, Wyz
		Wxy = MA_inv * Bxy; Wxz = MA_inv * Bxz; Wyz = MA_inv * Byz;
		// ����Ť������
		// ���й�����������
		#pragma omp critical
		{
			distortionEnergy += fArea * (pow((Wxy.block<2, 1>(0, 0).squaredNorm() - 1), 2) + pow((Wxy.block<2, 1>(2, 0).squaredNorm() - 1), 2)
				+ pow((Wxz.block<2, 1>(0, 0).squaredNorm() - 1), 2) + pow((Wxz.block<2, 1>(2, 0).squaredNorm() - 1), 2)
				+ pow((Wyz.block<2, 1>(0, 0).squaredNorm() - 1), 2) + pow((Wyz.block<2, 1>(2, 0).squaredNorm() - 1), 2)
				+ pow(Wxy.block<2, 1>(0, 0).transpose() * Wxy.block<2, 1>(2, 0), 2)
				+ pow(Wxz.block<2, 1>(0, 0).transpose() * Wxz.block<2, 1>(2, 0), 2)
				+ pow(Wyz.block<2, 1>(0, 0).transpose() * Wyz.block<2, 1>(2, 0), 2));
		}
		#pragma endregion
	}
	E = wNor * normalEnergy + wDst * distortionEnergy;

	#ifdef _DEBUG
	LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "PolycubeAlgo -> computeEnergy --- ȫ������: " + to_string(E)
		+ ", ��������: " + to_string(wNor * normalEnergy)
		+ ", Ť������: " + to_string(wDst * distortionEnergy));
	#endif

	return E;
}
#pragma endregion

#pragma region ���������ݶ��������㷽��
void PolycubeAlgo::normalGrad(const TriMesh &triMesh, const shared_ptr<Face> &face, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, concurrent_unordered_map<int, Eigen::Vector3d> &prNor) {
	Eigen::Vector3i verIdx = face->getVertexIndex();
	double fArea = face->area(); // ��ǰ������
	auto A = triMesh.getVertex(verIdx[0])->point(), B = triMesh.getVertex(verIdx[1])->point(), C = triMesh.getVertex(verIdx[2])->point(); // ��ǰ���ϵ�������
	auto AB = B - A, AC = C - A;
	Eigen::Vector3d N = AB.cross(AC); // N ����
	auto L = N.norm(); // L ģ
	Eigen::Vector3d N_ = N / L; // N' ������ N������partialDerivative��λ����
	Eigen::Vector3d D(faceDNormal.at(face->id())[0], faceDNormal.at(face->id())[1], faceDNormal.at(face->id())[2]); // D ��������׼��������

	// ���㷨�����
	int sig = N_.dot(D) < 0 ? -1 : 1;

	// Nxƫ��3���� pNa - pNAx,pNAy,pNAz | pNb - pNBx,pNBy,pNBz | pNc - pNCx,pNCy,pNCz
	Eigen::Vector3d pNa[3], pNb[3], pNc[3];
	pNa[0] = Eigen::Vector3d(0, A.z() - B.z() + C.z() - A.z(), A.y() - C.y() + B.y() - A.y());
	pNa[1] = Eigen::Vector3d(A.z() - C.z() + B.z() - A.z(), 0, A.x() - B.x() + C.x() - A.x());
	pNa[2] = Eigen::Vector3d(A.y() - B.y() + C.y() - A.y(), A.x() - C.x() + B.x() - A.x(), 0);
	pNb[0] = Eigen::Vector3d(0, A.z() - C.z(), C.y() - A.y());
	pNb[1] = Eigen::Vector3d(C.z() - A.z(), 0, A.x() - C.x());
	pNb[2] = Eigen::Vector3d(A.y() - C.y(), C.x() - A.x(), 0);
	pNc[0] = Eigen::Vector3d(0, B.z() - A.z(), A.y() - B.y());
	pNc[1] = Eigen::Vector3d(A.z() - B.z(), 0, B.x() - A.x());
	pNc[2] = Eigen::Vector3d(B.y() - A.y(), A.x() - B.x(), 0);
	
	// Nx'ƫ����������
	Eigen::Vector3d pN_a[3], pN_b[3], pN_c[3];
	for (int k = 0; k < 3; k++) {
		double temp[3] = { N.dot(pNa[k]), N.dot(pNb[k]), N.dot(pNc[k]) };
		pN_a[k] = (pNa[k] - N.cwiseProduct(Eigen::Vector3d(temp[0], temp[0], temp[0])) / (L*L)) * sig;
		pN_b[k] = (pNb[k] - N.cwiseProduct(Eigen::Vector3d(temp[1], temp[1], temp[1])) / (L*L)) * sig;
		pN_c[k] = (pNc[k] - N.cwiseProduct(Eigen::Vector3d(temp[2], temp[2], temp[2])) / (L*L)) * sig;
	}

	N_ *= sig; // ��N'��λ������

	// ��¼�ݶ�ֵ

	#pragma omp parallel for
	for (int k = 0; k < 3; k++) {
		#pragma omp critical 
		{
			prNor[verIdx.x()](k, 0) += fArea * ((N_ - D).dot(pN_a[k]));
			prNor[verIdx.y()](k, 0) += fArea * ((N_ - D).dot(pN_b[k]));
			prNor[verIdx.z()](k, 0) += fArea * ((N_ - D).dot(pN_c[k]));
		}
	}
}
#pragma endregion

#pragma region Ť�������ݶ��������㷽��
void PolycubeAlgo::distortionGrad(const TriMesh &triMesh, const shared_ptr<Face> &face, concurrent_unordered_map<int, Eigen::Vector3d> &prDst) {
	auto verIdx = face->getVertexIndex();
	auto fArea = face->area(); // ��ǰ������
	auto A = triMesh.getVertex(verIdx[0])->point(), B = triMesh.getVertex(verIdx[1])->point(), C = triMesh.getVertex(verIdx[2])->point(); // ��ǰ���ϵ�������
	auto AB = B - A, AC = C - A;
	Eigen::Array2d Auv, Buv, Cuv;
	Auv << 0, 0;
	Buv << AB.norm(), 0; 
	Cuv << AB.dot(AC) / AB.norm(), (AB.cross(AC)).norm() / AB.norm();
	Eigen::MatrixXd MA(4, 4), MA_inv(4, 4); // ����A��A�������
	MA << Auv(0) - Cuv(0), Auv(1) - Cuv(1), 0, 0,
		0, 0, Auv(0) - Cuv(0), Auv(1) - Cuv(1),
		Buv(0) - Cuv(0), Buv(1) - Cuv(1), 0, 0,
		0, 0, Buv(0) - Cuv(0), Buv(1) - Cuv(1);
	MA_inv = MA.inverse(); // ����A����
	Eigen::MatrixXd Wxy(4, 1), Wxz(4, 1), Wyz(4, 1);
	Eigen::MatrixXd Bxy(4, 1), Bxz(4, 1), Byz(4, 1);
	Bxy << A.x() - C.x(), A.y() - C.y(), B.x() - C.x(), B.y() - C.y();
	Bxz << A.x() - C.x(), A.z() - C.z(), B.x() - C.x(), B.z() - C.z();
	Byz << A.y() - C.y(), A.z() - C.z(), B.y() - C.y(), B.z() - C.z();
	// ����Wxy, Wxz, Wyz
	Wxy = MA_inv * Bxy; Wxz = MA_inv * Bxz; Wyz = MA_inv * Byz;
	
	// ��Wu*Wuƫ����Wv*Wvƫ��������ʱ����
	Eigen::MatrixXd tempUU(2, 9), tempVV(2, 9);
	tempUU << MA_inv(0, 0), MA_inv(0, 2), -MA_inv(0, 0)-MA_inv(0, 2), MA_inv(0, 1), MA_inv(0, 3), -MA_inv(0, 1)-MA_inv(0, 3), 0, 0, 0,
		MA_inv(1, 0), MA_inv(1, 2), -MA_inv(1, 0)-MA_inv(1, 2), MA_inv(1, 1), MA_inv(1, 3), -MA_inv(1, 1)-MA_inv(1, 3), 0, 0, 0;
	tempVV << MA_inv(2, 0), MA_inv(2, 2), -MA_inv(2, 0)-MA_inv(2, 2), MA_inv(2, 1), MA_inv(2, 3), -MA_inv(2, 1)-MA_inv(2, 3), 0, 0, 0,
		MA_inv(3, 0), MA_inv(3, 2), -MA_inv(3, 0)-MA_inv(3, 2), MA_inv(3, 1), MA_inv(3, 3), -MA_inv(3, 1)-MA_inv(3, 3), 0, 0, 0;

	// ����Wu*Wu������ƽ���ƫ��
	// ����WuuXY�Ķ���x1,x2,x3,y1,y2,y3,z1,z2,z3ƫ��
	auto pWuuXY = 2 * (Wxy.block<2, 1>(0, 0).transpose() * tempUU).array();
	// ����tempUU��4��5��6�к�7��8��9��, ����WuuXZ������ƫ��
	for (int i = 0; i < 3; i++) tempUU.col(i+3).swap(tempUU.col(i+6));
	auto pWuuXZ = 2 * (Wxz.block<2, 1>(0, 0).transpose() * tempUU).array();
	// ����tempUU��1��2��3�к�4��5��6��, ����WuuYZ������ƫ��
	for (int i = 0; i < 3; i++) tempUU.col(i).swap(tempUU.col(i+3));
	auto pWuuYZ = 2 * (Wyz.block<2, 1>(0, 0).transpose() * tempUU).array();

	// ����Wv*Wv������ƽ���ƫ��
	// ����WvvXY�Ķ���x1,x2,x3,y1,y2,y3,z1,z2,z3ƫ��
	auto pWvvXY = 2 * (Wxy.block<2, 1>(2, 0).transpose() * tempVV).array();
	// ����tempVV��4��5��6�к�7��8��9��, ����WvvXZ������ƫ��
	for (int i = 0; i < 3; i++) tempVV.col(i + 3).swap(tempVV.col(i + 6));
	auto pWvvXZ = 2 * (Wxz.block<2, 1>(2, 0).transpose() * tempVV).array();
	// ����tempVV��1��2��3�к�4��5��6��, ����WvvYZ������ƫ��
	for (int i = 0; i < 3; i++) tempVV.col(i).swap(tempVV.col(i+3));
	auto pWvvYZ = 2 * (Wyz.block<2, 1>(2, 0).transpose() * tempVV).array();

	// ����Wu*Wvƫ��������ʱ����, Wxy_,Wxz_,Wyz_���·�תԭ�е�Wxy,Wxz,Wyz
	Eigen::MatrixXd tempUV(9, 4), Wxy_(Wxy), Wxz_(Wxz), Wyz_(Wyz);
	for (int i = 0; i < 2; i++) {
		Wxy_.row(i).swap(Wxy_.row(i + 2));
		Wxz_.row(i).swap(Wxz_.row(i + 2));
		Wyz_.row(i).swap(Wyz_.row(i + 2));
	}
	tempUV << MA_inv(0, 0), MA_inv(1, 0), MA_inv(2, 0), MA_inv(3, 0),
		MA_inv(0, 2), MA_inv(1, 2), MA_inv(2, 2), MA_inv(3, 2),
		-MA_inv(0, 0)-MA_inv(0, 2), -MA_inv(1, 0)-MA_inv(1, 2), -MA_inv(2, 0)-MA_inv(2, 2), -MA_inv(3, 0)-MA_inv(3, 2),
		MA_inv(0, 1), MA_inv(1, 1), MA_inv(2, 1), MA_inv(3, 1),
		MA_inv(0, 3), MA_inv(1, 3), MA_inv(2, 3), MA_inv(3, 3),
		-MA_inv(0, 1)-MA_inv(0, 3), -MA_inv(1, 1)-MA_inv(1, 3), -MA_inv(2, 1)-MA_inv(2, 3), -MA_inv(3, 1)-MA_inv(3, 3),
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	
	// ����Wu*Wv������ƽ���ƫ��
	// ����WuvXY��ƫ��
	auto pWuvXY = (tempUV * Wxy_).transpose();
	// ����tempUV��4��5��6�к�7��8��9��, ����WuvXZ������ƫ��
	for (int i = 0; i < 3; i++) tempUV.row(i + 3).swap(tempUV.row(i + 6));
	auto pWuvXZ = (tempUV * Wxz_).transpose();
	// ����tempUV��1��2��3�к�4��5��6��, ����WuvXZ������ƫ��
	for (int i = 0; i < 3; i++) tempUV.row(i).swap(tempUV.row(i + 3));
	auto pWuvYZ = (tempUV * Wyz_).transpose();
	
	// �ݶȵĲ������
	Eigen::MatrixXd preMat(1, 9), partialDMat(9, 9), gradientInc(1, 9);
	preMat << Wxy.block<2, 1>(0, 0).squaredNorm() - 1, Wxy.block<2, 1>(2, 0).squaredNorm() - 1, 
		Wxz.block<2, 1>(0, 0).squaredNorm() - 1, Wxz.block<2, 1>(2, 0).squaredNorm() - 1, 
		Wyz.block<2, 1>(0, 0).squaredNorm() - 1, Wyz.block<2, 1>(2, 0).squaredNorm() - 1,
		Wxy.block<2, 1>(0, 0).transpose() * Wxy.block<2, 1>(2, 0), 
		Wxz.block<2, 1>(0, 0).transpose() * Wxz.block<2, 1>(2, 0),
		Wyz.block<2, 1>(0, 0).transpose() * Wyz.block<2, 1>(2, 0);
	partialDMat << pWuuXY, pWvvXY, pWuuXZ, pWvvXZ, pWuuYZ, pWvvYZ, pWuvXY, pWuvXZ, pWuvYZ;
	
	// ����ݶȵ���ֵ��Ϊ1*9�ľ��󣬷Ǳ��ǹ���x1,x2,x3,y1,y2,y3,z1,z2,z3�ĵ�������
	gradientInc = preMat * partialDMat;

	// ��¼�ݶ�ֵ
	#pragma omp parallel for
	for (int i = 0; i < 3; i++) {
		#pragma omp critical
		{
			prDst[verIdx.x()](i, 0) += fArea * gradientInc(0, i * 3);
			prDst[verIdx.y()](i, 0) += fArea * gradientInc(0, i * 3 + 1);
			prDst[verIdx.z()](i, 0) += fArea * gradientInc(0, i * 3 + 2);
		}
	}
}
#pragma endregion