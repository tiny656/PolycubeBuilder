/*
* Author: tiny656
* Date: 2015-10-15
*/
#include "PolycubeAlgo.h"
#include "TinyLogger.h"
#include "MeshViewer.h"
#include "Timer.h"

void PolycubeAlgo::run(TriMesh &triMesh) {
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- 开始执行Polycube生成");
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- 模型共 " + to_string(triMesh.nFaces()) + " 个面, " + to_string(triMesh.nVertices()) + "个点");
	
	Timer timer; // 计时器
	concurrent_unordered_map<int, Eigen::Vector3d> color, faceDNormal;

	// 1. 根据法向对面片染色
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- 开始对模型进行染色");
	dye(triMesh, color, faceDNormal); // dye mesh Mesh染色, 返回 颜色信息 和 三角面片的标准法向
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- 染色完成, 共耗时" + to_string(timer.elapsed_seconds()) + "秒");
	
	#ifdef MESH_VIEW
	MeshViewer::show(triMesh, color, "染色");
	#endif // MESH_VIEW

	// 2. 能量优化生成伪Polycube
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- 开始对模型进行能量最优化");
	timer.reset();
	optimizeEnergy(triMesh, faceDNormal);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> run --- 能量最优化完成, 共耗时" + to_string(timer.elapsed_seconds()) + "秒");

	#ifdef MESH_VIEW
	MeshViewer::show(triMesh, color, "能量最优化");
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
	//TODO: 考虑是否可以更换其他优化模型, SGD, ADAGD, ADADELTA等
	#pragma region 梯度下降能量模型参数
	int max_iter_times = 100000;
	double learnRate = 0.005; // learn rate 学习速度a
	double wNor = 1, wDst = 0; // 法向能量和扭曲能量的权重。迭代的过程中法向能量的权重保持不变，扭曲能量的权重逐渐缩小直至0
	#pragma endregion

	const int NUMS_PRINT_ENERGY = 250; // 打印能量输出的频率
	const int BREAK_CONDITION = 250; // 判断是否跳出条件
	int iter_times = 1; // max iterator times 记录当前迭代次数
	double preEnergy = 1e16, curEnergy = computeEnergy(triMesh, faceDNormal, wNor, wDst); // 存储前一次的能量，进行比较看是否收敛
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> optimizeEnergy --- 初始输入模型的能量: " + to_string(curEnergy));
	while ((curEnergy < preEnergy || iter_times < max_iter_times) && curEnergy > 1e-5) {

		concurrent_unordered_map<int, Eigen::Vector3d> prNor, prDst; // 法向能量和扭曲能量的梯度迭代增量, 分别表示第id个点，在x,y,z上的增量
		for (auto v : triMesh.allVertexs()) {
			prNor[v.first].setZero();
			prDst[v.first].setZero();
		}
		
		#pragma omp parallel for
		for (int i = 1; i <= triMesh.nFaces(); i++) {
			auto face = triMesh.getFace(i);
			normalGrad(triMesh, face, faceDNormal, prNor); // 计算法向能量梯度更新值
			distortionGrad(triMesh, face, prDst); // 计算扭曲能量梯度更新值
		}

		// 梯度下降更新Mesh模型上的所有坐标值
		// TODO: 并行加速,unordered_map的并行需要使用bucket方式
		for (auto v : triMesh.allVertexs()) {
			auto vertex = v.second;
			for (int j = 0; j < 3; j++) 
				vertex->point()(j, 0) -= 2.0 * learnRate / vertex->getFaceId().size() * ((wNor * prNor[v.first](j, 0)) + (wDst * prDst[v.first](j, 0)));
		}

		#ifdef _DEBUG // 调试模式下，打印每一次的能量值
		LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "PolycubeAlgo -> optimizeEnergy --- 第 " + to_string(iter_times) + "次迭代");
		#endif

		preEnergy = curEnergy; //存储当前的能量
		curEnergy = computeEnergy(triMesh, faceDNormal, wNor, wDst);

		// wDst *= 0.95; // 调整扭曲能量的系数
		// if (iter_times >= 510) wDst = 0;
		// 当开始发散的时候，降低学习率
		if (curEnergy > preEnergy)  learnRate *= 0.80;
		
		// if (curEnergy < preEnergy) learnRate += learnRate * 0.1;
		#ifdef NDEBUG // 非调试模式下,到NUMS_PRINT_ENERGY
		if (iter_times <=  250 || iter_times % NUMS_PRINT_ENERGY == 0)
			LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo -> optimizeEnergy --- 第 " + to_string(iter_times) + "次迭代, " + "总能量值为: " + to_string(curEnergy));
		#endif

		// TODO: 添加迭代到达一定次数是否跳出
		// 如果迭代为1000,1500,2000,2500...时候，接受控制台输入是否要跳出循环
		// 是否动态调整学习率，每隔多少次
		if (iter_times % BREAK_CONDITION == 0) {
			string op = "";
			// cout << "是否需要跳出梯度下降 - 是:Y(y)";
			// cin >> op;
			if (op == "Y" || op == "y") break;
		}
		iter_times++; // 迭代次数+1
	}
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "PolycubeAlgo->optimizeEnergy --- 能量优化后最终的能量值为: " + to_string(curEnergy));

	// 更新每个面的法向,采用并行加速
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

#pragma region 模型总能量计算
double PolycubeAlgo::computeEnergy(TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, double wNor, double wDst) {
	double E = 0, normalEnergy = 0, distortionEnergy = 0; // 总能量,法向能量,扭曲能量
	#pragma omp parallel for
	for (int i = 1; i <= triMesh.nFaces(); i++) {
		auto face = triMesh.getFace(i);
		Eigen::Vector3i verIdx = face->getVertexIndex();
		auto fArea = face->area(); // 当前面的面积
		auto A = triMesh.getVertex(verIdx(0, 0))->point(), B = triMesh.getVertex(verIdx(1, 0))->point(), C = triMesh.getVertex(verIdx(2, 0))->point(); // 当前面上的三个点
		auto AB = B - A, AC = C - A;

		#pragma region 面片法向能量计算
		/* 面片法向能量 */
		Eigen::Vector3d D = faceDNormal.at(face->id());
		Eigen::Vector3d N = AB.cross(AC);
		auto N_ = Eigen::Vector3d(N);
		N_.normalize();
		// 计算法向符号
		int sig = N_.dot(D) < 0 ? -1 : 1;
		N_ *= sig;

		// 累加法向能量 
		// 并行公共数据区域
		#pragma omp critical
		{
			normalEnergy += fArea * (N_ - D).squaredNorm();
		}
		#pragma endregion

		#pragma region 面片扭曲能量计算
		/* 面片扭曲能量 */
		// 构造面片uv平面坐标
		Eigen::Array2d Auv, Buv, Cuv;
		Auv << 0, 0;
		Buv << AB.norm(), 0;
		Cuv << AB.dot(AC) / AB.norm(), (AB.cross(AC)).norm() / AB.norm();
		Eigen::MatrixXd MA(4, 4), MA_inv(4, 4); // 矩阵A
		MA << Auv(0) - Cuv(0), Auv(1) - Cuv(1), 0, 0,
			0, 0, Auv(0) - Cuv(0), Auv(1) - Cuv(1),
			Buv(0) - Cuv(0), Buv(1) - Cuv(1), 0, 0,
			0, 0, Buv(0) - Cuv(0), Buv(1) - Cuv(1);
		MA_inv = MA.inverse();// 矩阵A的逆
		Eigen::MatrixXd Wxy(4, 1), Wxz(4, 1), Wyz(4, 1);
		Eigen::MatrixXd Bxy(4, 1), Bxz(4, 1), Byz(4, 1);
		Bxy << A.x() - C.x(), A.y() - C.y(), B.x() - C.x(), B.y() - C.y();
		Bxz << A.x() - C.x(), A.z() - C.z(), B.x() - C.x(), B.z() - C.z();
		Byz << A.y() - C.y(), A.z() - C.z(), B.y() - C.y(), B.z() - C.z();
		// 计算Wxy, Wxz, Wyz
		Wxy = MA_inv * Bxy; Wxz = MA_inv * Bxz; Wyz = MA_inv * Byz;
		// 计算扭曲能量
		// 并行公共数据区域
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
	LOGGER_WRITE_CONSOLE(TinyLogger::DEBUG, "PolycubeAlgo -> computeEnergy --- 全局能量: " + to_string(E)
		+ ", 法向能量: " + to_string(wNor * normalEnergy)
		+ ", 扭曲能量: " + to_string(wDst * distortionEnergy));
	#endif

	return E;
}
#pragma endregion

#pragma region 法向能量梯度增量计算方法
void PolycubeAlgo::normalGrad(const TriMesh &triMesh, const shared_ptr<Face> &face, const concurrent_unordered_map<int, Eigen::Vector3d> &faceDNormal, concurrent_unordered_map<int, Eigen::Vector3d> &prNor) {
	Eigen::Vector3i verIdx = face->getVertexIndex();
	double fArea = face->area(); // 当前面的面积
	auto A = triMesh.getVertex(verIdx[0])->point(), B = triMesh.getVertex(verIdx[1])->point(), C = triMesh.getVertex(verIdx[2])->point(); // 当前面上的三个点
	auto AB = B - A, AC = C - A;
	Eigen::Vector3d N = AB.cross(AC); // N 向量
	auto L = N.norm(); // L 模
	Eigen::Vector3d N_ = N / L; // N' 向量是 N向量的partialDerivative单位向量
	Eigen::Vector3d D(faceDNormal.at(face->id())[0], faceDNormal.at(face->id())[1], faceDNormal.at(face->id())[2]); // D 向量，标准朝向向量

	// 计算法向符号
	int sig = N_.dot(D) < 0 ? -1 : 1;

	// Nx偏导3个点 pNa - pNAx,pNAy,pNAz | pNb - pNBx,pNBy,pNBz | pNc - pNCx,pNCy,pNCz
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
	
	// Nx'偏导的三个点
	Eigen::Vector3d pN_a[3], pN_b[3], pN_c[3];
	for (int k = 0; k < 3; k++) {
		double temp[3] = { N.dot(pNa[k]), N.dot(pNb[k]), N.dot(pNc[k]) };
		pN_a[k] = (pNa[k] - N.cwiseProduct(Eigen::Vector3d(temp[0], temp[0], temp[0])) / (L*L)) * sig;
		pN_b[k] = (pNb[k] - N.cwiseProduct(Eigen::Vector3d(temp[1], temp[1], temp[1])) / (L*L)) * sig;
		pN_c[k] = (pNc[k] - N.cwiseProduct(Eigen::Vector3d(temp[2], temp[2], temp[2])) / (L*L)) * sig;
	}

	N_ *= sig; // 对N'单位法向反向

	// 记录梯度值

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

#pragma region 扭曲能量梯度增量计算方法
void PolycubeAlgo::distortionGrad(const TriMesh &triMesh, const shared_ptr<Face> &face, concurrent_unordered_map<int, Eigen::Vector3d> &prDst) {
	auto verIdx = face->getVertexIndex();
	auto fArea = face->area(); // 当前面的面积
	auto A = triMesh.getVertex(verIdx[0])->point(), B = triMesh.getVertex(verIdx[1])->point(), C = triMesh.getVertex(verIdx[2])->point(); // 当前面上的三个点
	auto AB = B - A, AC = C - A;
	Eigen::Array2d Auv, Buv, Cuv;
	Auv << 0, 0;
	Buv << AB.norm(), 0; 
	Cuv << AB.dot(AC) / AB.norm(), (AB.cross(AC)).norm() / AB.norm();
	Eigen::MatrixXd MA(4, 4), MA_inv(4, 4); // 矩阵A和A的逆矩阵
	MA << Auv(0) - Cuv(0), Auv(1) - Cuv(1), 0, 0,
		0, 0, Auv(0) - Cuv(0), Auv(1) - Cuv(1),
		Buv(0) - Cuv(0), Buv(1) - Cuv(1), 0, 0,
		0, 0, Buv(0) - Cuv(0), Buv(1) - Cuv(1);
	MA_inv = MA.inverse(); // 矩阵A的逆
	Eigen::MatrixXd Wxy(4, 1), Wxz(4, 1), Wyz(4, 1);
	Eigen::MatrixXd Bxy(4, 1), Bxz(4, 1), Byz(4, 1);
	Bxy << A.x() - C.x(), A.y() - C.y(), B.x() - C.x(), B.y() - C.y();
	Bxz << A.x() - C.x(), A.z() - C.z(), B.x() - C.x(), B.z() - C.z();
	Byz << A.y() - C.y(), A.z() - C.z(), B.y() - C.y(), B.z() - C.z();
	// 计算Wxy, Wxz, Wyz
	Wxy = MA_inv * Bxy; Wxz = MA_inv * Bxz; Wyz = MA_inv * Byz;
	
	// 求Wu*Wu偏导和Wv*Wv偏导引入临时矩阵
	Eigen::MatrixXd tempUU(2, 9), tempVV(2, 9);
	tempUU << MA_inv(0, 0), MA_inv(0, 2), -MA_inv(0, 0)-MA_inv(0, 2), MA_inv(0, 1), MA_inv(0, 3), -MA_inv(0, 1)-MA_inv(0, 3), 0, 0, 0,
		MA_inv(1, 0), MA_inv(1, 2), -MA_inv(1, 0)-MA_inv(1, 2), MA_inv(1, 1), MA_inv(1, 3), -MA_inv(1, 1)-MA_inv(1, 3), 0, 0, 0;
	tempVV << MA_inv(2, 0), MA_inv(2, 2), -MA_inv(2, 0)-MA_inv(2, 2), MA_inv(2, 1), MA_inv(2, 3), -MA_inv(2, 1)-MA_inv(2, 3), 0, 0, 0,
		MA_inv(3, 0), MA_inv(3, 2), -MA_inv(3, 0)-MA_inv(3, 2), MA_inv(3, 1), MA_inv(3, 3), -MA_inv(3, 1)-MA_inv(3, 3), 0, 0, 0;

	// 计算Wu*Wu在所有平面的偏导
	// 计算WuuXY的对于x1,x2,x3,y1,y2,y3,z1,z2,z3偏导
	auto pWuuXY = 2 * (Wxy.block<2, 1>(0, 0).transpose() * tempUU).array();
	// 交换tempUU的4，5，6列和7，8，9列, 计算WuuXZ的所有偏导
	for (int i = 0; i < 3; i++) tempUU.col(i+3).swap(tempUU.col(i+6));
	auto pWuuXZ = 2 * (Wxz.block<2, 1>(0, 0).transpose() * tempUU).array();
	// 交换tempUU的1，2，3列和4，5，6列, 计算WuuYZ的所有偏导
	for (int i = 0; i < 3; i++) tempUU.col(i).swap(tempUU.col(i+3));
	auto pWuuYZ = 2 * (Wyz.block<2, 1>(0, 0).transpose() * tempUU).array();

	// 计算Wv*Wv在所有平面的偏导
	// 计算WvvXY的对于x1,x2,x3,y1,y2,y3,z1,z2,z3偏导
	auto pWvvXY = 2 * (Wxy.block<2, 1>(2, 0).transpose() * tempVV).array();
	// 交换tempVV的4，5，6列和7，8，9列, 计算WvvXZ的所有偏导
	for (int i = 0; i < 3; i++) tempVV.col(i + 3).swap(tempVV.col(i + 6));
	auto pWvvXZ = 2 * (Wxz.block<2, 1>(2, 0).transpose() * tempVV).array();
	// 交换tempVV的1，2，3列和4，5，6列, 计算WvvYZ的所有偏导
	for (int i = 0; i < 3; i++) tempVV.col(i).swap(tempVV.col(i+3));
	auto pWvvYZ = 2 * (Wyz.block<2, 1>(2, 0).transpose() * tempVV).array();

	// 计算Wu*Wv偏导引入临时矩阵, Wxy_,Wxz_,Wyz_上下翻转原有的Wxy,Wxz,Wyz
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
	
	// 计算Wu*Wv在所有平面的偏导
	// 计算WuvXY的偏导
	auto pWuvXY = (tempUV * Wxy_).transpose();
	// 交换tempUV的4，5，6行和7，8，9行, 计算WuvXZ的所有偏导
	for (int i = 0; i < 3; i++) tempUV.row(i + 3).swap(tempUV.row(i + 6));
	auto pWuvXZ = (tempUV * Wxz_).transpose();
	// 交换tempUV的1，2，3行和4，5，6行, 计算WuvXZ的所有偏导
	for (int i = 0; i < 3; i++) tempUV.row(i).swap(tempUV.row(i + 3));
	auto pWuvYZ = (tempUV * Wyz_).transpose();
	
	// 梯度的不变参数
	Eigen::MatrixXd preMat(1, 9), partialDMat(9, 9), gradientInc(1, 9);
	preMat << Wxy.block<2, 1>(0, 0).squaredNorm() - 1, Wxy.block<2, 1>(2, 0).squaredNorm() - 1, 
		Wxz.block<2, 1>(0, 0).squaredNorm() - 1, Wxz.block<2, 1>(2, 0).squaredNorm() - 1, 
		Wyz.block<2, 1>(0, 0).squaredNorm() - 1, Wyz.block<2, 1>(2, 0).squaredNorm() - 1,
		Wxy.block<2, 1>(0, 0).transpose() * Wxy.block<2, 1>(2, 0), 
		Wxz.block<2, 1>(0, 0).transpose() * Wxz.block<2, 1>(2, 0),
		Wyz.block<2, 1>(0, 0).transpose() * Wyz.block<2, 1>(2, 0);
	partialDMat << pWuuXY, pWvvXY, pWuuXZ, pWvvXZ, pWuuYZ, pWvvYZ, pWuvXY, pWuvXZ, pWuvYZ;
	
	// 求解梯度递增值，为1*9的矩阵，非别是关于x1,x2,x3,y1,y2,y3,z1,z2,z3的递增分量
	gradientInc = preMat * partialDMat;

	// 记录梯度值
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