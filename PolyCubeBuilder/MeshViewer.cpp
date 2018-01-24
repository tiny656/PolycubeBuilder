/*
* Author: tiny656
* Date: 2015-10-12
*/

#include "MeshViewer.h"
#include "TinyLogger.h"
#include "KeyboardHandler.h"
#include "GeometryModel.h"

osg::ref_ptr<osgViewer::CompositeViewer> MeshViewer::compositeViewer = new osgViewer::CompositeViewer;

void MeshViewer::show(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceColor, const string &windowName) {
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(internalFaceColorBuildDrawable(triMesh, faceColor));
	geode->addDrawable(internalLineDrawable(triMesh));
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> show --- 添加Drawble对象到geode");
	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 40;
		traits->y = 40;
		traits->width = 1200;
		traits->height = 960;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->windowName = windowName;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());

		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		// add this slave camera to the viewer, with a shift left of the projection matrix
		view->addSlave(camera.get());
	}
	view->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	view->setSceneData(geode);
	view->addEventHandler(new KeyboardHandler); // 添加键盘事件
	compositeViewer->addView(view);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> show --- 添加view到CompositeViewer");
}

void MeshViewer::showTriMeshPoint(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor, const string &windowName) {
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(internalFaceColorBuildDrawable(triMesh, concurrent_unordered_map<int, Eigen::Vector3d>()));
	geode->addDrawable(internalLineDrawable(triMesh)); // 画线
	geode->addDrawable(internalPointDrawable(triMesh, pointColor)); // 画点
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> show --- 添加Drawble对象到geode");
	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 40;
		traits->y = 40;
		traits->width = 1200;
		traits->height = 960;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->windowName = windowName;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());

		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		// add this slave camera to the viewer, with a shift left of the projection matrix
		view->addSlave(camera.get());
	}
	osg::ref_ptr<osg::Point> pt = new osg::Point(20.0);
	geode->getOrCreateStateSet()->setAttribute(pt, osg::StateAttribute::ON);

	view->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	view->setSceneData(geode);
	view->addEventHandler(new KeyboardHandler); // 添加键盘事件
	compositeViewer->addView(view);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> show --- 添加view到CompositeViewer");
}


void MeshViewer::show(const TetMesh &tetMesh, const string &windowName /*= ""*/) {
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(internalBuildDrawable(tetMesh));
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> show --- 添加Drawble对象到geode");
	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 40;
		traits->y = 40;
		traits->width = 1200;
		traits->height = 960;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->windowName = windowName;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		// add this slave camera to the viewer, with a shift left of the projection matrix
		view->addSlave(camera.get());
	}
	view->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	view->setSceneData(geode);
	view->addEventHandler(new KeyboardHandler); // 添加键盘事件
	compositeViewer->addView(view);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> show --- 添加view到CompositeViewer");
}

void MeshViewer::showTetMeshPoint(const TetMesh& tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor, const string& windowName) {
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(internalPointDrawable(tetMesh, pointColor)); // 画点
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> showTetMeshPoint --- 添加Drawble对象到geode");
	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 40;
		traits->y = 40;
		traits->width = 1200;
		traits->height = 960;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->windowName = windowName;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		// add this slave camera to the viewer, with a shift left of the projection matrix
		view->addSlave(camera.get());
	}
	view->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	view->setSceneData(geode);
	view->addEventHandler(new KeyboardHandler); // 添加键盘事件
	compositeViewer->addView(view);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> showTetMeshPoint --- 添加view到CompositeViewer");
}

void MeshViewer::showSimulationResult(const TetMesh& tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd>& pointSimulationData, const string& windowName) {
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(internalSimuationBuildDrawable(tetMesh, pointSimulationData));
	geode->addDrawable(internalSimuationLineBuildDrawable(tetMesh, pointSimulationData));
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> showSimulationResult --- 添加Drawble对象到geode");
	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 40;
		traits->y = 40;
		traits->width = 1200;
		traits->height = 960;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->windowName = windowName;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		// add this slave camera to the viewer, with a shift left of the projection matrix
		view->addSlave(camera.get());
	}
	osg::ref_ptr <osg::LineWidth> LineSize = new osg::LineWidth;
	LineSize->setWidth(3.0);
	geode->getOrCreateStateSet()->setAttributeAndModes(LineSize.get(), osg::StateAttribute::ON);

	view->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	view->setSceneData(geode);
	view->addEventHandler(new KeyboardHandler); // 添加键盘事件
	compositeViewer->addView(view);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> showSimulationResult --- 添加view到CompositeViewer");
}

void MeshViewer::showCompareTetMesh(const TetMesh& tetMesh1, const TetMesh& tetMesh2, 
	const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor1, 
	const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor2,
	GeometryModel &geometryModel,
	const string &windowName) {
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
	osg::ref_ptr<osg::Group> root = new osg::Group;
	osg::ref_ptr<osg::Geode> geode1 = new osg::Geode, geode2 = new osg::Geode;

	geode1->addDrawable(internalCompareTetMeshBuildDrawable(tetMesh1, tetMesh2, pointColor1, pointColor2));
	geode2->addDrawable(internalBoundryBuildDrawable(tetMesh1, geometryModel));
	root->addChild(geode1);
	root->addChild(geode2);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> showCompareTetMesh --- 添加Drawble对象到geode");
	{
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->x = 40;
		traits->y = 40;
		traits->width = 1200;
		traits->height = 960;
		traits->windowDecoration = true;
		traits->doubleBuffer = true;
		traits->sharedContext = 0;
		traits->windowName = windowName;
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);
		// add this slave camera to the viewer, with a shift left of the projection matrix
		view->addSlave(camera.get());
	}
	view->setLightingMode(osg::View::LightingMode::SKY_LIGHT);
	view->setSceneData(root);
	view->addEventHandler(new KeyboardHandler); // 添加键盘事件
	osg::ref_ptr <osg::LineWidth> LineSize = new osg::LineWidth;
	LineSize->setWidth(3.0);
	geode2->getOrCreateStateSet()->setAttributeAndModes(LineSize.get(), osg::StateAttribute::ON);
	compositeViewer->addView(view);
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> showCompareTetMesh --- 添加view到CompositeViewer");
}

void MeshViewer::run() {
	LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "MeshViewer -> run --- 显示所有的view");
	compositeViewer->run();
}

osg::Drawable* MeshViewer::internalFaceColorBuildDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &faceColor) {
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= triMesh.nFaces(); i++) {
		auto face = triMesh.getFace(i);
		auto fIdx = face->getVertexIndex();
		auto p1 = triMesh.getVertex(fIdx[0])->point();
		auto p2 = triMesh.getVertex(fIdx[1])->point();
		auto p3 = triMesh.getVertex(fIdx[2])->point();
		vertices->push_back(osg::Vec3(p1.x(), p1.y(), p1.z()));
		vertices->push_back(osg::Vec3(p2.x(), p2.y(), p2.z()));
		vertices->push_back(osg::Vec3(p3.x(), p3.y(), p3.z()));
		if (faceColor.find(face->id()) != faceColor.end()) {
			auto color = faceColor.at(face->id());
			colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
		} else {
			colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}


osg::Drawable* MeshViewer::internalPointColorBuildDrawable(const TriMesh &triMesh, const concurrent_unordered_map<int, Eigen::Vector3d> &pointColor) {
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= triMesh.nFaces(); i++) {
		auto face = triMesh.getFace(i);
		auto fIdx = face->getVertexIndex();
		auto p1 = triMesh.getVertex(fIdx[0])->point();
		auto p2 = triMesh.getVertex(fIdx[1])->point();
		auto p3 = triMesh.getVertex(fIdx[2])->point();
		vertices->push_back(osg::Vec3(p1.x(), p1.y(), p1.z()));
		vertices->push_back(osg::Vec3(p2.x(), p2.y(), p2.z()));
		vertices->push_back(osg::Vec3(p3.x(), p3.y(), p3.z()));
		if (pointColor.find(fIdx[0]) != pointColor.end()) {
			auto color = pointColor.at(fIdx[0]);
			colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
		}
		else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
		if (pointColor.find(fIdx[1]) != pointColor.end()) {
			auto color = pointColor.at(fIdx[1]);
			colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
		}
		else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
		if (pointColor.find(fIdx[2]) != pointColor.end()) {
			auto color = pointColor.at(fIdx[2]);
			colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
		}
		else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalBuildDrawable(const TetMesh &tetMesh) {
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= tetMesh.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement= tetMesh.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = { 
			tetMesh.getNode(tetNodeId.x())->point(),
			tetMesh.getNode(tetNodeId.y())->point(),
			tetMesh.getNode(tetNodeId.z())->point(),
			tetMesh.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			vertices->push_back(osg::Vec3(p[a].x(), p[a].y(), p[a].z()));
			vertices->push_back(osg::Vec3(p[b].x(), p[b].y(), p[b].z()));
			vertices->push_back(osg::Vec3(p[c].x(), p[c].y(), p[c].z()));
			if (!tetMesh.getNode(tetNodeId(a, 0))->onSurface()) colors->push_back(osg::Vec4(1.0, 0, 0, 1.0));
			else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			if (!tetMesh.getNode(tetNodeId(b, 0))->onSurface()) colors->push_back(osg::Vec4(1.0, 0, 0, 1.0));
			else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			if (!tetMesh.getNode(tetNodeId(c, 0))->onSurface()) colors->push_back(osg::Vec4(1.0, 0, 0, 1.0));
			else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalPointColorBuildDrawable(const TetMesh& tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor) {
	
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= tetMesh.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh.getNode(tetNodeId.x())->point(),
			tetMesh.getNode(tetNodeId.y())->point(),
			tetMesh.getNode(tetNodeId.z())->point(),
			tetMesh.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			vertices->push_back(osg::Vec3(p[a].x(), p[a].y(), p[a].z()));
			vertices->push_back(osg::Vec3(p[b].x(), p[b].y(), p[b].z()));
			vertices->push_back(osg::Vec3(p[c].x(), p[c].y(), p[c].z()));
			if (pointColor.find(tetNodeId[a]) != pointColor.end()) {
				auto color = pointColor.at(tetNodeId[a]);
				colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			} else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			if (pointColor.find(tetNodeId[b]) != pointColor.end()) {
				auto color = pointColor.at(tetNodeId[b]);
				colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			} else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			if (pointColor.find(tetNodeId[c]) != pointColor.end()) {
				auto color = pointColor.at(tetNodeId[c]);
				colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			} else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();


}

osg::Drawable* MeshViewer::internalSimuationBuildDrawable(const TetMesh& tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData) {
	osg::Vec4 colorSet[13] = {
		osg::Vec4(0.000000, 0.000000, 1.000000, 1.0),
		osg::Vec4(0.000000, 0.360784, 1.000000, 1.0),
		osg::Vec4(0.000000, 0.725490, 1.000000, 1.0),
		osg::Vec4(0.000000, 1.000000, 0.905882, 1.0),
		osg::Vec4(0.000000, 1.000000, 0.545098, 1.0),
		osg::Vec4(0.000000, 1.000000, 0.180392, 1.0),
		osg::Vec4(0.180392, 1.000000, 0.000000, 1.0),
		osg::Vec4(0.545098, 1.000000, 0.000000, 1.0),
		osg::Vec4(0.905882, 1.000000, 0.000000, 1.0),
		osg::Vec4(1.000000, 0.725490, 0.000000, 1.0),
		osg::Vec4(1.000000, 0.360784, 0.000000, 1.0),
		osg::Vec4(1.000000, 0.000000, 0.000000, 1.0),
		osg::Vec4(1.000000, 0.000000, 0.000000, 1.0)
	};
	double misesSet[13] = {
		-1, 0.3835, 0.5897, 0.7958, 1.002, 1.208, 1.414, 1.620, 1.827, 2.033, 2.239, 2.445, 100
	};
	double thermalSet[13] = {
		-1, 308.3, 316.7, 325, 333.3, 341.7, 350, 358.3, 366.7, 375, 383.3, 391.7, 10000
	};
	double dengziMisesSet[13] = {
		-1, 843.1, 1652, 2462, 3271, 4080, 4889, 5698, 6508, 7317, 8126, 8935, 100000
	};
	double dengziThermal[13] = {
		-1, 361.7, 378.4, 395.0, 411.7, 428.4, 445.1, 461.8, 478.4, 495.1, 511.8, 528.5, 100000
	};;

	auto misesIndexFunc = [&](double mises) {
		for (int i = 0; i < 13; i++) {
			if (mises < dengziThermal[i]) return i - 1;
		}
		return 0;
	};

	double ratio = 15 * 1e10;
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= tetMesh.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh.getNode(tetNodeId.x())->point(),
			tetMesh.getNode(tetNodeId.y())->point(),
			tetMesh.getNode(tetNodeId.z())->point(),
			tetMesh.getNode(tetNodeId.w())->point() };
		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			Eigen::VectorXd sim_data_a = pointSimulationData.at(tetNodeId(a, 0));
			Eigen::VectorXd sim_data_b = pointSimulationData.at(tetNodeId(b, 0));
			Eigen::VectorXd sim_data_c = pointSimulationData.at(tetNodeId(c, 0));
			vertices->push_back(osg::Vec3(p[a].x() + sim_data_a[1] * sim_data_a[4] * ratio, p[a].y() + sim_data_a[2] * sim_data_a[4] * ratio, p[a].z() + sim_data_a[3] * sim_data_a[4] * ratio));
			vertices->push_back(osg::Vec3(p[b].x() + sim_data_b[1] * sim_data_b[4] * ratio, p[b].y() + sim_data_b[2] * sim_data_b[4] * ratio, p[b].z() + sim_data_b[3] * sim_data_b[4] * ratio));
			vertices->push_back(osg::Vec3(p[c].x() + sim_data_c[1] * sim_data_c[4] * ratio, p[c].y() + sim_data_c[2] * sim_data_c[4] * ratio, p[c].z() + sim_data_c[3] * sim_data_c[4] * ratio));
			int index_a = misesIndexFunc(sim_data_a[0]);
			int index_b = misesIndexFunc(sim_data_b[0]);
			int index_c = misesIndexFunc(sim_data_c[0]);
			colors->push_back(colorSet[index_a]);
			colors->push_back(colorSet[index_b]);
			colors->push_back(colorSet[index_c]);
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalSimuationLineBuildDrawable(const TetMesh& tetMesh, const concurrent_unordered_map<int, Eigen::VectorXd> &pointSimulationData) {
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	double ratio = 15*1e10;
	for (int i = 1; i <= tetMesh.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh.getNode(tetNodeId.x())->point(),
			tetMesh.getNode(tetNodeId.y())->point(),
			tetMesh.getNode(tetNodeId.z())->point(),
			tetMesh.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			Eigen::VectorXd sim_data_a = pointSimulationData.at(tetNodeId(a, 0));
			Eigen::VectorXd sim_data_b = pointSimulationData.at(tetNodeId(b, 0));
			Eigen::VectorXd sim_data_c = pointSimulationData.at(tetNodeId(c, 0));
			vertices->push_back(osg::Vec3(p[a].x() + sim_data_a[1] * sim_data_a[4] * ratio, p[a].y() + sim_data_a[2] * sim_data_a[4] * ratio, p[a].z() + sim_data_a[3] * sim_data_a[4] * ratio));
			vertices->push_back(osg::Vec3(p[b].x() + sim_data_b[1] * sim_data_b[4] * ratio, p[b].y() + sim_data_b[2] * sim_data_b[4] * ratio, p[b].z() + sim_data_b[3] * sim_data_b[4] * ratio));
			vertices->push_back(osg::Vec3(p[c].x() + sim_data_c[1] * sim_data_c[4] * ratio, p[c].y() + sim_data_c[2] * sim_data_c[4] * ratio, p[c].z() + sim_data_c[3] * sim_data_c[4] * ratio));
			colors->push_back(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
			colors->push_back(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
			colors->push_back(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	for (int i = 0; i < vertices->size(); i += 3) {
		meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, i, 3));
	}
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();

}

osg::Drawable* MeshViewer::internalCompareTetMeshBuildDrawable(const TetMesh& tetMesh1, const TetMesh& tetMesh2, 
	const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor1, 
	const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor2) {
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= tetMesh1.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh1.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh1.getNode(tetNodeId.x())->point(),
			tetMesh1.getNode(tetNodeId.y())->point(),
			tetMesh1.getNode(tetNodeId.z())->point(),
			tetMesh1.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			vertices->push_back(osg::Vec3(p[a].x(), p[a].y(), p[a].z()));
			vertices->push_back(osg::Vec3(p[b].x(), p[b].y(), p[b].z()));
			vertices->push_back(osg::Vec3(p[c].x(), p[c].y(), p[c].z()));
			colors->push_back(osg::Vec4(0, 1, 0, 1.0));
			colors->push_back(osg::Vec4(0, 1, 0, 1.0));
			colors->push_back(osg::Vec4(0, 1, 0, 1.0));
		}
	}
	for (int i = 1; i <= tetMesh2.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh2.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh2.getNode(tetNodeId.x())->point(),
			tetMesh2.getNode(tetNodeId.y())->point(),
			tetMesh2.getNode(tetNodeId.z())->point(),
			tetMesh2.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			vertices->push_back(osg::Vec3(p[a].x(), p[a].y(), p[a].z()));
			vertices->push_back(osg::Vec3(p[b].x(), p[b].y(), p[b].z()));
			vertices->push_back(osg::Vec3(p[c].x(), p[c].y(), p[c].z()));
			colors->push_back(osg::Vec4(1, 0, 0, 1.0));
			colors->push_back(osg::Vec4(1, 0, 0, 1.0));
			colors->push_back(osg::Vec4(1, 0, 0, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalLineDrawable(const TriMesh& triMesh) {
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= triMesh.nFaces(); i++) {
		auto face = triMesh.getFace(i);
		auto fIdx = face->getVertexIndex();
		auto p1 = triMesh.getVertex(fIdx[0])->point();
		auto p2 = triMesh.getVertex(fIdx[1])->point();
		auto p3 = triMesh.getVertex(fIdx[2])->point();
		vertices->push_back(osg::Vec3(p1.x(), p1.y(), p1.z()));
		vertices->push_back(osg::Vec3(p2.x(), p2.y(), p2.z()));
		vertices->push_back(osg::Vec3(p3.x(), p3.y(), p3.z()));
		colors->push_back(osg::Vec4(0, 0, 0, 1.0));
		colors->push_back(osg::Vec4(0, 0, 0, 1.0));
		colors->push_back(osg::Vec4(0, 0, 0, 1.0));
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	for (int i = 0; i < vertices->size(); i += 3) {
		meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, i, 3));
	}
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalLineDrawable(const TetMesh& tetMesh) {
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	double ratio = 3e8;
	for (int i = 1; i <= tetMesh.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh.getNode(tetNodeId.x())->point(),
			tetMesh.getNode(tetNodeId.y())->point(),
			tetMesh.getNode(tetNodeId.z())->point(),
			tetMesh.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			vertices->push_back(osg::Vec3(p[a].x(), p[a].y(), p[a].z()));
			vertices->push_back(osg::Vec3(p[b].x(), p[b].y(), p[b].z()));
			vertices->push_back(osg::Vec3(p[c].x(), p[c].y(), p[c].z()));
			colors->push_back(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
			colors->push_back(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
			colors->push_back(osg::Vec4f(0.0, 0.0, 0.0, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	for (int i = 0; i < vertices->size(); i += 3) {
		meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, i, 3));
	}
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();

}

osg::Drawable* MeshViewer::internalPointDrawable(const TriMesh& triMesh, const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor) {
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;

	for (const auto &it : pointColor) {
		auto p = triMesh.getVertex(it.first)->point();
		vertices->push_back(osg::Vec3(p.x(), p.y(), p.z()));
		auto color = it.second;
		colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalPointDrawable(const TetMesh& tetMesh, const concurrent_unordered_map<int, Eigen::Vector3d>& pointColor) {
	auto mod4 = [](int i){return i % 4; };
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	for (int i = 1; i <= tetMesh.nTets(); i++) {
		shared_ptr<Tetrahedra> tetElement = tetMesh.getTetElement(i);
		Eigen::Vector4i tetNodeId = tetElement->getTetNodeId();
		Eigen::Vector3d p[4] = {
			tetMesh.getNode(tetNodeId.x())->point(),
			tetMesh.getNode(tetNodeId.y())->point(),
			tetMesh.getNode(tetNodeId.z())->point(),
			tetMesh.getNode(tetNodeId.w())->point() };

		for (int j = 0; j < 4; j++) {
			int a = j, b = mod4(j + 1), c = mod4(j + 2);
			vertices->push_back(osg::Vec3(p[a].x(), p[a].y(), p[a].z()));
			vertices->push_back(osg::Vec3(p[b].x(), p[b].y(), p[b].z()));
			vertices->push_back(osg::Vec3(p[c].x(), p[c].y(), p[c].z()));
			if (pointColor.find(tetNodeId[a]) != pointColor.end()) {
				auto color = pointColor.at(tetNodeId[a]);
				colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			}
			else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			if (pointColor.find(tetNodeId[b]) != pointColor.end()) {
				auto color = pointColor.at(tetNodeId[b]);
				colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			}
			else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
			if (pointColor.find(tetNodeId[c]) != pointColor.end()) {
				auto color = pointColor.at(tetNodeId[c]);
				colors->push_back(osg::Vec4(color[0], color[1], color[2], 1.0));
			}
			else colors->push_back(osg::Vec4(0.47, 0.47, 0.47, 1.0));
		}
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	for (int i = 0; i < vertices->size(); i++) {
		meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, i, 1));
	}
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}

osg::Drawable* MeshViewer::internalBoundryBuildDrawable(const TetMesh& tetMesh, const GeometryModel &geometryModel) {
	osg::ref_ptr<osg::Geometry> meshDrawable = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		
	for (int i = 1; i <= geometryModel.nBoundrys(); i++) {
		auto boundry = geometryModel.getBoundry(i);
		auto pId = boundry->endPointId();
		auto p1 = tetMesh.getNode(pId.first)->point();
		auto p2 = tetMesh.getNode(pId.second)->point();
		vertices->push_back(osg::Vec3(p1.x(), p1.y(), p1.z()));
		vertices->push_back(osg::Vec3(p2.x(), p2.y(), p2.z()));
		colors->push_back(osg::Vec4(1, 1, 1, 1.0));
		colors->push_back(osg::Vec4(1, 1, 1, 1.0));
	}
	meshDrawable->setVertexArray(vertices.get());
	meshDrawable->setColorArray(colors.get());
	meshDrawable->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	for (int i = 0; i < vertices->size(); i += 2) {
		meshDrawable->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, i, 2));
	}
	//osgUtil::SmoothingVisitor smv;
	//smv.smooth(*meshDrawable);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(meshDrawable.get());
	return meshDrawable.release();
}
