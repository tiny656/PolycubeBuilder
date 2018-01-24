/*
* Author: tiny656
* Date: 2015-10-14
*/

#include "KeyboardHandler.h"
#include "TinyLogger.h"

bool KeyboardHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object* obj, osg::NodeVisitor* nv) {
	osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
	if (!view) return false;

	switch (ea.getKey()) {
	case osgGA::GUIEventAdapter::KEY_F1: {
		LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "开启填充显示模式");
		// 开启填充模式
		osg::Geode* geode = dynamic_cast<osg::Geode*>(view->getSceneData());
		if (!geode) return false;
		osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
		polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
		geode->getOrCreateStateSet()->setAttribute(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		return true;
	}
	case osgGA::GUIEventAdapter::KEY_F2: {
		LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "开启线框显示模式");
		// 开启线框模式
		osg::Geode* geode = dynamic_cast<osg::Geode*>(view->getSceneData());
		if (geode) {
			osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
			polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
			geode->getOrCreateStateSet()->setAttribute(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		}
		osg::Group* group = dynamic_cast<osg::Group*>(view->getSceneData());
		if (group) {
			osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
			polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
			group->getChild(0)->getOrCreateStateSet()->setAttribute(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		}
		return true;
	}
	case osgGA::GUIEventAdapter::KEY_F3: {
		LOGGER_WRITE_CONSOLE(TinyLogger::INFO, "开启点显示模式");
		// 开启点模式
		osg::Geode* geode = dynamic_cast<osg::Geode*>(view->getSceneData());
		if (!geode) return false;
		osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
		polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
		geode->getOrCreateStateSet()->setAttribute(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
		return true;
	}
	default: break;
	}
	return false;
}
