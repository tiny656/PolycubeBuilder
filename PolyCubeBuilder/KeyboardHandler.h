/*
* Author: tiny656
* Date: 2015-10-14
*/

#pragma once

#include "utility.h"

class KeyboardHandler : public osgGA::GUIEventHandler {
public:
	virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Object*, osg::NodeVisitor*);
};