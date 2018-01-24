/*
* Author: tiny656
* Date: 2015-10-13
*/
#pragma once
#include "utility.h"

class TinyLogger {
public:
	enum Priority { DEBUG, INFO, WARNING, ERROR };
	static void write(Priority priority, const string &message, bool printConsole=false);
private:
	TinyLogger() {}
	TinyLogger(const TinyLogger &logger);
	TinyLogger& operator=(const TinyLogger &logger);
	
	ofstream fileStream; // 文件流
	Priority priority; // logger优先级状态
	static const string LOGFILE; // logger文件名称
	static const string PRIORITY_NAMES[]; // logger优先级
	static TinyLogger instance; // logger实例，单例模式
};

// logger宏 - 写入文件
#define LOGGER_WRITE(PRIORITY, MESSAGE) TinyLogger::write(PRIORITY, MESSAGE)
// logger宏 - 写入文件和控制台
#define LOGGER_WRITE_CONSOLE(PRIORITY, MESSAGE) TinyLogger::write(PRIORITY, MESSAGE, true)