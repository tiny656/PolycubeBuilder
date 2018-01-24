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
	
	ofstream fileStream; // �ļ���
	Priority priority; // logger���ȼ�״̬
	static const string LOGFILE; // logger�ļ�����
	static const string PRIORITY_NAMES[]; // logger���ȼ�
	static TinyLogger instance; // loggerʵ��������ģʽ
};

// logger�� - д���ļ�
#define LOGGER_WRITE(PRIORITY, MESSAGE) TinyLogger::write(PRIORITY, MESSAGE)
// logger�� - д���ļ��Ϳ���̨
#define LOGGER_WRITE_CONSOLE(PRIORITY, MESSAGE) TinyLogger::write(PRIORITY, MESSAGE, true)