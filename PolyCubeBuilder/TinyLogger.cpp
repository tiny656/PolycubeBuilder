/*
* Author: tiny656
* Date: 2015-10-13
*/

#include "TinyLogger.h"

time_t t(time(0));
const string TinyLogger::LOGFILE = "log_" + to_string(t) + ".txt";
const string TinyLogger::PRIORITY_NAMES[] = { "DEBUG", "INFO", "WARNING", "ERROR" };
TinyLogger TinyLogger::instance;

void TinyLogger::write(Priority priority, const string &message, bool printConsole) {
	ostream& stream = instance.fileStream.is_open() ? instance.fileStream : (instance.fileStream.open("log\\"+LOGFILE), instance.fileStream);
	stream << PRIORITY_NAMES[priority] << ": " << message << endl;
	if (printConsole) cout << PRIORITY_NAMES[priority] << ": " << message << endl;
}




