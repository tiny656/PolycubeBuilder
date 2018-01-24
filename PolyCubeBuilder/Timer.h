/*
* Author: tiny656
* Date: 2015-11-18
*/
#pragma once

#include "utility.h"
#include <chrono>
using namespace std::chrono;

class Timer {
public:
	Timer():m_begin(high_resolution_clock::now()) { }
	void reset();
	int64_t elapsed() const; //Ä¬ÈÏÊä³öºÁÃë
	int64_t elapsed_micro() const; //Î¢Ãë
	int64_t elapsed_nano() const; //ÄÉÃë
	int64_t elapsed_seconds() const; //Ãë
	int64_t elapsed_minutes() const; //·Ö
	int64_t elapsed_hours() const; //Ê±

private:
	time_point<high_resolution_clock> m_begin;
};
