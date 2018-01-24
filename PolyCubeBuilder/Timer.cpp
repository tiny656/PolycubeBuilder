/*
* Author: tiny656
* Date: 2015-11-18
*/
#include "Timer.h"

void Timer::reset() {
	this->m_begin = high_resolution_clock::now();
}

int64_t Timer::elapsed() const {
	return duration_cast<chrono::milliseconds>(high_resolution_clock::now() - m_begin).count();
}

int64_t Timer::elapsed_micro() const {
	return duration_cast<chrono::microseconds>(high_resolution_clock::now() - m_begin).count();
}

int64_t Timer::elapsed_nano() const {
	return duration_cast<chrono::nanoseconds>(high_resolution_clock::now() - m_begin).count();
}

int64_t Timer::elapsed_seconds() const {
	return duration_cast<chrono::seconds>(high_resolution_clock::now() - m_begin).count();
}

int64_t Timer::elapsed_minutes() const {
	return duration_cast<chrono::minutes>(high_resolution_clock::now() - m_begin).count();
}

int64_t Timer::elapsed_hours() const {
	return duration_cast<chrono::hours>(high_resolution_clock::now() - m_begin).count();
}