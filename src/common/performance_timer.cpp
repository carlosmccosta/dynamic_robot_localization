#include <dynamic_robot_localization/common/performance_timer.h>

namespace dynamic_robot_localization {
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <PerformanceTimer>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
PerformanceTimer::PerformanceTimer() {
	reset();
}


PerformanceTimer::~PerformanceTimer() {}


void PerformanceTimer::start() {
	stopped = 0;
	#ifdef _WIN32
		QueryPerformanceCounter(&startCountWin);
	#else
		gettimeofday(&startCount, NULL);
	#endif
}


void PerformanceTimer::stop() {
	stopped = true;
	
	#ifdef _WIN32
		QueryPerformanceCounter(&endCountWin);
	#else
		gettimeofday(&endCount, NULL);
	#endif
	
	calculateElapsedTimeMicroSec();
}


void PerformanceTimer::reset() {
	#ifdef _WIN32
		QueryPerformanceFrequency(&frequencyWin);
		startCountWin.QuadPart = 0;
		endCountWin.QuadPart = 0;
	#else
		startCount.tv_sec = startCount.tv_usec = 0;
		endCount.tv_sec = endCount.tv_usec = 0;
	#endif
	
	stopped = false;
	elapsedTimeMicroSec = 0;
}

void PerformanceTimer::restart() {
	reset();
	start();
}


double PerformanceTimer::getElapsedTimeInSec() {
	return getElapsedTimeInMicroSec() / 1000000.0;
}


double PerformanceTimer::getElapsedTimeInMilliSec() {
	
	return getElapsedTimeInMicroSec() / 1000.0;
}


double PerformanceTimer::getElapsedTimeInMicroSec() {
	if (!stopped) {
		#ifdef _WIN32
				QueryPerformanceCounter(&endCountWin);
		#else
				gettimeofday(&endCount, NULL);
		#endif
		calculateElapsedTimeMicroSec();
	}
	
	return elapsedTimeMicroSec;
}


void PerformanceTimer::calculateElapsedTimeMicroSec() {
	#ifdef _WIN32
		elapsedTimeMicroSec = (endCountWin.QuadPart - startCountWin.QuadPart) * 1000000.0 / frequencyWin.QuadPart;
	#else
		elapsedTimeMicroSec = (double)((endCount.tv_sec - startCount.tv_sec) * 1000000 + (endCount.tv_usec - startCount.tv_usec));
	#endif
}


std::string PerformanceTimer::getElapsedTimeFormated() {
	return dynamic_robot_localization::TimeUtils::formatSecondsToDate(getElapsedTimeInSec());
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </PerformanceTimer>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
} /* namespace dynamic_robot_localization */

