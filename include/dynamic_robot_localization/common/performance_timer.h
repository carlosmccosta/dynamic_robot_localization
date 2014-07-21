#pragma once


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#ifdef _WIN32   // Windows system specific
	#include <windows.h>
#else           // Unix based system specific
	#include <sys/time.h>
#endif

#include <stdlib.h>
#include <string>
#include <sstream>

#include <dynamic_robot_localization/common/time_utils.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace dynamic_robot_localization {
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <PerformanceTimer>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
class PerformanceTimer {
	public:
		PerformanceTimer();
		virtual ~PerformanceTimer();

		void start();                             // start timer
		void stop();                              // stop the timer
		void reset();
		void restart();
		double getElapsedTimeInSec();             // get elapsed time in second
		double getElapsedTimeInMilliSec();        // get elapsed time in milli-second
		double getElapsedTimeInMicroSec();        // get elapsed time in micro-second
		std::string getElapsedTimeFormated();


	private:
		double elapsedTimeMicroSec;               // starting time in micro-second
		bool stopped;                             // stop flag

		#ifdef _WIN32
			LARGE_INTEGER frequencyWin;           // ticks per second
			LARGE_INTEGER startCountWin;
			LARGE_INTEGER endCountWin;
		#else
			timeval startCount;
			timeval endCount;
		#endif

		void calculateElapsedTimeMicroSec();
};
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </PerformanceTimer>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
} /* namespace dynamic_robot_localization */
