#ifndef RAVENSTATE_H
#define RAVENSTATE_H



#include <fstream>
#include "PracticalSocket.h"
#include "../ITPteleoperation.h"
#include "CommonDS.h"
#include <vector>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "Raven_Struct.h"
#include <HDU/hduQuaternion.h>
#include <Eigen/Dense>

#define READ_FREQUENCY		750		// in Hz
#define FILE_FREQUENCY		50		// in Hz

using namespace std;


class RavenState
{
public:
	RavenState();
	~RavenState();
	
	void start(void);
	void suspend(void);

	pos_struct CurrentRavenPosition;

	bool is_ready(void);
	bool start_thread(void);

protected:
	
	void ReadRavenPosition();
	boost::thread ReadRavenPositionThread;
	
	bool read_enabled;
	bool read_run;
	
	
	// For timing
	typedef boost::chrono::duration<double, boost::micro> micro_duration_t;
	typedef boost::chrono::time_point<boost::chrono::high_resolution_clock> micro_time_point_t;
	micro_duration_t time_difference;
	micro_duration_t period;
	
	micro_time_point_t last_entry_time;
	
	bool allOK;

};


#endif // RAVENSTATE_H
