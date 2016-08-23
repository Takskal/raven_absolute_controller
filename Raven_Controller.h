#ifndef RAVEN_CONTROLLER_H
#define RAVEN_CONTROLLER_H

#include <cmath>
#include <math.h>
#define _USE_MATH_DEFINES
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include "RavenState.h"
#include "PosStruct.h"
#include "omni_comm.h"

// UDP data types
#include <HDU/hduQuaternion.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduVector.h>
#include "Vision.h"



// #defines for control

// Safety and linear interpolation parameters
#define POSITION_SAFETY_THRESHOLD		3		// in mm
#define ORIENTATION_SAFETY_THRESHOLD	2.5		// in degrees

// Controller speed
#define LOOP_FREQUENCY	 125	// in Hz





extern Omni_Comm comm;


class Raven_Controller
{
public:
	enum Ctrl_mode
	{
		ABSOLUTE_CONTROL_MODE,
		OMNI_MODE,
		
	};

	// Functions
	Raven_Controller(void);
	~Raven_Controller(void);
	bool start_thread(void);
	void toggle_send_null(void);
	void set_send_null(const bool &val);
	void start(void);
	
	void suspend(void);
	bool is_omni_mode(void);
	
	void set_omni_mode(void);
	void set_mode(Ctrl_mode desired_mode);
	void set_absolute_ctrl_mode(void);
	
	void write_to_demo_file(const bool &start);
	void write_to_loop_file(const bool &start);
	void play_demo_loop(const bool &start);
	
	vec_t transform_from_raven_to_itp(const vec_t &position, const int &devID);

	bool is_grasp_reached(const double& grasp1, const double& grasp2);
	void go_to(const pos_struct& point);
	
	


protected:

	// Functions
	//double getPositionDirection(const double &delta, const double &stepSize);

	qtr_t qtr_interpolation(const qtr_t &current, const qtr_t &target, const double &time, qtr_t &desired);

	template<typename data_type>
	data_type pos_interpolation(const data_type &start, const data_type &goal, const double &time, data_type &desired);

	vec_t get_pos_inc( const vec_t &start,  const vec_t &goal);
	qtr_t get_qtr_inc( const qtr_t &current,  const qtr_t &target);

	double get_max_pos_inc(const pos_struct &current, const pos_struct &target);
	double get_max_qtr_inc(const pos_struct &current, const pos_struct &target);

	unsigned int get_time_resolution(const pos_struct &current, const pos_struct &target);

	void control_loop();
	void check_path();
	bool has_moved();
	bool is_incr_safe(const hduVector3Dd pos[], const hduQuaternion qtr[]);
	

	void transform(const tf_t &mat, qtr_t &qtr, vec_t &Pos);

	bool is_reached(const pos_struct &current, const pos_struct &target);

	double refine_lin_interpolation;
	


protected:
	// Variables
	double t_interpolation[2];
	hduVector3Dd pos_delta[2];
	boost::thread control_thread;
	bool ctrl_run;
	bool ctrl_enabled;
public:
	pos_struct current_position;
protected:
	pos_struct previous_position;
	pos_struct tmp_position;
	pos_struct target_position;
	pos_struct del_pos;
	pos_struct desired_pos;

	RavenState RavenPosition;

	typedef boost::chrono::duration<double, boost::micro> micro_duration_t;
	typedef boost::chrono::time_point<boost::chrono::high_resolution_clock> micro_time_point_t;
	micro_duration_t time_difference;
	micro_duration_t period;
	micro_time_point_t last_entry_time;

	bool send_null;

	// For linear interpolation
	unsigned int time_res;
	unsigned int lin_step;

	unsigned int path_index;
	

	bool run_demo_loop;
	

	bool right_pedal_down;

public:
	// Variables
	raven_path_t raven_path;
	pos_struct delta_ctrl;
	uint *id_haptic[2];
	
	unsigned int collaboration_path_index;

	Ctrl_mode current_ctrl_mode;

	Vision *vision_ptr;

	bool allow_omni;
};

#endif