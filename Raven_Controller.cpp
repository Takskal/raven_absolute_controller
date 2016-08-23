#include "Raven_Controller.h"


tf_t ITP_to_L;
tf_t ITP_to_R;

bool send_once = false;

Raven_Controller::Raven_Controller(void) : ctrl_run(false), ctrl_enabled(true)
{
	
	path_index = 0;
	collaboration_path_index = 0;

	time_res = 2000;
	lin_step = 0;

	// From mapping.cpp
	ITP_to_L <<	0,0,-1,0,  -1,0,0,0,  0,1,0,0,	0,0,0,1;
	ITP_to_R <<	0,0,-1,0,	1,0,0,0,  0,-1,0,0,	0,0,0,1;

	delta_ctrl.pos[0][0] = 0;
	delta_ctrl.pos[0][1] = 0;
	delta_ctrl.pos[0][2] = 0;

	delta_ctrl.pos[1][0] = 0;
	delta_ctrl.pos[1][1] = 0;
	delta_ctrl.pos[1][2] = 0;

	delta_ctrl.qtr[0].w() = 1;
	delta_ctrl.qtr[0].x() = 0;
	delta_ctrl.qtr[0].y() = 0;
	delta_ctrl.qtr[0].z() = 0;

	delta_ctrl.qtr[1].w() = 1;
	delta_ctrl.qtr[1].x() = 0;
	delta_ctrl.qtr[1].y() = 0;
	delta_ctrl.qtr[1].z() = 0;

	tmp_position = delta_ctrl;

	// Time for quaternion interpolation
	t_interpolation[LEFT] = 0;
	t_interpolation[RIGHT] = 0;

	// force send 0 increment
	send_null = true;

	// Timing
	period = micro_duration_t( 1000000/LOOP_FREQUENCY );
	last_entry_time = boost::chrono::high_resolution_clock::now();

	//current_time = 0;


	
	current_ctrl_mode = OMNI_MODE;
	
	ctrl_run = false;

	run_demo_loop = false;

	allow_omni = false;

	right_pedal_down = false;

	refine_lin_interpolation = 1.0;


}


Raven_Controller::~Raven_Controller(void)
{
}

bool Raven_Controller::start_thread(void)
{
	// First start raven state thread
	RavenPosition.start_thread();

	// Then start controller thread
	try
	{
		control_thread = boost::thread(boost::bind(&Raven_Controller::control_loop, this));
	}
	catch(exception &e)
	{
		std::cout << "\n\nERROR WHILE CREATING RAVEN CONTROLLER THREAD\n\n";
		return false;
	}
	std::cout << "Raven controller thread created succesfully\n";

	// Enable read thread
	RavenPosition.start();

	return true;
}

void Raven_Controller::toggle_send_null(void)
{
	send_null = !send_null;
	//send_once = true;
}

void Raven_Controller::start(void)
{
	if(RavenPosition.is_ready())
	{

		tmp_position = RavenPosition.CurrentRavenPosition;

		target_position = raven_path[path_index];
		time_res = get_time_resolution(RavenPosition.CurrentRavenPosition, target_position);
		desired_pos = tmp_position;

		ctrl_run = true;
	}
	else
	{
		std::cout << "\n\nRAVEN NOT PUBLISHING ITS STATE !!!\n";
		std::cout << "RAVEN NOT PUBLISHING ITS STATE !!!\n\n";
		suspend();
	}
}



void Raven_Controller::suspend(void)
{
	ctrl_run = false;
}

void Raven_Controller::control_loop()
{

	static long unsigned int cpt_disp = 0;
	vec_t tmpleft;
	vec_t tmpright;
	int grasp[2] = {0,0};
	double tmp_grasp[2] = {0,0};
	hduQuaternion qIncr[2];
	qtr_t qtr[2];
	qtr_t debug_qtr;

	unsigned int servo = 0;
	hduMatrix xfs[2];

	double lin_time = 0;

	static unsigned int cpt_send = 0;

	bool update_ids_haptic = false;

	// Control loop
	while(ctrl_enabled)
	{
		if(ctrl_run)
		{
			// Timing
			time_difference = boost::chrono::high_resolution_clock::now() - last_entry_time;

			if(time_difference >= period)
			{
				// Update entry time
				last_entry_time = boost::chrono::high_resolution_clock::now();

				// Read current position
				current_position = RavenPosition.CurrentRavenPosition;


				switch(current_ctrl_mode)
				{
				case ABSOLUTE_CONTROL_MODE:
					// In this case we control the robot and we make sure it reaches the destination before going to the next point in the path
					// To do so we do a linear interpolation between each point 

					// Check if last point reached
					check_path();

					// Control part
					if( path_index < raven_path.size())
					{

						if(!raven_path[path_index].reached)
						{

							if(lin_step >= (time_res/*-1*/))
							{

								raven_path[path_index].reached = true;

								tmp_position = RavenPosition.CurrentRavenPosition;

								desired_pos = tmp_position;

							}
							
						}

						if(!raven_path[path_index].reached)
						{

							// First I check if the raven has moved before increasing time step
							if( ( lin_step < time_res ) )
							{
								if(!send_null)
								{
									lin_step++;
								}
							}

							// Time variable for linear interpolation
							lin_time = (double)lin_step/(double)time_res;




							// Position interpolation
							tmpright = pos_interpolation(tmp_position.pos[RIGHT], target_position.pos[RIGHT], lin_time, desired_pos.pos[RIGHT]);
							tmpleft = pos_interpolation(tmp_position.pos[LEFT], target_position.pos[LEFT], lin_time, desired_pos.pos[LEFT]);

							// grasper control
							tmp_grasp[LEFT] = target_position.grasp[LEFT] - current_position.grasp[LEFT];
							tmp_grasp[RIGHT] = target_position.grasp[RIGHT] - current_position.grasp[RIGHT];

							
							qtr[RIGHT] = qtr_interpolation(tmp_position.qtr[RIGHT], target_position.qtr[RIGHT], lin_time * 0.1,desired_pos.qtr[RIGHT]);
							qtr[LEFT] = qtr_interpolation(tmp_position.qtr[LEFT], target_position.qtr[LEFT], lin_time * 0.1,desired_pos.qtr[LEFT]);

							
							// Frame transformation
							transform(ITP_to_R, qtr[RIGHT], tmpright);
							transform(ITP_to_L, qtr[LEFT], tmpleft);


							// Data type change for UDP communication

							// POSITION
							double tmp_scale = 0.1;	// Unstable above .25
							pos_delta[RIGHT][0] = tmpright[0] * tmp_scale;		pos_delta[LEFT][0] = tmpleft[0] * tmp_scale;
							pos_delta[RIGHT][1] = tmpright[1] * tmp_scale;		pos_delta[LEFT][1] = tmpleft[1] * tmp_scale;
							pos_delta[RIGHT][2] = tmpright[2] * tmp_scale;		pos_delta[LEFT][2] = tmpleft[2] * tmp_scale;

							// GRASPER
							grasp[LEFT] = (int)(tmp_grasp[LEFT]*-20);
							grasp[RIGHT] = (int)(tmp_grasp[RIGHT]*-20);
							

							// ORIENTATION
							qIncr[RIGHT].s() =    qtr[RIGHT].w() ;	qIncr[LEFT].s() =    qtr[LEFT].w() ;	
							qIncr[RIGHT].v()[0] = qtr[RIGHT].x();	qIncr[LEFT].v()[0] = qtr[LEFT].x();
							qIncr[RIGHT].v()[1] = qtr[RIGHT].y();	qIncr[LEFT].v()[1] = qtr[LEFT].y();
							qIncr[RIGHT].v()[2] = qtr[RIGHT].z();	qIncr[LEFT].v()[2] = qtr[LEFT].z();


							if(send_null )
							{
								pos_delta[RIGHT][0] = 0.0; pos_delta[RIGHT][1] = 0.0; pos_delta[RIGHT][2] = 0.0;
								pos_delta[LEFT][0] = 0.0;  pos_delta[LEFT][1] = 0.0;  pos_delta[LEFT][2] = 0.0;
								qIncr[RIGHT][0] = 1.0;	qIncr[RIGHT][1] = 0.0;	qIncr[RIGHT][2] = 0.0;	qIncr[RIGHT][3] = 0.0;
								qIncr[LEFT][0] = 1.0;		qIncr[LEFT][1] = 0.0;		qIncr[LEFT][2] = 0.0;		qIncr[LEFT][3] = 0.0;
								grasp[RIGHT] = 0;	grasp[LEFT] = 0;
							}


							// Send data to raven
							if(comm.Check_Flag(BASIC_PROGRAM))
							{

								// Check if values are safe
								if(send_null || is_incr_safe(pos_delta,qIncr))
								{
									servo++;

									comm.Update_UDP_Data(pos_delta, qIncr, grasp, 1, servo, xfs );
									comm.Send_UDP();

									previous_position = tmp_position;

								}
								else
								{
									// Suspend controller
									std::cout << "Controller suspended" << std::endl;
									suspend();
								}
							}
						}
					}

					break;
				
				case OMNI_MODE:
					// IN this case we just sleep as the omni are controlling the raven

					boost::this_thread::sleep_for( boost::chrono::microseconds(1));
					break;

					
				}
			}
			else
			{
				// Sleep 
				//boost::this_thread::sleep_for( boost::chrono::nanoseconds(10));
				
			}
		}
		else
		{
			// Timing
			time_difference = boost::chrono::high_resolution_clock::now() - last_entry_time;

			if(time_difference >= period)
			{
				// Update entry time
				last_entry_time = boost::chrono::high_resolution_clock::now();
				current_position = RavenPosition.CurrentRavenPosition;
				
			}

			// Sleep 
			//boost::this_thread::sleep_for( boost::chrono::nanoseconds(10));
			
		}
	}
}


void Raven_Controller::check_path()
{
	// Point waiting in queue
	if( path_index < raven_path.size() )
	{
		static bool disp_size = true;

		// Last point reached
		if(raven_path[path_index].reached)
		{
			if(run_demo_loop)
			{
				// Reset point reached to allow looping
				raven_path[path_index].reached = false;
			}

			path_index++;
			//std::cout << "Path idx:" << path_index << std::endl;
			// Sleep 

			disp_size = true;

			// Update values to reach
			if( path_index < raven_path.size() )
			{
				//std::cout << "Going for new point" << std::endl;

				target_position = raven_path[path_index];
				time_res = get_time_resolution(RavenPosition.CurrentRavenPosition, target_position);

				lin_step = 0;

			}
			else
			{

				std::cout << "No more points in path!!!" << std::endl;
				if(!run_demo_loop)
				{
					send_null = true;
					set_omni_mode();
				}
			}
		}
		if(disp_size)
		{
			//std::cout << "Path size:" << raven_path.size() - path_index << std::endl;
			disp_size = false;
		}
	}
	else
	{
		if(run_demo_loop)
		{
			path_index = 0;
			std::cout << "Restarting loop...\n";
		}
	}
}



qtr_t Raven_Controller::qtr_interpolation(const qtr_t &current, const qtr_t &target, const double &time, qtr_t &desired)
{
	qtr_t qincr, qtmp;

	// ## With Spherical interpolation
	qtmp = current.slerp(time, target);

	qincr = qtmp * desired.inverse();

	// Update desired position
	desired = qtmp;

	return qincr;
}

template<typename data_type>
data_type Raven_Controller::pos_interpolation(const data_type &start, const data_type &goal, const double &time, data_type &desired)
{
	data_type vec, inc;

	// Linear interpolation
	vec = start * (1 - time) + goal * time;

	inc = vec - desired;

	// Update desired position
	desired = vec;

	return inc;
}


bool Raven_Controller::is_omni_mode(void)
{
	if(current_ctrl_mode == OMNI_MODE)
	{
		return true;
	}

	return false;

}





bool Raven_Controller::is_incr_safe(const hduVector3Dd pos[], const hduQuaternion qtr[])
{
	bool result = true;
	double dist_left = abs(pos[LEFT].magnitude());
	double dist_right = abs(pos[RIGHT].magnitude());
	double pos_threshold = POSITION_SAFETY_THRESHOLD;

	// First lets take a look at the position
	if(dist_left > pos_threshold)
	{
		std::cout << std::endl << "####################### Left position increment too large!!!" << std::endl;
		std::cout << "Value: " << pos[LEFT] << std::endl;
		std::cout << "Magnitude: " << dist_left << std::endl;
		std::cout << "Current pos: " << (vec_t)current_position.pos[LEFT] << std::endl; 
		std::cout << "Target pos: " << (vec_t)target_position.pos[LEFT] << std::endl;

		result = false;
	}
	if(dist_right > pos_threshold)
	{
		std::cout << std::endl << "####################### Right position increment too large!!!" << std::endl;
		std::cout << "Value: " << pos[RIGHT] << std::endl;
		std::cout << "Magnitude: " << dist_right << std::endl;
		std::cout << "Current pos: " << (vec_t)current_position.pos[RIGHT] << std::endl; 
		std::cout << "Target pos: " << (vec_t)target_position.pos[RIGHT] << std::endl;
		result = false;
	}

	// Second, lets take a look at the orientation
	double angle_threshold = (ORIENTATION_SAFETY_THRESHOLD * 2) * M_PI / 180.0;		// add 100%
	double angle_right = abs(acos(qtr[RIGHT][0]) * 2);
	double angle_left = abs(acos(qtr[LEFT][0]) * 2);

	if(qtr[LEFT][0] == 0)
	{
		std::cout << std::endl << "####################### NULL Left Quaternion issue!!!" << std::endl;
		result = false;
	}
	if(qtr[RIGHT][0] == 0)
	{
		std::cout << std::endl << "####################### NULL Right Quaternion issue!!!" << std::endl;
		result = false;
	}

	if(angle_left > angle_threshold)
	{
		std::cout << std::endl << "####################### Left orientation increment too large!!!" << std::endl;
		std::cout << "t = " << lin_step << "/" << time_res << std::endl;
		std::cout << "Value: " << angle_left * 180 / M_PI << " degrees" << std::endl;
		std::cout << "Current qtr: " << current_position.qtr[LEFT].w() << " " << current_position.qtr[LEFT].x() << " "  << current_position.qtr[LEFT].y() << " " << current_position.qtr[LEFT].z() << std::endl; 
		std::cout << "Target qtr: " << target_position.qtr[LEFT].w() << " " << target_position.qtr[LEFT].x() << " " << target_position.qtr[LEFT].y() << " " << target_position.qtr[LEFT].z() <<std::endl;
		std::cout << "Inc qtr: " << qtr[LEFT] << std::endl;
		result = false;
	}
	if(angle_right > angle_threshold)
	{
		std::cout << std::endl << "####################### Right orientation increment too large!!!" << std::endl;
		std::cout << "t = " << lin_step << "/" << time_res << std::endl;
		std::cout << "Value: " << angle_right * 180 / M_PI << " degrees" << std::endl;
		std::cout << "Current qtr: " << current_position.qtr[RIGHT].w() << " " << current_position.qtr[RIGHT].x() << " " << current_position.qtr[RIGHT].y() << " " << current_position.qtr[RIGHT].z() << std::endl; 
		std::cout << "Target qtr: " << target_position.qtr[RIGHT].w() << " " << target_position.qtr[RIGHT].x() << " " << target_position.qtr[RIGHT].y() << " " << target_position.qtr[RIGHT].z() <<std::endl;
		std::cout << "Inc qtr: " << qtr[RIGHT] << std::endl;
		result = false;
	}
	
	return result;
}


vec_t Raven_Controller::get_pos_inc( const vec_t &start,  const vec_t &goal)
{
	vec_t  inc;

	inc = goal - start;

	return inc;
}

qtr_t Raven_Controller::get_qtr_inc( const qtr_t &current,  const qtr_t &target)
{
	qtr_t qincr;

	qincr = target * current.inverse();

	return qincr;
}

double Raven_Controller::get_max_pos_inc(const pos_struct &current, const pos_struct &target)
{
	double max_dist = 0;

	vec_t vec_left = get_pos_inc(current.pos[LEFT], target.pos[LEFT]);
	vec_t vec_right = get_pos_inc(current.pos[RIGHT], target.pos[RIGHT]);

	double dist_left =  abs(vec_left.norm());
	double dist_right = abs(vec_right.norm());

	max_dist = (std::max)(dist_left,dist_right);

	return max_dist;

}

double Raven_Controller::get_max_qtr_inc(const pos_struct &current, const pos_struct &target)
{
	double max_angle = 0;

	double angle_left = abs(current.qtr[LEFT].angularDistance(target.qtr[LEFT]));

	double angle_right = abs(current.qtr[RIGHT].angularDistance(target.qtr[RIGHT]));

	max_angle = fmod((std::max)(angle_left,angle_right), M_PI/2);
	
	return (max_angle * 180.0 / M_PI );
}


unsigned int Raven_Controller::get_time_resolution(const pos_struct &current, const pos_struct &target)
{
	double max_pos = get_max_pos_inc(current, target);
	double max_qtr = get_max_qtr_inc(current, target); 
	double t_res = 500, pos_res, qtr_res;

	// Look at position resolution
	pos_res = max_pos / POSITION_SAFETY_THRESHOLD;

	// Look at quaternion resolution
	qtr_res = max_qtr / (ORIENTATION_SAFETY_THRESHOLD );

	t_res = ceil((std::max)(pos_res, qtr_res) + 1) * refine_lin_interpolation;


	return (unsigned int)(t_res);
}


void Raven_Controller::transform(const tf_t &mat, qtr_t &qtr, vec_t &Pos)
{
	tf_t tf, tf_out;
	rot_t rot;
	qtr_t qtr_out;

	// Initialization
	rot =  qtr.toRotationMatrix();
	tf << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1;

	tf.block<3,3>(0,0) = rot;
	tf.block<3,1>(0,3) = Pos;

	// transformation
	tf_out = mat.inverse() * tf * mat;
	

	// Update Pos Input
	Pos = tf_out.block<3,1>(0,3);

	rot = tf_out.block<3,3>(0,0);
	qtr_out = rot;


	// Update Qtr Input
	qtr = qtr_out;

}


void Raven_Controller::set_omni_mode(void)
{
	current_ctrl_mode = OMNI_MODE;
}

void Raven_Controller::set_mode(Ctrl_mode desired_mode)
{
	current_ctrl_mode = desired_mode;
	ctrl_run = true;
}

void Raven_Controller::set_absolute_ctrl_mode(void)
{
	path_index = 0;
	current_ctrl_mode = ABSOLUTE_CONTROL_MODE;
	
}



void Raven_Controller::write_to_demo_file(const bool &start)
{
	RavenPosition.write_to_demo_file_ctrl(start);
	path_index = 0;
}

void Raven_Controller::write_to_loop_file(const bool &start)
{
	RavenPosition.write_to_loop_file_ctrl(start);
	path_index = 0;
}

void Raven_Controller::play_demo_loop(const bool &start)
{
	period = micro_duration_t( 1000000/275 );
	run_demo_loop = start;

	send_null = false;
	current_ctrl_mode = ABSOLUTE_CONTROL_MODE;
	this->start();
	refine_lin_interpolation = 1.5;
}



bool Raven_Controller::is_reached(const pos_struct &current, const pos_struct &target)
{
	bool result = false;

	
	double angle_right = abs(current.qtr[RIGHT].angularDistance(target.qtr[RIGHT]));
	angle_right = angle_right * 180  / M_PI;

	double angle_left = abs(current.qtr[LEFT].angularDistance(target.qtr[LEFT]));
	angle_left = angle_left * 180  / M_PI;

	if((angle_left < 1) && (angle_right < 1))
	{
		result = true;
	}



	return result;
}

void Raven_Controller::set_send_null(const bool &val)
{
	send_null = val;
}




vec_t Raven_Controller::transform_from_raven_to_itp(const vec_t &position, const int &arm)
{
	// First step raven to itp
	tf_t tf, tf_out;
	vec_t pos;

	// Initialization
	tf << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1;

	tf.block<3,1>(0,3) = position;

	// transformation
	switch(arm)
	{
	case LEFT:
		tf_out = ITP_to_L.inverse() * tf ;
		break;

	case RIGHT:
		tf_out = ITP_to_R.inverse() * tf ;
		break;

	}

	// Update Pos Input
	pos = tf_out.block<3,1>(0,3);

	return pos;

}

sawq


void Raven_Controller::go_to(const pos_struct& point)
{
	raven_path.clear();
	pos_struct pt = point;
	pt.reached = true;
	raven_path.push_back(pt);
	pt.reached = false;

	for(int i = 0; i < 25; i++)
	{
		raven_path.push_back(pt);
	}

	path_index = 0;
	set_absolute_ctrl_mode();
	set_send_null(false);
	start();
}





