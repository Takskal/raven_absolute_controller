
// In omni_client

	Raven_Controller ctrl;

	// Start controller thread
	ctrl.start_thread();

	// Current and target position of the robot
	Raven_Struct startup_position, target;
		
	// retreive current position of the robot
	startup_position = ctrl.current_position;
		
	// assign target with current position
	target = startup_position;

	// Fill in target with your values
	target.pos = ...;
	// or
	target.pos[LEFT][0] = ...;	// x
	target.pos[LEFT][1] = ...;	// y
	target.pos[LEFT][2] = ...;	// z

	target.qtr[LEFT] = ...;
	// or
	target.qtr[RIGHT][0] = ...;
	target.qtr[RIGHT][1] = ...;
	target.qtr[RIGHT][2] = ...;
	target.qtr[RIGHT][3] = ...;
	

	

	ctrl.go_to(target);
	