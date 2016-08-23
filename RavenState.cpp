#include "RavenState.h"

extern std::string user_name;

RavenState::RavenState():read_run(false),read_enabled(true)
{
	allOK = false;

	// Timing
	period = micro_duration_t( 1000000/READ_FREQUENCY );
	file_period = micro_duration_t( 1000000/FILE_FREQUENCY );

	last_entry_time = boost::chrono::high_resolution_clock::now();
	last_file_write_time = boost::chrono::high_resolution_clock::now();
	

}


RavenState::~RavenState()
{
}

bool RavenState::start_thread(void)
{
	try
	{
		ReadRavenPositionThread = boost::thread(boost::bind(&RavenState::ReadRavenPosition, this));
	}
	catch(exception &e)
	{
		std::cout << "\n\nERROR WHILE CREATING RAVEN STATE THREAD\n\n";
		return false;
	}
	std::cout << "Raven state thread created succesfully\n";

	return true;
}



void RavenState::start(void)
{
	read_run = true;
}

void RavenState::suspend(void)
{
	read_run = false;
}

bool RavenState::is_ready(void)
{
	return allOK;
}

void RavenState::ReadRavenPosition()
{
	double ScaleRavenPosition = 10.0/10000.0;
	unsigned short echoServPort = atoi("36001");     // First arg:  local port
	UDPSocket sock(echoServPort);                
	string sourceAddress;              // Address of datagram source
	unsigned short sourcePort;         // Port of datagram source*/
	u_struct ReceivedData;

	std::cout << "Opening socket toward raven\n";

	int bytesRcvd = sock.recvFrom(&ReceivedData, sizeof(u_struct), sourceAddress, sourcePort);

	std::cout << "Start reading raven state\n";

	while (read_enabled)
	{
		if(read_run)
		{
			
			// Timing
			time_difference = boost::chrono::high_resolution_clock::now() - last_entry_time;

			if(time_difference >= period)
			{

				// Update entry time
				last_entry_time = boost::chrono::high_resolution_clock::now();

				bytesRcvd = sock.recvFrom(&ReceivedData, sizeof(u_struct), sourceAddress, sourcePort);

				
				CurrentRavenPosition.pos[RIGHT][0] = ReceivedData.delx[RIGHT] * ScaleRavenPosition;
				CurrentRavenPosition.pos[RIGHT][1] = ReceivedData.dely[RIGHT] * ScaleRavenPosition;
				CurrentRavenPosition.pos[RIGHT][2] = ReceivedData.delz[RIGHT] * ScaleRavenPosition;
				CurrentRavenPosition.pos[LEFT][0] =  ReceivedData.delx[LEFT] * ScaleRavenPosition;
				CurrentRavenPosition.pos[LEFT][1] = ReceivedData.dely[LEFT] * ScaleRavenPosition;
				CurrentRavenPosition.pos[LEFT][2] = ReceivedData.delz[LEFT] * ScaleRavenPosition;

				// Quaternion right
				CurrentRavenPosition.qtr[RIGHT].w() = ReceivedData.Qw[RIGHT];
				CurrentRavenPosition.qtr[RIGHT].x() = ReceivedData.Qx[RIGHT];
				CurrentRavenPosition.qtr[RIGHT].y() = ReceivedData.Qy[RIGHT];
				CurrentRavenPosition.qtr[RIGHT].z() = ReceivedData.Qz[RIGHT];


				// Quaternion left
				CurrentRavenPosition.qtr[LEFT].w() = ReceivedData.Qw[LEFT];
				CurrentRavenPosition.qtr[LEFT].x() = ReceivedData.Qx[LEFT];
				CurrentRavenPosition.qtr[LEFT].y() = ReceivedData.Qy[LEFT];
				CurrentRavenPosition.qtr[LEFT].z() = ReceivedData.Qz[LEFT];

				// Grasp
				CurrentRavenPosition.grasp[RIGHT] = (double)ReceivedData.grasp[RIGHT] / 1000.0;
				CurrentRavenPosition.grasp[LEFT] = (double)ReceivedData.grasp[LEFT] / 1000.0;

				if((ReceivedData.Qw[RIGHT] !=0) && (ReceivedData.Qw[LEFT] != 0))
				{
					allOK = true;
				}

				// Timing
				time_difference = boost::chrono::high_resolution_clock::now() - last_file_write_time;

				}
			else
			{
				//boost::this_thread::sleep_for( boost::chrono::microseconds(1));
			}
		}
		else
		{
			//boost::this_thread::sleep_for( boost::chrono::microseconds(1));
		}
	}
}

