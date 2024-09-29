#include <string.h>

#include "drummer.h"
#include "drummer.h"
#include "vex.h"

#define NB_ACC_SMOOTH 64

// Communication Variables
static double voltage = 0;
static double tmp_voltage = 0;

static double angle = 0;
static double tmp_angle = 0;

static double accx = 0;
static double off_accx = 0;
static double tmp_accx = 0;

static double accy = 0;
static double off_accy = 0;
static double tmp_accy = 0;

static double accz = 0;
static double off_accz = 0;
static double tmp_accz = 0;

static double acc_amp = 0;
static double tmp_acc_amp = 0;

using namespace vex;


void dyn_system_drummer(double *input, double *state, double *dstate, double *output)
{
	// TODO
}


void drummer_init()
{
	off_accx = 0;
	off_accy = 0;
	off_accz = 0;
	for (int i = 0; i < NB_ACC_SMOOTH; i++) {
		off_accx += CollisionSensor.acceleration(xaxis);
		off_accy += CollisionSensor.acceleration(yaxis);
		off_accz += CollisionSensor.acceleration(zaxis);
		wait(2.0, msec);
	}
	off_accx /= NB_ACC_SMOOTH;
	off_accy /= NB_ACC_SMOOTH;
	off_accz /= NB_ACC_SMOOTH;
}

void drummer_create_header_file()
{
	char text[10000];

	int nb_char = sprintf(text,
						  "Param :"
						  "\nSystem channels : %d"
						  "\nSensor channels : %d",
						  FREQ,
						  NB_CHANNEL_SYSTEM,
						  NB_CHANNEL_SENSORS);

	Brain.SDcard.savefile(HEADER_FILENAME,
						  (uint8_t *)text,
						  nb_char * sizeof(char));
}

double drummer_system_update()
{
	tmp_angle = (SpeedSensor.position(deg) * M_PI / 180.0);

	tmp_accx = CollisionSensor.acceleration(xaxis) - off_accx;
	tmp_accy = CollisionSensor.acceleration(yaxis) - off_accy;
	tmp_accz = CollisionSensor.acceleration(zaxis) - off_accz;
	tmp_acc_amp = sqrt(tmp_accx*tmp_accx + tmp_accy*tmp_accy + tmp_accz*tmp_accz);

	lock.lock();
	angle = tmp_angle;
	accx = tmp_accx;
	accy = tmp_accy;
	accz = tmp_accz;
	acc_amp = tmp_acc_amp;
	tmp_voltage = voltage;
	lock.unlock();

	return tmp_voltage;
}

void drummer_system_record(int count)
{
	double torque = PendulumMotor.torque(Nm);
	double voltage_mot = PendulumMotor.voltage(volt);
	double current = PendulumMotor.current(amp);
	double power = PendulumMotor.power(watt);
	double time = ((double)Brain.Timer.systemHighResolution()) * 1e-6;
	double speed = SpeedSensor.velocity(rpm) * M_PI / 30.0;

	int offset = NB_CHANNEL_SENSORS * count;

	//lock.lock();
	sensors_data[offset + 0] = time;
	sensors_data[offset + 1] = tmp_angle;
	sensors_data[offset + 2] = speed;
	sensors_data[offset + 3] = torque;
	sensors_data[offset + 4] = voltage_mot;
	sensors_data[offset + 5] = current;
	sensors_data[offset + 6] = power;
	sensors_data[offset + 7] = tmp_voltage;
	sensors_data[offset + 8] = tmp_acc_amp;
	//lock.unlock();
}

void drummer_setup(int *nb_state, int *nb_input, int *nb_output,
			   double **input, double **state, double **state2, double **dstate, double **dstate2, double **output) 
{
	// TODO
}

void drummer_sim_com(double *input, double *output) 
{
	lock.lock();

	input[0] = angle;
	input[1] = acc_amp;

	voltage = output[0];

	lock.unlock();
}

void drummer_sim_record(double *input, double *state, double *dstate, double *output, int count)
{
	int offset = NB_CHANNEL_SYSTEM * count;

	//lock.lock();
	system_data[offset] = ((double)Brain.Timer.systemHighResolution()) * 1e-6; // Time
	system_data[offset + 1] = output[0];									   // out 1
	system_data[offset + 2] = output[1];									   // out 2
	system_data[offset + 3] = input[0];										   // in 1
	system_data[offset + 4] = input[1];	
	//lock.unlock();
}
