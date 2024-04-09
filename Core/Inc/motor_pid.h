#ifndef MOTOR_PID_H__
#define MOTOR_PID_H__

typedef struct {
	double kp;
	double ki;
	double kd;
	double err;
	double err_l;
	double sum_err;
} PID_Data_Typedef;

void setupPIDParameter(PID_Data_Typedef *_pid_data, double _kp, double _ki, double _kd);
double calculatePID(double _sp, double _pv, PID_Data_Typedef *_pid_data, double _dt);
void resetPIDData(PID_Data_Typedef *_pid_data);

#endif
