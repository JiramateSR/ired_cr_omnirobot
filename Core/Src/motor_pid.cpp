#include "motor_pid.h"

double calculatePID(double _sp, double _pv, PID_Data_Typedef *_pid_data, double _dt) {
	_pid_data->err = _sp - _pv;

	if (_pid_data->err > 1000.0)
		_pid_data->err = 1000.0;
	if (_pid_data->err < -1000.0)
		_pid_data->err = -1000.0;

	_pid_data->sum_err += _pid_data->err;

	double ret = (_pid_data->kp * _pid_data->err) + (_pid_data->ki * _pid_data->sum_err * _dt)
			+ (_pid_data->kd * ((_pid_data->err - _pid_data->err_l) / _dt));

	_pid_data->err_l = _pid_data->err;

	if (ret > 1000.0)
		ret = 1000.0;
	if (ret < -1000.0)
		ret = 1000.0;

	return ret;
}

void resetPIDData(PID_Data_Typedef *_pid_data) {
	_pid_data->err = 0.0;
	_pid_data->err_l = 0.0;
	_pid_data->sum_err = 0.0;
}

void setupPIDParameter(PID_Data_Typedef *_pid_data, double _kp, double _ki, double _kd) {
	_pid_data->kp = _kp;
	_pid_data->ki = _ki;
	_pid_data->kd = _kd;
}
