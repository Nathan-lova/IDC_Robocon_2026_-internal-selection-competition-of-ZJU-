#include "pid.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x)

PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;
extern int isMove;

static void pid_param_init(
	PID_TypeDef * pid,
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,
	float 	kp,
	float 	ki,
	float 	kd)
{
	pid->id = id;
	pid->ControlPeriod = period;
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->measure     = 0;
	pid->err         = 0;
	pid->last_err    = 0;
	pid->pout        = 0;
	pid->iout        = 0;
	pid->dout        = 0;
	pid->output      = 0;
	pid->last_output = 0;
	pid->lasttime    = HAL_GetTick();
	pid->thistime    = HAL_GetTick();
	pid->dtime       = 0;
}

static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->target      = 0.0f;
	pid->measure     = 0;
	pid->err         = 0;
	pid->last_err    = 0;
	pid->pout        = 0;
	pid->iout        = 0;
	pid->dout        = 0;
	pid->output      = 0;
	pid->last_output = 0;
	pid->lasttime    = HAL_GetTick();
	pid->thistime    = HAL_GetTick();
	pid->dtime       = 0;
}

static float pid_calculate(PID_TypeDef* pid, float measure)
{
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	pid->err = pid->target - pid->measure;

	if ((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		{
			float i_inc = pid->ki * pid->err;
			/* anti-windup: don't accumulate integral when output saturated */
			if (!((i_inc > 0 && pid->last_output >=  pid->MaxOutput) ||
			      (i_inc < 0 && pid->last_output <= -pid->MaxOutput)))
				pid->iout += i_inc;
		}
		pid->dout = pid->kd * (pid->err - pid->last_err);

		if (pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if (pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;

		pid->output = pid->pout + pid->iout + pid->dout;

		if (pid->output > pid->MaxOutput)
			pid->output = pid->MaxOutput;
		if (pid->output < -(pid->MaxOutput))
			pid->output = -(pid->MaxOutput);
	}
	else if (ABS(pid->target) < 1.0f)
	{
		/* joystick released + speed in deadband -> stop */
		pid->iout   = 0.0f;
		pid->output = 0.0f;
	}

	return pid->output;
}

void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
