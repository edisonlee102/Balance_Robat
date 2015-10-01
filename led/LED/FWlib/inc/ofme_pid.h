#ifndef __OFME_PID_H__
#define __OFME_PID_H__


typedef struct PID
{
	float target;
	float integral;
	float Kp;	
	float Ki;	
	float Kd;	
}  pid_s, *pid_t;


void pid_init(pid_t pid, float Kp, float Ki, float Kd);
int pid_proc(pid_s *p, float current, float differential);


#endif
