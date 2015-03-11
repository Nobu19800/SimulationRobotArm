#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

using namespace Eigen;
const double PI = 3.141592;


enum MoveType { Joint = 0, Point = 1, };

class TargetPos
{
public:
	TargetPos();
	double end_time;
	double time;
	MoveType type;
	Vector3d start_pos;
	Vector3d target_pos;
	double start_joint_pos[4];
	double target_joint_pos[4];
	double target_theta;
	double start_theta; 

	void setPoint(double t, Vector3d t_p, double the);
	void setJointPos(double t, double *t_p);

	void setStartPoint(Vector3d s_p, double the, double speed);
	void setStartJointPos(double *s_p, double speed);

	
};



class RobotArm
{
public:
	RobotArm();
	Vector3d calcKinematics();
	void setAngle(double t1, double t2, double t3, double t4);
	Matrix3d calcJacobian();
	Vector3d calcJointVel(Vector3d v);
	void updatePos(double v1, double v2, double v3, double v4);
	void setOffset(double o1, double o2, double o3, double o4);
	void setTargetPos();
	void update(double st);
	void openGripper();
	void closeGripper();
	void addTargetPos(Vector3d p, double the, double T);
	void addTargetJointPos(double *p, double T);


	double l[4],lh,lf;
	double m[4],mh,mf;
	double wi, wf;
	double hi, hf;
	double rh;
	Vector3d *jl;
	Vector3d jh;
	Vector3d jf;
	Vector3d *pl;
	Vector3d ph;
	Vector3d pf;
	double hw;

	double theta[4];
	double homeTheta[4];

	double dt;
	//double endTime;
	double time;
	//Vector3d targetPoint;
	//Vector3d startPoint;

	double offset[4];

	double Kp;
	double Kjp;

	double gripperPos;

	std::vector<TargetPos> targetPoints;
	TargetPos targetPoint;

	void setBaseOffset(double *bo);
	void setMaxSpeedCartesian(Vector3d msc);
	void setMaxSpeedJoint(double *msj);
	void setSoftLimitCartesian(Vector3d usl, Vector3d lsl);
	void pause();
	void resume();
	void stop();
	void setSoftLimitJoint(double *usl, double *lsl);
	void goHomePosition();
	void setHomePosition(double *jp);
	void setSerbo(bool state);
	void judgeSoftLimitJoint();
	


	double baseOffset[12];
	Vector3d maxSpeedCartesian;
	double maxSpeedJoint[4];
	Vector3d softUpperLimitCartesian;
	Vector3d softLowerLimitCartesian;
	bool pauseFalg;
	bool stopFalg;
	Vector3d homePosition;
	double softUpperLimitJoint[4];
	double softLowerLimitJoint[4];
	double jointOffset[4];
	double motorAngle[4];
	bool serbo;
	std::string manifactur;
	std::string type;
	int axisNum;
	int cmdCycle;
	bool isGripper;

	double speedPoint;
	double speedJointPos;

	void setHandJointPosition(double hjp);
	void setStartPos(double j1, double j2, double j3, double j4);
	void start();

	double* getMotorPosition();
	double calcVel(double target_theta, double start_theta, double end_time, double time, double angle);
};



#endif