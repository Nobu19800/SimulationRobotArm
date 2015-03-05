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


class TargetPoint
{
public:
	TargetPoint(Vector3d p, double t);
	double time;
	Vector3d pos;
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
	void addTargetPos(Vector3d p, double T);


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
	double endTime;
	double time;
	Vector3d targetPoint;
	Vector3d startPoint;

	double offset[4];

	double Kp;

	double gripperPos;

	std::vector<TargetPoint> targetPoints;

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
	


	Matrix4d baseOffset;
	Vector3d maxSpeedCartesian;
	double maxSpeedJoint[4];
	Vector3d softUpperLimitCartesian;
	Vector3d softLowerLimitCartesian;
	bool pauseFalg;
	bool stopFalg;
	Vector3d homePosition;
	double softUpperLimitJoint[4];
	double softLowerLimitJoint[4];
	bool serbo;
	std::string manifactur;
	std::string type;
	int axisNum;
	int cmdCycle;
	bool isGripper;

	void setHandJointPosition(double hjp);

};



#endif