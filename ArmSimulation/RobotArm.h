#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>

using namespace Eigen;
const double PI = 3.141592;



class RobotArm
{
public:
	RobotArm();
	Vector3d calcKinematics();
	void setAngle(double t1, double t2, double t3);
	Matrix3d calcJacobian();
	Vector3d calcJointVel(double vx, double vy, double vz);
	void updatePos(double v1, double v2, double v3);
	void setOffset(double ox, double oy, double oz);
	bool setTargetPos(double px, double py, double pz, double T);
	void update(double st);
	void OpenGripper();
	void CloseGripper();


	double l[4],lh,lf;
	double m[4],mh,mf;
	double wi, wf;
	double hi, hf;
	double rh;
	double jx[4], jy[4], jz[4];
	double jxh, jyh, jzh;
	double jxf, jyf, jzf;
	double px[4], py[4], pz[4];
	double pxh, pyh, pzh;
	double pxf, pyf, pzf;
	double hw;

	double theta[3];

	double dt;
	double endTime;
	double time;
	double targetPoint[3];
	double startPoint[3];

	double offset[3];

	double Kp;

	double gripperPos;
};
