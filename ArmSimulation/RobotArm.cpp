#include <fstream>
#include "RobotArm.h"

//std::ofstream ofs( "test.txt" );

TargetPoint::TargetPoint(Vector3d p, double t)
{
	pos(0) = p(0);
	pos(1) = p(1);
	pos(2) = p(2);
	time = t;
}

RobotArm::RobotArm()
{
	jl = new Vector3d[4];
	pl = new Vector3d[4];
	l[0] = 0.05;
	l[1] = 0.05;
	l[2] = 0.15;
	l[3] = 0.15;
	lh = 0.01;
	lf = 0.02;
	m[0] = 0.1;
	m[1] = 0.1;
	m[2] = 0.1;
	m[3] = 0.1;
	mh = 0.1;
	mf = 0.1;
	wi = 0.01;
	wf = 0.005;
	hi = 0.01;
	hf = 0.005;
	rh = 0.01;
	jl[0](0) = 0;
	jl[0](1) = 0;
	jl[0](2) = 0;
	pl[0](0) = jl[0](0);
	pl[0](1) = jl[0](1);
	pl[0](2) = jl[0](2)+l[0]/2;
	jl[1](0) = pl[0](0);
	jl[1](1) = pl[0](1);
	jl[1](2) = pl[0](2)+l[0]/2;
	pl[1](0) = jl[1](0);
	pl[1](1) = jl[1](1);
	pl[1](2) = jl[1](2)+l[1]/2;
	jl[2](0) = pl[1](0);
	jl[2](1) = pl[1](1);
	jl[2](2) = pl[1](2)+l[1]/2;
	pl[2](0) = jl[2](0);
	pl[2](1) = jl[2](1);
	pl[2](2) = jl[2](2)+l[2]/2;
	jl[3](0) = pl[2](0);
	jl[3](1) = pl[2](1);
	jl[3](2) = pl[2](2)+l[2]/2;
	pl[3](0) = jl[3](0);
	pl[3](1) = jl[3](1)+l[3]/2;
	pl[3](2) = jl[3](2);
	jh(0) = pl[3](0);
	jh(1) = pl[3](1)+l[3]/2;
	jh(2) = pl[3](2);
	ph(0) = jh(0);
	//pyh = jyh+lh/2;
	ph(1) = jh(1);
	ph(2) = jh(2);
	jf(0) = ph(0);
	//jyf = pyh+lh/2;
	jf(1) = ph(1);
	jf(2) = ph(2);
	pf(0) = jf(0);
	pf(1) = jf(1);
	pf(2) = jf(2)-lf/2;
	hw = 0.02;

	setAngle(0, 0, 0, 0);


	dt = 0.01;
	endTime = -1;
	time = 0;
	/*targetPoint(0) = 0;
	targetPoint(1) = 0;
	targetPoint(2) = 0;

	startPoint(0) = 0;
	startPoint(1) = 0;
	startPoint(2) = 0;*/

	setOffset(0,0,0,0);

	Kp = 10;
	
	openGripper();

	maxSpeedCartesian = Vector3d(1000, 1000, 1000);
	maxSpeedJoint[0] = 1000;
	maxSpeedJoint[1] = 1000;
	maxSpeedJoint[2] = 1000;
	maxSpeedJoint[3] = 1000;

	softUpperLimitCartesian = Vector3d(1000, 1000, 1000);
	softLowerLimitCartesian = Vector3d(-1000, -1000, -1000);

	pauseFalg = false;
	stopFalg = false;

	//homePosition = Vector3d(0, 0, 0);
	


	softUpperLimitJoint[0] = PI;
	softUpperLimitJoint[1] = PI;
	softUpperLimitJoint[2] = PI*3/2;
	softUpperLimitJoint[3] = PI/2;

	softLowerLimitJoint[0] = -PI;
	softLowerLimitJoint[1] = -PI;
	softLowerLimitJoint[2] = -PI/2;
	softLowerLimitJoint[3] = -PI/2;

	serbo = true;

	manifactur = "";
	type = "";
	axisNum = 4;
	cmdCycle = 100;
	isGripper = false;
	

}

void RobotArm::goHomePosition()
{
	homePosition = calcKinematics();

	targetPoint = homePosition;
	

	startPoint = homePosition;
	
}


void RobotArm::setHomePosition(double *jp)
{
	for(int i=0;i < 4;i++)
	{
		homeTheta[i] = jp[i];
	}

	//homePosition = calcKinematics();
	//homePosition = calcKinematics();
}

void RobotArm::openGripper()
{
	gripperPos = 0;
}

void RobotArm::closeGripper()
{
	gripperPos = hw - hf;
}

void RobotArm::update(double st)
{
	if(pauseFalg || stopFalg || !serbo)
		return;
	if(endTime < time)
	{
		Vector3d pos = calcKinematics();

		double dPx = Kp*(targetPoint(0)-pos(0));
		double dPy = Kp*(targetPoint(1)-pos(1));
		double dPz = Kp*(targetPoint(2)-pos(2));

		Vector3d dthe = calcJointVel(Vector3d(dPx, dPy, dPz));

		updatePos(dthe(0), dthe(1), dthe(2), 0);

		

		setTargetPos();

		

		return;
	}
	else
	{
		dt = st;

		double dx = targetPoint(0)-startPoint[0];
		double dy = targetPoint(1)-startPoint[1];
		double dz = targetPoint(2)-startPoint[2];

		double ST = sqrt(dx*dx+dy*dy+dz*dz);

		double A = 2*PI*ST/(endTime*endTime);

		double s = A*endTime/(2*PI)*(time - endTime/(2*PI)*sin(2*PI/endTime*time));

		double Px = s*dx/ST + startPoint[0];
		double Py = s*dy/ST + startPoint[1];
		double Pz = s*dz/ST + startPoint[2];


		double ds = A*endTime/(2*PI)*(1 - cos(2*PI/endTime*time));

		Vector3d pos = calcKinematics();

		double dPx = ds*dx/ST + Kp*(Px-pos(0));
		double dPy = ds*dy/ST + Kp*(Py-pos(1));
		double dPz = ds*dz/ST + Kp*(Pz-pos(2));

		
		//ofs << ds << "\t" << s << std::endl;
		//std::cout << pos << std::endl;

		time += dt;

		Vector3d dthe = calcJointVel(Vector3d(dPx, dPy, dPz));

		updatePos(dthe(0), dthe(1), dthe(2), 0);
	}
	
}

void RobotArm::setOffset(double o1, double o2, double o3, double o4)
{
	offset[0] = o1;
	offset[1] = o2;
	offset[2] = o3;
	offset[2] = o4;

	setAngle(o1,o2,o3,o4);

	double hp[4] = {o1, o2, o3, o4};
	setHomePosition(hp);
	goHomePosition();
}

void RobotArm::addTargetPos(Vector3d p, double T)
{
	TargetPoint tp(p,T);
	targetPoints.push_back(tp);
}

void RobotArm::setTargetPos()
{
	
	if(targetPoints.size() == 0)
	{
		return;
	}
	
	//std::cout << "test" << std::endl;
	
	time = 0;
	targetPoint(0) = targetPoints[0].pos(0);
	targetPoint(1) = targetPoints[0].pos(1);
	targetPoint(2) = targetPoints[0].pos(2);


	Vector3d pos = calcKinematics();
	startPoint(0) = pos(0);
	startPoint(1) = pos(1);
	startPoint(2) = pos(2);

	if(endTime > 0)
	{
		endTime = targetPoints[0].time;
	}
	else
	{
		double dx = targetPoint(0)-startPoint[0];
		double dy = targetPoint(1)-startPoint[1];
		double dz = targetPoint(2)-startPoint[2];

		double ST = sqrt(dx*dx+dy*dy+dz*dz)*20;
		if(ST < 0.1)
			ST = 0.1;
		endTime = ST;
		std::cout << ST << std::endl;
	}

	targetPoints.erase(targetPoints.begin());

	

	return;
}

void RobotArm::setAngle(double t1, double t2, double t3, double t4)
{
	theta[0] = t1;
	theta[1] = t2;
	theta[2] = t3;
	theta[3] = t4;

	judgeSoftLimitJoint();
}

Vector3d RobotArm::calcKinematics()
{
	Vector3d A;
	double S1 = sin(theta[0]);
	double S2 = sin(theta[1]);
	double C1 = cos(theta[0]);
	double C2 = cos(theta[1]);
	double S23 = sin(theta[1]+theta[2]);
	double C23 = cos(theta[1]+theta[2]);

	A(0) = -S1*C2*l[2] - S1*S23*l[3];
	A(1) = C1*C2*l[2] + C1*S23*l[3];
	
	A(2) = l[0] + l[1] + S2*l[2] - C23*l[3];

	return A;
}

Matrix3d RobotArm::calcJacobian()
{

	double S1 = sin(theta[0]);
	double S2 = sin(theta[1]);
	double C1 = cos(theta[0]);
	double C2 = cos(theta[1]);
	double S23 = sin(theta[1]+theta[2]);
	double C23 = cos(theta[1]+theta[2]);


	Matrix3d A;
	A(0,0) = -C1*C2*l[2] - C1*S23*l[3];
	A(0,1) = S1*S2*l[2] - C1*C23*l[3];
	A(0,2) = -S1*C23*l[3];
	A(1,0) = -S1*C2*l[2] - S1*S23*l[3];
	A(1,1) = -C1*S2*l[2] + C1*C23*l[3];
	A(1,2) = C1*C23*l[3];
	A(2,0) = 0;
	A(2,1) = C2*l[2] + S23*l[3];
	A(2,2) = S23*l[3];

	return A;

}

Vector3d RobotArm::calcJointVel(Vector3d v)
{
	Matrix3d J = calcJacobian();
	Matrix3d Jinv = J.inverse();

	Vector3d vf(v(0), v(1), v(2));
	

	Vector3d A = Jinv * vf;

	//std::cout << Jinv << std::endl << std::endl;
	
	return A;

}

void RobotArm::judgeSoftLimitJoint()
{
	for(int i=0;i < 4;i++)
	{
		if(theta[i] > softUpperLimitJoint[i])
		{
			
			theta[i] = softUpperLimitJoint[i];
			stop();
		}
		else if(theta[i] < softLowerLimitJoint[i])
		{
			theta[i] = softLowerLimitJoint[i];
			stop();
		}
	}
}

void RobotArm::updatePos(double v1, double v2, double v3, double v4)
{

	/*std::cout << v1 << std::endl;
	std::cout << v2 << std::endl;
	std::cout << v3 << std::endl;*/

	theta[0] = theta[0] + v1*dt;
	theta[1] = theta[1] + v2*dt;
	theta[2] = theta[2] + v3*dt;
	theta[3] = theta[3] + v4*dt;

	judgeSoftLimitJoint();

	

	
}


void RobotArm::setBaseOffset(double *bo)
{
	baseOffset(0,0) = bo[0];
	baseOffset(1,0) = bo[1];
	baseOffset(2,0) = bo[2];
	baseOffset(3,0) = bo[3];
	baseOffset(0,1) = bo[4];
	baseOffset(1,1) = bo[5];
	baseOffset(2,1) = bo[6];
	baseOffset(3,1) = bo[7];
	baseOffset(0,2) = bo[8];
	baseOffset(1,2) = bo[9];
	baseOffset(2,2) = bo[10];
	baseOffset(3,2) = bo[11];
	baseOffset(0,3) = bo[12];
	baseOffset(1,3) = bo[13];
	baseOffset(2,3) = bo[14];
	baseOffset(3,3) = bo[15];
}

void RobotArm::setMaxSpeedCartesian(Vector3d msc)
{
	maxSpeedCartesian = msc;
}

void RobotArm::setMaxSpeedJoint(double *msj)
{
	maxSpeedJoint[0] = msj[0];
	maxSpeedJoint[1] = msj[1];
	maxSpeedJoint[2] = msj[2];
	maxSpeedJoint[3] = msj[3];
}

void RobotArm::setSoftLimitCartesian(Vector3d usl, Vector3d lsl)
{
	softUpperLimitCartesian = usl;
	softLowerLimitCartesian = lsl;
}

void RobotArm::pause()
{
	pauseFalg = true;
}

void RobotArm::resume()
{
	pauseFalg = false;
}

void RobotArm::stop()
{
	stopFalg = false;
}

void RobotArm::setSoftLimitJoint(double *usl, double *lsl)
{
	softUpperLimitJoint[0] = usl[0];
	softUpperLimitJoint[1] = usl[1];
	softUpperLimitJoint[2] = usl[2];
	softUpperLimitJoint[3] = usl[3];

	softLowerLimitJoint[0] = lsl[0];
	softLowerLimitJoint[1] = lsl[1];
	softLowerLimitJoint[2] = lsl[2];
	softLowerLimitJoint[3] = lsl[3];
}

void RobotArm::setSerbo(bool state)
{
	serbo = state;
}

void RobotArm::setHandJointPosition(double hjp)
{
	theta[3] = hjp;
}