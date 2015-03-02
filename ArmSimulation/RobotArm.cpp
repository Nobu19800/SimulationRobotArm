#include <fstream>
#include "RobotArm.h"

//std::ofstream ofs( "test.txt" );


RobotArm::RobotArm()
{
	l[0] = 0.05;
	l[1] = 0.05;
	l[2] = 0.1;
	l[3] = 0.1;
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
	jx[0] = 0;
	jy[0] = 0;
	jz[0] = 0;
	px[0] = jx[0];
	py[0] = jy[0];
	pz[0] = jz[0]+l[0]/2;
	jx[1] = px[0];
	jy[1] = py[0];
	jz[1] = pz[0]+l[0]/2;
	px[1] = jx[1];
	py[1] = jy[1];
	pz[1] = jz[1]+l[1]/2;
	jx[2] = px[1];
	jy[2] = py[1];
	jz[2] = pz[1]+l[1]/2;
	px[2] = jx[2];
	py[2] = jy[2];
	pz[2] = jz[2]+l[2]/2;
	jx[3] = px[2];
	jy[3] = py[2];
	jz[3] = pz[2]+l[2]/2;
	px[3] = jx[3];
	py[3] = jy[3]+l[3]/2;
	pz[3] = jz[3];
	jxh = px[3];
	jyh = py[3]+l[3]/2;
	jzh = pz[3];
	pxh = jxh;
	//pyh = jyh+lh/2;
	pyh = jyh;
	pzh = jzh;
	jxf = pxh;
	//jyf = pyh+lh/2;
	jyf = pyh;
	jzf = pzh;
	pxf = jxf;
	pyf = jyf;
	pzf = jzf-lf/2;
	hw = 0.02;

	setAngle(0, 0, 0);


	dt = 0.01;
	endTime = -1;
	time = 0;
	targetPoint[0] = 0;
	targetPoint[1] = 0;
	targetPoint[2] = 0;

	startPoint[0] = 0;
	startPoint[1] = 0;
	startPoint[2] = 0;

	setOffset(0,0,0);

	Kp = 10;
	
	OpenGripper();
}


void RobotArm::OpenGripper()
{
	gripperPos = 0;
}

void RobotArm::CloseGripper()
{
	gripperPos = hw - hf;
}

void RobotArm::update(double st)
{
	if(endTime < time)
	{
		return;
	}
	else
	{
		dt = st;

		double dx = targetPoint[0]-startPoint[0];
		double dy = targetPoint[1]-startPoint[1];
		double dz = targetPoint[2]-startPoint[2];

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

		Vector3d dthe = calcJointVel(dPx, dPy, dPz);

		updatePos(dthe(0), dthe(1), dthe(2));
	}
	
}

void RobotArm::setOffset(double ox, double oy, double oz)
{
	offset[0] = ox;
	offset[1] = oy;
	offset[2] = oz;

	setAngle(ox,oy,oz);
}

bool RobotArm::setTargetPos(double px, double py, double pz, double T)
{
	if(endTime > time)
	{
		return false;
	}
	//std::cout << "test" << std::endl;
	endTime = T;
	time = 0;
	targetPoint[0] = px;
	targetPoint[1] = py;
	targetPoint[2] = pz;


	Vector3d pos = calcKinematics();
	startPoint[0] = pos(0);
	startPoint[1] = pos(1);
	startPoint[2] = pos(2);

	return true;
}

void RobotArm::setAngle(double t1, double t2, double t3)
{
	theta[0] = t1;
	theta[1] = t2;
	theta[2] = t3;
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

Vector3d RobotArm::calcJointVel(double vx, double vy, double vz)
{
	Matrix3d J = calcJacobian();
	Matrix3d Jinv = J.inverse();

	Vector3d vf(vx, vy, vz);
	

	Vector3d A = Jinv * vf;

	//std::cout << Jinv << std::endl << std::endl;

	return A;

}

void RobotArm::updatePos(double v1, double v2, double v3)
{

	/*std::cout << v1 << std::endl;
	std::cout << v2 << std::endl;
	std::cout << v3 << std::endl;*/

	theta[0] = theta[0] + v1*dt;
	theta[1] = theta[1] + v2*dt;
	theta[2] = theta[2] + v3*dt;

	

	

	
}