#include <fstream>
#include "RobotArm.h"

//std::ofstream ofs( "test.txt" );

TargetPos::TargetPos()
{
	time = 0;
	
}

void TargetPos::setPoint(double t, Vector3d t_p,  double the)
{

	target_pos(0) = t_p(0);
	target_pos(1) = t_p(1);
	target_pos(2) = t_p(2);
	target_theta = the;
	type = Point;

	end_time = t;
}

void TargetPos::setStartPoint(Vector3d s_p, double the, double speed)
{
	start_pos(0) = s_p(0);
	start_pos(1) = s_p(1);
	start_pos(2) = s_p(2);

	start_theta = the;

	if(end_time <= 0)
	{
		double dx = target_pos(0)-start_pos[0];
		double dy = target_pos(1)-start_pos[1];
		double dz = target_pos(2)-start_pos[2];

		end_time = sqrt(dx*dx+dy*dy+dz*dz)*speed;

		double dt = target_theta-start_theta;
		dt = sqrt(dt*dt)*speed/10.;
		if(end_time < dt)
			end_time = dt;

		

		if(end_time < 0.1)
			end_time = 0.1;
		
		std::cout << end_time << std::endl;
	}
}


void TargetPos::setJointPos(double t, double *t_p)
{
	for(int i=0;i < 4;i++)
	{

		target_joint_pos[i] = t_p[i];
	}
	type = Joint;

	end_time = t;
}

void TargetPos::setStartJointPos(double *s_p, double speed)
{
	for(int i=0;i < 4;i++)
	{
		start_joint_pos[i] = s_p[i];

	}

	if(end_time <= 0)
	{
		
		double d = 0;
		for(int i=0;i < 4;i++)
		{
			double tmp = target_joint_pos[i]-start_joint_pos[i];
			tmp = tmp*tmp;
			if(d < tmp)
				d += tmp;
		}

		end_time = sqrt(d)*speed;

		if(end_time < 0.1)
			end_time = 0.1;
		
		std::cout << end_time << std::endl;
	}
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
	//endTime = -1;
	//time = 0;
	/*targetPoint(0) = 0;
	targetPoint(1) = 0;
	targetPoint(2) = 0;

	startPoint(0) = 0;
	startPoint(1) = 0;
	startPoint(2) = 0;*/

	setOffset(0,0,0,0);

	Kp = 10;
	Kjp = 10;
	
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
	


	softUpperLimitJoint[0] = PI*90/180;
	softUpperLimitJoint[1] = PI*105/180;
	softUpperLimitJoint[2] = PI*90/180;
	softUpperLimitJoint[3] = PI/2;

	softLowerLimitJoint[0] = -PI*90/180;
	softLowerLimitJoint[1] = -0.001;
	softLowerLimitJoint[2] = -0.001;
	softLowerLimitJoint[3] = -PI/2;

	serbo = true;

	manifactur = "";
	type = "";
	axisNum = 4;
	cmdCycle = 100;
	isGripper = false;

	speedPoint = 10;
	speedJointPos = 1;

	jointOffset[0] = PI*90/180;
	jointOffset[1] = PI*20/180;
	jointOffset[2] = PI*105/180;
	jointOffset[3] = PI/2;
	

}

void RobotArm::goHomePosition()
{
	//homePosition = calcKinematics();

	//targetPoint = homePosition;

	addTargetJointPos(homeTheta, -1);
	

	//startPoint = homePosition;
	
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
	//std::cout << targetPoint.end_time << "\t" << targetPoint.time << std::endl;
	if(targetPoint.end_time < targetPoint.time)
	{
		if(targetPoint.type == Point)
		{
			/*Vector3d pos = calcKinematics();

			double dPx = Kp*(targetPoint.target_pos(0)-pos(0));
			double dPy = Kp*(targetPoint.target_pos(1)-pos(1));
			double dPz = Kp*(targetPoint.target_pos(2)-pos(2));

			Vector3d dthe = calcJointVel(Vector3d(dPx, dPy, dPz));

			updatePos(dthe(0), dthe(1), dthe(2), 0);*/
			
		}
		else
		{
			for(int i=0;i < 4;i++)
			{
				theta[i] = targetPoint.target_joint_pos[i];
			}
			judgeSoftLimitJoint();
		}

		
		setTargetPos();

		

		return;
	}
	else
	{
		if(targetPoint.type == Point)
		{
			double td = calcVel(targetPoint.target_theta, targetPoint.start_theta, targetPoint.end_time, targetPoint.time, theta[3]);

			dt = st;

			double dx = targetPoint.target_pos(0)-targetPoint.start_pos(0);
			double dy = targetPoint.target_pos(1)-targetPoint.start_pos(1);
			double dz = targetPoint.target_pos(2)-targetPoint.start_pos(2);

			double ST = sqrt(dx*dx+dy*dy+dz*dz);
			if(ST < 0.001)
			{
				updatePos(0, 0, 0, td);
				targetPoint.time += dt;
				return;
			}

			double A = 2*PI*ST/(targetPoint.end_time*targetPoint.end_time);

			double s = A*targetPoint.end_time/(2*PI)*(targetPoint.time - targetPoint.end_time/(2*PI)*sin(2*PI/targetPoint.end_time*targetPoint.time));

			double Px = s*dx/ST + targetPoint.start_pos(0);
			double Py = s*dy/ST + targetPoint.start_pos(1);
			double Pz = s*dz/ST + targetPoint.start_pos(2);


			double ds = A*targetPoint.end_time/(2*PI)*(1 - cos(2*PI/targetPoint.end_time*targetPoint.time));

			Vector3d pos = calcKinematics();

			double dPx = ds*dx/ST + Kp*(Px-pos(0));
			double dPy = ds*dy/ST + Kp*(Py-pos(1));
			double dPz = ds*dz/ST + Kp*(Pz-pos(2));

		
			//ofs << ds << "\t" << s << std::endl;
			//std::cout << pos << std::endl;

			

			Vector3d dthe = calcJointVel(Vector3d(dPx, dPy, dPz));


			
			

			updatePos(dthe(0), dthe(1), dthe(2), td);
		}
		else
		{
			double dthe[4];
			
			for(int i=0;i < 4;i++)
			{
				
				dthe[i] = calcVel(targetPoint.target_joint_pos[i], targetPoint.start_joint_pos[i], targetPoint.end_time, targetPoint.time, theta[i]);


				
			}

			updatePos(dthe[0], dthe[1], dthe[2], dthe[3]);

			
			
		}
		targetPoint.time += dt;
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
	//goHomePosition();
}

void RobotArm::addTargetPos(Vector3d p, double the, double T)
{
	TargetPos tp;
	tp.setPoint(T, p, the);
	targetPoints.push_back(tp);
}

void RobotArm::addTargetJointPos(double *p, double T)
{
	TargetPos tp;
	tp.setJointPos(T, p);
	targetPoints.push_back(tp);
}

void RobotArm::setTargetPos()
{
	
	if(targetPoints.size() == 0)
	{
		return;
	}
	
	//std::cout << "test" << std::endl;
	
	/*time = 0;
	targetPoint(0) = targetPoints[0].pos(0);
	targetPoint(1) = targetPoints[0].pos(1);
	targetPoint(2) = targetPoints[0].pos(2);*/


	if(targetPoints[0].type == Point)
	{
		Vector3d pos = calcKinematics();
		/*startPoint(0) = pos(0);
		startPoint(1) = pos(1);
		startPoint(2) = pos(2);*/

		targetPoints[0].setStartPoint(pos,theta[3],speedPoint);
	}

	else if(targetPoints[0].type == Joint)
	{
		targetPoints[0].setStartJointPos(theta, speedJointPos);

		
	}

	targetPoint = targetPoints[0];

	

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
		double mpos = theta[i];
		if(i == 2)
			mpos = theta[2] + theta[1];

		
		
		if(mpos > softUpperLimitJoint[i])
		{
			
			if(i == 2)
				theta[2] = softUpperLimitJoint[i] - theta[1];
			else
				theta[i] = softUpperLimitJoint[i];
			
			stop();
			
		}
		else if(mpos < softLowerLimitJoint[i])
		{
			if(i == 2)
				theta[2] = softLowerLimitJoint[i] - theta[1];
			else
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
	for(int i=0;i < 12;i++)
	{
		baseOffset[i] = bo[i];
	}
	
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
	stopFalg = true;
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

void RobotArm::setStartPos(double j1, double j2, double j3, double j4)
{
	double hp[4] = {j1, j2,j3, j4};
	setAngle(j1, j2,j3, j4);
	setHomePosition(hp);
	homePosition = calcKinematics();
	targetPoint.setJointPos(1, hp);
	targetPoint.setStartJointPos(hp, speedJointPos);
	targetPoint.time = 1000;
	//targetPoint = homePosition;
	//startPoint = homePosition;

	if(targetPoints.size() > 0)
		targetPoints.erase(targetPoints.begin());
	//endTime = -1;

	start();
}

void RobotArm::start()
{
	stopFalg = false;
}

double RobotArm::calcVel(double target_theta, double start_theta, double end_time, double time, double angle)
{
	double d = target_theta - start_theta;
	double A = 2*PI*d/(end_time*end_time);
	double s = A*end_time/(2*PI)*(time - end_time/(2*PI)*sin(2*PI/end_time*time));
	double ds = A*end_time/(2*PI)*(1 - cos(2*PI/end_time*time));
	double the = s + start_theta;
		
	
	return ds + Kjp*(the-angle);

	
}

double* RobotArm::getMotorPosition()
{
	
	motorAngle[0] = theta[0] + jointOffset[0];
	motorAngle[1] = theta[1] + jointOffset[1];
	motorAngle[2] = - theta[2] - theta[1] + jointOffset[2];
	motorAngle[3] = theta[3] + jointOffset[3];

	return motorAngle;
}