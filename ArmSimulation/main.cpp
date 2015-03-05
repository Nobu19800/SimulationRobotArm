#define dDOUBLE

#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "RobotArm.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#define dsDrawTriangle    dsDrawTriangleD
#endif



dWorldID      world;       
dSpaceID      space;       
dGeomID       ground;       
dJointGroupID contactgroup; 
dsFunctions   fn;


typedef struct {
  dBodyID  body;
  dGeomID  geom;
  dJointID joint;
  int  dir;
  float  red, green, blue;
  dReal    m,r,x,y,z,lx,ly,lz, the, dthe, ddthe, axisx, axisy, axisz, Tthe, Tdthe, Tddthe, tau, jx,jy,jz; // 質量(weight)，半径(radius)，位置(positin:x,y,z)
} MyLink;










MyLink link0, link1, link2, link3, linkhs, linkh, linkf[2];

RobotArm *rb;


void makeParam()
{
	link0.m = rb->m[0];
	link0.lx = rb->hi;
	link0.ly = rb->wi;
	link0.lz = rb->l[0];
	link0.x = rb->pl[0](0);
	link0.y = rb->pl[0](1);
	link0.z = rb->pl[0](2);
	link0.jx = rb->jl[0](0);
	link0.jy = rb->jl[0](1);
	link0.jz = rb->jl[0](2);
	link0.axisx = 0;
	link0.axisy = 0;
	link0.axisz = 1;
	link0.red = 1.;
	link0.green = 1.;
	link0.blue = 0.;

	link1.m = rb->m[1];
	link1.lx = rb->hi;
	link1.ly = rb->wi;
	link1.lz = rb->l[1];
	link1.x = rb->pl[1](0);
	link1.y = rb->pl[1](1);
	link1.z = rb->pl[1](2);
	link1.jx = rb->jl[1](0);
	link1.jy = rb->jl[1](1);
	link1.jz = rb->jl[1](2);
	link1.axisx = 0;
	link1.axisy = 0;
	link1.axisz = 1;
	link1.red = 1.;
	link1.green = 1.;
	link1.blue = 0.;



	link2.m = rb->m[2];
	link2.lx = rb->hi;
	link2.ly = rb->wi;
	link2.lz = rb->l[2];
	link2.x = rb->pl[2](0);
	link2.y = rb->pl[2](1);
	link2.z = rb->pl[2](2);
	link2.jx = rb->jl[2](0);
	link2.jy = rb->jl[2](1);
	link2.jz = rb->jl[2](2);
	link2.axisx = 1;
	link2.axisy = 0;
	link2.axisz = 0;
	link2.red = 1.;
	link2.green = 1.;
	link2.blue = 0.;

	link3.m = rb->m[3];
	link3.lx = rb->hi;
	link3.ly = rb->l[3];
	link3.lz = rb->wi;
	link3.x = rb->pl[3](0);
	link3.y = rb->pl[3](1);
	link3.z = rb->pl[3](2);
	link3.jx = rb->jl[3](0);
	link3.jy = rb->jl[3](1);
	link3.jz = rb->jl[3](2);
	link3.axisx = 1;
	link3.axisy = 0;
	link3.axisz = 0;
	link3.red = 1.;
	link3.green = 1.;
	link3.blue = 0.;

	linkhs.m = rb->mh;
	linkhs.lx = rb->wi;
	linkhs.ly = rb->wi;
	linkhs.lz = rb->wi;
	linkhs.x = rb->jh(0);
	linkhs.y = rb->jh(1);
	linkhs.z = rb->jh(2);
	linkhs.jx = rb->jh(0);
	linkhs.jy = rb->jh(1);
	linkhs.jz = rb->jh(2);
	linkhs.axisx = 1;
	linkhs.axisy = 0;
	linkhs.axisz = 0;
	linkhs.red = 1.;
	linkhs.green = 1.;
	linkhs.blue = 0.;

	linkh.m = rb->mh;
	linkh.lx = rb->rh;
	linkh.ly = rb->wi;
	linkh.lz = rb->lh;
	linkh.x = rb->ph(0);
	linkh.y = rb->ph(1);
	linkh.z = rb->ph(2);
	linkh.jx = rb->jh(0);
	linkh.jy = rb->jh(1);
	linkh.jz = rb->jh(2);
	linkh.axisx = 0;
	linkh.axisy = 0;
	linkh.axisz = 1;
	linkh.red = 1.;
	linkh.green = 1.;
	linkh.blue = 0.;

	for(int i = 0;i < 2;i++)
	{
		linkf[i].m = rb->mh;
		linkf[i].lx = rb->hf;
		linkf[i].ly = rb->wf;
		linkf[i].lz = rb->lf;
		if(i == 0)
			linkf[i].x = rb->pf(0) + rb->hw/2;
		else
			linkf[i].x = rb->pf(0) - rb->hw/2;
		linkf[i].y = rb->pf(1);
		linkf[i].z = rb->pf(2);
		linkf[i].jx = rb->jf(0);
		linkf[i].jy = rb->jf(1);
		linkf[i].jz = rb->jf(2);
		linkf[i].axisx = 1;
		linkf[i].axisy = 0;
		linkf[i].axisz = 0;
		linkf[i].red = 1.;
		linkf[i].green = 1.;
		linkf[i].blue = 0.;
	}

}



void setBox(MyLink *body)
{
	dMass mass;
	body->body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass,body->m , body->lx, body->ly, body->lz);
	dBodySetMass(body->body,&mass);
	body->geom = dCreateBox(space,body->lx, body->ly, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

void setCylinder(MyLink *body)
{
	dMass mass;
	body->body  = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass,body->m , 2, body->lx,  body->lz);
	dBodySetMass(body->body,&mass);
	body->geom = dCreateCylinder(space,body->lx, body->lz);
	dGeomSetBody(body->geom, body->body);
	dBodySetPosition(body->body, body->x, body->y, body->z);
}

void setHinge(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateHinge(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetHingeAnchor(body1->joint, body1->jx, body1->jy, body1->jz);
	dJointSetHingeAxis(body1->joint, body1->axisx, body1->axisy,body1->axisz);
}

void setSlider(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateSlider(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	
	dJointSetSliderAxis(body1->joint, body1->axisx, body1->axisy,body1->axisz);
}

void setFixed(MyLink *body1, MyLink *body2)
{
	body1->joint = dJointCreateFixed(world, 0);
	dJointAttach(body1->joint, body2->body, body1->body);
	dJointSetFixed(body1->joint);
}

void  makeRobot()
{
	setBox(&link0);
	setBox(&link1);
	setBox(&link2);
	setBox(&link3);
	setBox(&linkhs);
	setCylinder(&linkh);
	/*dMatrix3 R;
	dRFromAxisAndAngle(R, 1, 0, 0, PI/2);
	dGeomSetRotation(linkh.geom,  R);*/

	setBox(&linkf[0]);
	setBox(&linkf[1]);

	link0.joint = dJointCreateFixed(world, 0);
	dJointAttach(link0.joint, link0.body, 0);
	dJointSetFixed(link0.joint);
	setHinge(&link1, &link0);
	setHinge(&link2, &link1);
	setHinge(&link3, &link2);
	setHinge(&linkhs, &link3);
	setHinge(&linkh, &linkhs);
	setSlider(&linkf[0], &linkh);
	setSlider(&linkf[1], &linkh);

}

void drawBox(MyLink *body)
{
	const double sides[3] = {body->lx, body->ly, body->lz};
	dsSetColorAlpha(body->red,body->green,body->blue,1.0);
	dsDrawBoxD(dBodyGetPosition(body->body),
						dBodyGetRotation(body->body),sides);
}

void drawCylinder(MyLink *body)
{

	
	dsSetColorAlpha(body->red,body->green,body->blue,1.0);
	dsDrawCylinderD(dBodyGetPosition(body->body),
						dBodyGetRotation(body->body),body->lz,body->lx);
}

void drawRobot()
{
	drawBox(&link0);
	drawBox(&link1);
	drawBox(&link2);
	drawBox(&link3);
	
	drawCylinder(&linkh);
	

	drawBox(&linkf[0]);
	drawBox(&linkf[1]);
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
	//return;
	
  dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
  if (b1 && b2 && dAreConnected(b1,b2)) return;
  if ((o1 != ground) && (o2 != ground)) return;
 
   static const int N = 20;
	dContact contact[N];
	int n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));

	

	if (n > 0) {
		for (int i=0; i<n; i++) {
			contact[i].surface.mode = dContactApprox1|dContactSoftERP|dContactSoftCFM|dContactSlip1|dContactSlip2;



			contact[i].surface.mu   = 0.5;
			contact[i].surface.slip1 = 0.001;
			contact[i].surface.slip2 = 0.001;
			contact[i].surface.soft_erp = 0.3;
			contact[i].surface.soft_cfm = 1e-4;
			dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
			dJointAttach(c,b1,b2);
		}
	}

  
		
}

void controlHinge(MyLink *body, dReal theta)
{
	dReal kp = 100;
	dReal tmp = dJointGetHingeAngle(body->joint);
	dReal diff = theta - tmp;
	dReal u = kp * diff;

	dJointSetHingeParam(body->joint,dParamVel, u);
	dJointSetHingeParam(body->joint,dParamFMax,20.);
}

void controlSlider(MyLink *body, dReal length)
{
	dReal kp = 10;
	dReal tmp = dJointGetSliderPosition(body->joint);
	dReal diff = length - tmp;
	dReal u = kp * diff;

	dJointSetSliderParam (body->joint,dParamVel, u);
	dJointSetSliderParam (body->joint,dParamFMax,20.);
}



void Pcontrol()
{
	/*dReal t1 = dJointGetHingeAngle(link1.joint);
	dReal t2 = dJointGetHingeAngle(link2.joint);
	dReal t3 = dJointGetHingeAngle(link3.joint);
	rb->setAngle(-t1, -t2+PI/2, -t3);
	Vector3d ans = rb->calcKinematics();
	const dReal *db = dBodyGetPosition(linkh.body);
	for(int i=0;i < 3;i++)
	{
		std::cout << i << ":\t" << ans(i) << std::endl;
		std::cout << i << ":\t" << db[i] << std::endl;
	}*/

	
	


	//Vector3d fpos = rb->calcKinematics();
	//Vector3d ans = rb->calcJointVel(1*(0.05-fpos(0)), 1*(0.05-fpos(1)), 1*(0.2-fpos(2)));
	//Vector3d ans = rb->calcJointVel(0.005, 0.005, 0.005);
	//std::cout << fpos << std::endl;
	//rb->updatePos(ans(0), ans(1), ans(2));
	static bool test = true;
	
	if(test)
		rb->addTargetPos(Vector3d(0.05, 0.05, 0.2), -1);
	test = false;
	rb->update(0.01);

	//rb->CloseGripper();

	dReal r2 = dJointGetHingeAngle (link2.joint);
	dReal r3 = dJointGetHingeAngle (link3.joint);

	controlHinge(&link1, -rb->theta[0]+rb->offset[0]);//
	controlHinge(&link2, -rb->theta[1]+rb->offset[1]);//
	controlHinge(&link3, -rb->theta[2]+rb->offset[2]);//
	controlHinge(&linkhs, -r2-r3);//
	controlHinge(&linkh, -rb->theta[3]);//
	controlSlider(&linkf[0], rb->gripperPos/2);//
	controlSlider(&linkf[1], -rb->gripperPos/2);//

	dReal t1 = dJointGetHingeAngleRate (link1.joint);
	dReal t2 = dJointGetHingeAngleRate (link2.joint);
	dReal t3 = dJointGetHingeAngleRate (link3.joint);

	Vector3d  vel(-t1, -t2, -t3);
	

	Vector3d A = rb->calcJacobian() * vel;

	
	//std::cout << A << std::endl << std::endl;

	//std::cout << ans << std::endl << std::endl;
	
	
}



	




void simLoop(int pause)
{
  if (!pause) {
    Pcontrol();                              // 歩行制御 (gait control)
    dSpaceCollide(space,0,&nearCallback); // 衝突検出 (collision detection)
    dWorldStep(world, 0.01);              // ステップ更新 (step a simulation)
    dJointGroupEmpty(contactgroup);       // 接触点グループを空 (empty jointgroup)
  }
  drawRobot();                            // ロボットの描画 (draw a robot)
}




void start()
{
  //float xyz[3] = {  .0f,  1.0f, 3.0f};  // 視点[m] (View point)
  float xyz[3] = {  0.5f,  -0.0f, 0.25f};  // 視点[m] (View point)
  float hpr[3] = {180.0f, -10.0f, 0.0f};  // 視線[°] (View direction)
  //float hpr[3] = {0.0f, -90.0f, 90.0f};  // 視線[°] (View direction)
  //float xyz[3] = {  5.0f,  -5.0f, 3.0f};  // 視点[m] (View point)
  //float hpr[3] = {180.0f, -10.0f, 0.0f};  // 視線[°] (View direction)
  dsSetViewpoint(xyz,hpr);                // 視点と視線の設定 (Set View point and direction)
  dsSetSphereQuality(3);
  dsSetCapsuleQuality(6);


  
  
}


void setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.path_to_textures = "drawstuff/textures";
}






int main(int argc, char *argv[])
{
  rb = new RobotArm();
  rb->setOffset(0, PI/2, 0, 0);
  //rb->setHomePosition();


  dInitODE();
  setDrawStuff();
  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground       = dCreatePlane(space,0,0,1,0);
  dWorldSetGravity(world, 0, 0, -9.8);
  dWorldSetCFM(world, 1e-6); // CFMの設定 (global CFM)
  dWorldSetERP(world, 1.0);  // ERPの設定 (global ERP)
  makeParam();
  makeRobot();

dsSimulationLoop(argc,argv,800,480,&fn);

  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();
}
