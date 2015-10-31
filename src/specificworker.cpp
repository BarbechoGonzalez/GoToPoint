/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	state.state="IDLE";
	st=State::IDLE;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
	ldata=laser_proxy->getLaserData();
	differentialrobot_proxy->getBaseState(Basestate);
	switch(st){
	  case State::FINISH:
		break;
	  case State::WORKING:
			gototarget();
		break;
	  case State::IDLE:
		break;
	  case State::BLOCKED:
		break;
		
	}
	
}

float SpecificWorker::go(const TargetPose &target)
{
	state.state="WORKING";
	st=State::WORKING;
	QMutexLocker ml(&m);
	posetag=target;
	objetivoactual=target;
}

NavState SpecificWorker::getState()
{
	return state;
}

void SpecificWorker::stop()
{

}
void SpecificWorker::gototarget()
{
	if (!orientado)
		orientarse();
	if(!hayobtaculo()){
// 		if (puedopasar()){
// 			//avanzar
// 			if(hellegado()){
// 				st=State::FINISH;
// 				state.state="FINISH";
// 			}
// 		}
// 		else
// 			calcularsubobjetivo();
	}
	else
	  calcularsubobjetivo();
}
QVec cambiarinversoPlano(float alpha, QVec punto, QVec plano)
{
	QMat Rt(3,3);
	Rt(0,0)=cos(alpha);
	Rt(0,1)=-sin(alpha);
	Rt(0,2)=plano(0);
	Rt(1,0)=sin(alpha);
	Rt(1,1)=cos(alpha);
	Rt(1,2)=plano(1);
	Rt(2,0)=0;
	Rt(2,1)=0;
	Rt(2,2)=1;
	QVec m(3);
	m(0)=punto(0);
	m(1)=punto(1);
	m(2)=1;
	QMat Rtinver=Rt.invert();
	
	QVec TagR(3);
	TagR=Rtinver*m;
	TagR=TagR/TagR(2);
	return TagR;
}
void SpecificWorker::orientarse()
{
	differentialrobot_proxy->setSpeedBase(0,0);
	QVec plano(2);
	plano(0)=Basestate.z;
	plano(1)=Basestate.x;
	QVec punto(2);
	punto(0)=objetivoactual.z;
	punto(1)=objetivoactual.x;
	QVec objerobot=cambiarinversoPlano(Basestate.alpha,punto,plano);
	float rot=atan2(objerobot(1),objerobot(0));
	differentialrobot_proxy->setSpeedBase(0,rot);
	sleep(1);
	differentialrobot_proxy->setSpeedBase(0,0);
	orientado=true;
}

bool SpecificWorker::hayobtaculo()
{
	differentialrobot_proxy->setSpeedBase(0,0);
	QVec plano(2);
	plano(0)=Basestate.z;
	plano(1)=Basestate.x;
	QVec punto(2);
	punto(0)=objetivoactual.z;
	punto(1)=objetivoactual.x;
	QVec objerobot=cambiarinversoPlano(Basestate.alpha,punto,plano);
	float dist=sqrt((objerobot(0)*objerobot(0))+(objerobot(1)*objerobot(1)));
	if((ldata.data()+50)->dist<dist){
	  qDebug()<<"hay obtaculo";
	  return true;
	}
	else 
	  return false;
}

void SpecificWorker::calcularsubobjetivo()
{

}
bool SpecificWorker::puedopasar()
{

}
bool SpecificWorker::hellegado()
{

}






