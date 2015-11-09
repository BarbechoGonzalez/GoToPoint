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
	cango=false;
	lado=true;
	stgo=statego::ORIENTARSE;
	hayobt=true;
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
		
		if(state.state=="WORKING"){
			QMutexLocker ml(&m);
			posetag.x=target.x;
			posetag.y=target.y;
			posetag.z=target.z;
		}
		else{
			state.state="WORKING";
			st=State::WORKING;
			QMutexLocker ml(&m);
			posetag.x=target.x;
			posetag.y=target.y;
			posetag.z=target.z;
			objetivoactual.x=target.x;
			objetivoactual.y=target.y;
			objetivoactual.z=target.z;
		}
  
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
	
	writeinfo("poserobot:("+to_string(Basestate.z)+","+to_string(Basestate.x)+")\n"+"posetag:("+to_string(posetag.z)+","+to_string(posetag.x)+")\n"+"objetivoactual:("+to_string(objetivoactual.z)+","+to_string(objetivoactual.x)+")\n"+"subobjetivo:("+to_string(subobjetivo.z)+","+to_string(subobjetivo.x)+")");
	switch(stgo){
	  case statego::ORIENTARSE:
			qDebug()<<"ORIENTARSE";
			orientarse();
		break;
	  case statego::HAYOBTACULO:
			hayobt=hayobtaculo();
			qDebug()<<"HAYOBTACULO:"<<hayobt;
			if (!hayobt){
				if(cango)
					stgo=statego::AVANZAR;
				else
					stgo=statego::PUEDOPASAR;
			}
			else
			  stgo=statego::CALCULAROBJETIVO;
		break;
	  case statego::PUEDOPASAR:
			cango=puedopasar();
			qDebug()<<"PUEDOPASAR:"<<cango;
			if (cango)
				stgo=statego::AVANZAR;
			else
			  stgo=statego::CALCULAROBJETIVO;
		break;
	  case statego::AVANZAR:
			qDebug()<<"AVANZAR";
			avanzar();
		break;
	  case statego::CALCULAROBJETIVO:
			qDebug()<<"CALCULAROBJETIVO";
			calcularsubobjetivo();
		break;
	  case statego::HELLEGADO:
			qDebug()<<"HELLEGADO";
			hellegado();
		break;
	}
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
	stgo=statego::HAYOBTACULO;
}

bool SpecificWorker::hayobtaculo()
{
	QVec plano(2);
	plano(0)=Basestate.z;
	plano(1)=Basestate.x;
	QVec punto(2);
	punto(0)=objetivoactual.z;
	punto(1)=objetivoactual.x;
	QVec objerobot=cambiarinversoPlano(Basestate.alpha,punto,plano);
	float dist=sqrt((objerobot(0)*objerobot(0))+(objerobot(1)*objerobot(1)));
	if((ldata.data()+50)->dist<dist)
	  return true;
	else 
		return false;
}
QVec cambiarPlano(float alpha, QVec punto, QVec plano)
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
	QVec sol(3);
	sol=Rt*m;
	sol=sol/sol(2);
	return sol;
}

void SpecificWorker::calcularsubobjetivo()
{
	int i;
	int sentido;
	float anterior =(ldata.data()+50)->dist;
	float disfinal;
	float anglefinal;
	if(hayobt){
		if(lado){
			for(i=51;i<100;i++){
				if(((ldata.data()+i)->dist)-anterior>7){
					sentido=1;
					break;
				}
				anterior=(ldata.data()+i)->dist;
			}
		}
		else{
			for(i=49;i>0;i--){
				if(((ldata.data()+i)->dist)-anterior>7){
					sentido=-1;
					break;
				}
				anterior=(ldata.data()+i)->dist;
			}
		}
		float dist1=(ldata.data()+i-sentido)->dist;
		float dist2=200;
		disfinal=sqrt(dist1*dist1-dist2*dist2);
		float alpha=atan2(disfinal,dist2);
		disfinal=(ldata.data()+i)->dist/2;
		anglefinal=sentido*alpha+(ldata.data()+i)->angle;
	}
	else {
		int aux;
		if(lado){
			aux=49;
		}
		else{
			aux=51;
		}  
		anglefinal=(ldata.data()+aux)->angle;
		disfinal=(ldata.data()+aux)->dist/2;
	}
	float x=disfinal*sin(anglefinal);
	float z=disfinal*cos(anglefinal);
	QVec punto(2);
	QVec plano(2);
	punto(0)=z;
	punto(1)=x;
	plano(0)=Basestate.z;
	plano(1)=Basestate.x;
	QVec obj=cambiarPlano(Basestate.alpha,punto,plano);
	subobjetivo={obj(1),0,obj(0)};
	objetivoactual.x=subobjetivo.x;
	objetivoactual.y=subobjetivo.y;
	objetivoactual.z=subobjetivo.z;
	stgo=statego::ORIENTARSE;
	differentialrobot_proxy->setSpeedBase(0,0);
	cango=false;
}
bool SpecificWorker::puedopasar()
{	int i;
	float distobje=sqrt(objetivoactual.x*objetivoactual.x+objetivoactual.z*objetivoactual.z);
	if((ldata.data()+49)->dist<(ldata.data()+51)->dist){
		for(i=51;i<100;i++){
			float x=sin((ldata.data()+i)->angle)*(ldata.data()+i)->dist;
			float c=cos((ldata.data()+i)->angle)*(ldata.data()+i)->dist;
			x=abs(x);
			c=abs(c);
			qDebug()<<"-----------"<<x<<c<<(ldata.data()+i)->dist<<distobje<<i;
			if (x<210&&c<distobje){
				return false;
			}
		}
	}
	else{
		for(i=49;i>0;i--){
			float x=sin((ldata.data()+i)->angle)*(ldata.data()+i)->dist;
			float c=cos((ldata.data()+i)->angle)*(ldata.data()+i)->dist;
			x=abs(x);
			c=abs(c);
			qDebug()<<"-----------"<<x<<c<<(ldata.data()+i)->dist<<distobje<<i;
			if (x<210&&c<distobje){
				return false;
			}
		}
	}
	return true;
}
void SpecificWorker::hellegado()
{
	if(abs(Basestate.z-objetivoactual.z)<100&&abs(Basestate.x-objetivoactual.x)<100){
		m.lock();
		if (objetivoactual.x==posetag.x&&objetivoactual.z==posetag.z){
			state.state="FINISH";
			st=State::FINISH;
			if(lado)
				lado=false;
			else
				lado=true;
		}
		else{
			objetivoactual.x=posetag.x;
			objetivoactual.y=posetag.y;
			objetivoactual.z=posetag.z;
		}
		m.unlock();
		differentialrobot_proxy->setSpeedBase(0,0);
		stgo=statego::ORIENTARSE;
		cango=false;
	}
}
void SpecificWorker::avanzar()
{
	differentialrobot_proxy->setSpeedBase(200,0);
	stgo=statego::HELLEGADO;
}

void SpecificWorker::writeinfo(string _info)
{	
	texto->clear();
	QString *text=new QString(_info.c_str());
	texto->append(*text);
}  





