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
#include <qt4/Qt/qdebug.h>

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
	inner = new InnerModel("/home/ivan/robocomp/files/innermodel/simpleworld.xml");
	repe=0;
	graphicsView->setScene(&scene);
	graphicsView->show();
	graphicsView->scale(3,3);
	TBaseState s;
// 	differentialrobot_proxy->getBaseState(s);
// 	differentialrobot_proxy->setOdometerPose(0,0,s.alpha);

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
	histogram();
	ldata=laser_proxy->getLaserData();
	differentialrobot_proxy->getBaseState(Basestate);
	inner->updateTransformValues("base",Basestate.x,0,Basestate.z,0,Basestate.alpha,0);
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
			if(fabs(posetag.x-target.x)>200||fabs(posetag.z-target.z)>200){
				objetivoactual={target.x,target.y,target.z};
				stgo=statego::ORIENTARSE;
				cango=false;
			}
			posetag={target.x,target.y,target.z};
		}
		else{
			state.state="WORKING";
			st=State::WORKING;
			QMutexLocker ml(&m);
			posetag={target.x,target.y,target.z};;
			objetivoactual={target.x/2,target.y/2,target.z/2};
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
	qDebug()<<objetivoactual.x<<","<<objetivoactual.z;
	writeinfo("poserobot:("+to_string(Basestate.z)+","+to_string(Basestate.x)+")\n"+"posetag:("+to_string(posetag.z)+","+to_string(posetag.x)+")\n"+"objetivoactual:("+to_string(objetivoactual.z)+","+to_string(objetivoactual.x)+")\n"+"subobjetivo:("+to_string(subobjetivo.z)+","+to_string(subobjetivo.x)+")");
	switch(stgo){
	  case statego::ORIENTARSE:
			orientarse();
		break;
	  case statego::HAYOBTACULO:
			hayobt=hayobtaculo();
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
			if (cango)
				stgo=statego::AVANZAR;
			else
			  stgo=statego::CALCULAROBJETIVO;
		break;
	  case statego::AVANZAR:
			avanzar();
		break;
	  case statego::CALCULAROBJETIVO:
			calcularsubobjetivo();
		break;
	  case statego::HELLEGADO:
			hellegado();
		break;
	}
}

void SpecificWorker::orientarse()
{
	QVec objerobot=inner->transform("base",QVec::vec3(objetivoactual.x,0,objetivoactual.z),"world");
	float rot=atan2(objerobot(0),objerobot(2));
	differentialrobot_proxy->setSpeedBase(0,rot);
	sleep(1);
	differentialrobot_proxy->setSpeedBase(0,0);
	stgo=statego::HAYOBTACULO;
}

bool SpecificWorker::hayobtaculo()
{
	QVec objerobot=inner->transform("base",QVec::vec3(objetivoactual.x,0,objetivoactual.z),"world");
	float dist=sqrt((objerobot(0)*objerobot(0))+(objerobot(2)*objerobot(2)));
	if((ldata.data()+50)->dist<dist)
	  return true;
	else 
		return false;
}

void SpecificWorker::calcularsubobjetivo()
{
	int i,j;
	int sentido;
	float disfinal;
	float anglefinal;
	const float R = 500; //Robot radius
// 	for(i=(int)ldata.size()/2; i>1; i--)
// 	{
// 		if( (ldata[i].dist - ldata[i-1].dist) < -R )
// 		{
// 			int k=i-2;
// 			while( (k > 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
// 			{ k--; }
// 			i=k;
// 			break;
// 		}
// 	}
// 	for(j=(int)ldata.size()/2; j<(int)ldata.size()-2; j++)
// 	{
// 		if( (ldata[j].dist - ldata[j+1].dist) < -R )
// 		{
// 			int k=j+2;
// 			while( (k < (int)ldata.size()-1) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
// 			{ k++; }
// 			j=k;
// 			break;
// 		}
// 	}
		float anterior;
		for(i=51;i<100;i++){
			if((ldata[i].dist)-anterior>100){
				sentido=1;
				break;
			}
			anterior=ldata[i].dist;
		}
		for(j=49;j>0;j--){
			if((ldata[j].dist)-anterior>100){
				sentido=-1;
				break;
			}
			anterior=ldata[j].dist;
		}
		i--;
		j++;
		
	if(abs(50-i)<abs(j-50)){
// 		disfinal = ldata[i].dist/4;
// 		anglefinal = ldata[i].angle;
		float dist1=ldata[i].dist;
		float dist2=200;
		float alpha=abs(asin(dist2/dist1));
		disfinal=ldata[i+sentido].dist/2;
		anglefinal=alpha+abs(ldata[i].angle);
		anglefinal=sentido*anglefinal;
	}
	else 
	{
// 		disfinal = ldata[j].dist/4;
// 		anglefinal = ldata[j].angle;
		float dist1=ldata[j].dist;
		float dist2=200;
		float alpha=abs(asin(dist2/dist1));
		disfinal=ldata[j+sentido].dist/2;
		anglefinal=alpha+abs(ldata[j].angle);
		anglefinal=sentido*anglefinal;
	}
	QVec obj=inner->laserTo("world","laser",disfinal,anglefinal);
	subobjetivo={obj(0),0,obj(2)};
	objetivoactual={obj(0),0,obj(2)};
	stgo=statego::ORIENTARSE;
	differentialrobot_proxy->setSpeedBase(0,0);
	cango=false;
}
bool SpecificWorker::puedopasar()
{	
	int i;
	QVec objerobot=inner->transform("base",QVec::vec3(objetivoactual.x,0,objetivoactual.z),"world");
	float distobje=sqrt(objerobot(0)*objerobot(0)+objerobot(2)*objerobot(2));
	for(i=51;i<100;i++)
	{
		float x=fabs(sin((ldata.data()+i)->angle)*(ldata.data()+i)->dist);
		float c=fabs(cos((ldata.data()+i)->angle)*(ldata.data()+i)->dist);
		if (x<250&&c<distobje)
		{
			return false;
		}
	}
	for(i=49;i>0;i--)
	{
		float x=fabs(sin((ldata.data()+i)->angle)*(ldata.data()+i)->dist);
		float c=fabs(cos((ldata.data()+i)->angle)*(ldata.data()+i)->dist);
		if (x<250&&c<distobje)
		{
			return false;
		}
	}
	return true;
}
void SpecificWorker::hellegado()
{
	if(fabs(Basestate.z-objetivoactual.z)<100&&fabs(Basestate.x-objetivoactual.x)<100)
	{
		m.lock();
		if (objetivoactual.x==posetag.x&&objetivoactual.z==posetag.z)
		{
			state.state="FINISH";
			st=State::FINISH;
			if(lado)
				lado=false;
			else
				lado=true;
		}
		else
		{
			objetivoactual=posetag;
		}
		m.unlock();
		differentialrobot_proxy->setSpeedBase(0,0);
		stgo=statego::ORIENTARSE;
		cango=false;
	}
// 	
	else
	{
		QVec objerobot=inner->transform("base",QVec::vec3(objetivoactual.x,0,objetivoactual.z),"world");
		float rot=atan2(objerobot(0),objerobot(2));
		differentialrobot_proxy->setSpeedBase(200,rot);
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
void SpecificWorker::histogram()
{
	static QGraphicsPolygonItem *p;
	static QGraphicsLineItem *l, *sr, *sl, *safety;
	const float R = 500; //Robot radius
	const float SAFETY = 600;

	scene.removeItem(p);
	scene.removeItem(l);
	scene.removeItem(sr);
	scene.removeItem(sl);
	scene.removeItem(safety);
	
	//Search the first increasing step from the center to the right
	int i,j;
	for(i=(int)ldata.size()/2; i>1; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -R )
		{
			int k=i-2;
			while( (k > 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < R ))
			{ k--; }
			i=k;
			break;
		}
	}
	for(j=(int)ldata.size()/2; j<(int)ldata.size()-2; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -R )
		{
			int k=j+2;
			while( (k < (int)ldata.size()-1) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < R ))
			{ k++; }
			j=k;
			break;
		}
	}
	
	safety = scene.addLine(QLine(QPoint(0,-SAFETY/100),QPoint(ldata.size(),-SAFETY/100)), QPen(QColor(Qt::yellow)));
	sr = scene.addLine(QLine(QPoint(i,0),QPoint(i,-40)), QPen(QColor(Qt::blue)));
	sl = scene.addLine(QLine(QPoint(j,0),QPoint(j,-40)), QPen(QColor(Qt::magenta)));
	
	//DRAW		
	QPolygonF poly;
	int x=0;
	poly << QPointF(0, 0);
	
	for(auto d : ldata)
		poly << QPointF(++x, -d.dist/100); // << QPointF(x+5, d.dist) << QPointF(x+5, 0);
	poly << QPointF(x, 0);

	l = scene.addLine(QLine(QPoint(ldata.size()/2,0),QPoint(ldata.size()/2,-20)), QPen(QColor(Qt::red)));
  p = scene.addPolygon(poly, QPen(QColor(Qt::green)));
	
	scene.update();
	
	//select the best subtarget and return coordinates
}





