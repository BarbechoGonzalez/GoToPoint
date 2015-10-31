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
	QMutexLocker m(&mutex);
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
		if (puedopasar()){
			//avanzar
			if(hellegado()){
				st=State::FINISH;
				state="FINISH";
			}
		}
		else
			calcularsubobjetivo();
	}
	else
	  calcularsubobjetivo();
}

void SpecificWorker::orientarse()
{

}

bool SpecificWorker::hayobtaculo()
{
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






