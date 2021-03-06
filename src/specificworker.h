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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <qt4/QtCore/QMutex>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	float go(const TargetPose &target);
	NavState getState();
	void stop();

public slots:
	void compute(); 	

private:
	QMutex m;
	enum class State  { WORKING, FINISH, IDLE, BLOCKED};
	enum class statego  { ORIENTARSE, AVANZAR, HAYOBTACULO, PUEDOPASAR, CALCULAROBJETIVO, HELLEGADO};
	statego stgo;
	State st;
	TargetPose subobjetivo;
	TargetPose objetivoactual;
	TargetPose posetag;
	NavState state;
	TBaseState Basestate;
	InnerModel *inner;
	QGraphicsScene scene;
	int repe;
	RoboCompLaser::TLaserData ldata;
	bool cango;
	bool lado;
	bool hayobt;
	void gototarget();
	void orientarse();
	bool hayobtaculo();
	void calcularsubobjetivo();
	bool puedopasar();
	void hellegado();
	void avanzar();
	void writeinfo(string _info);
	void histogram();
};

#endif

