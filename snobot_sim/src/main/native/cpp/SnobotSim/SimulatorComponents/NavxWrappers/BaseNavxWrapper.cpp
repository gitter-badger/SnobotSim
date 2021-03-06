/*
 * BaseNavxWrapper.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: PJ
 */

#include "SnobotSim/SimulatorComponents/NavxWrappers/BaseNavxWrapper.h"
#include "SnobotSim/SensorActuatorRegistry.h"

BaseNavxWrapper::BaseNavxWrapper(int aBasePort, const std::shared_ptr<NavxSimulator>& aNavx)  :
	mXWrapper(new AccelerometerWrapper(AccelerometerWrapper::AXIS_X, aNavx)),
	mYWrapper(new AccelerometerWrapper(AccelerometerWrapper::AXIS_Y, aNavx)),
	mZWrapper(new AccelerometerWrapper(AccelerometerWrapper::AXIS_Z, aNavx)),

	mYawWrapper(new GyroWrapper(GyroWrapper::AXIS_YAW, aNavx)),
	mPitchWrapper(new GyroWrapper(GyroWrapper::AXIS_PITCH, aNavx)),
	mRollWrapper(new GyroWrapper(GyroWrapper::AXIS_ROLL, aNavx))
{

	SensorActuatorRegistry::Get().Register(aBasePort + 0, mXWrapper);
	SensorActuatorRegistry::Get().Register(aBasePort + 1, mYWrapper);
	SensorActuatorRegistry::Get().Register(aBasePort + 2, mZWrapper);

	SensorActuatorRegistry::Get().Register(aBasePort + 0, mYawWrapper);
	SensorActuatorRegistry::Get().Register(aBasePort + 1, mPitchWrapper);
	SensorActuatorRegistry::Get().Register(aBasePort + 2, mRollWrapper);
}

BaseNavxWrapper::~BaseNavxWrapper()
{

}

BaseNavxWrapper::AccelerometerWrapper::AccelerometerWrapper(AxisType aAxisType, const std::shared_ptr<NavxSimulator>& aNavx) :
	IAccelerometerWrapper("Hello"),
	mAxisType(aAxisType),
	mNavx(aNavx)
{

}

void BaseNavxWrapper::AccelerometerWrapper::SetAcceleration(double aAcceleration)
{
	if(!mNavx)
	{
		return;
	}

    switch(mAxisType)
    {
    case AXIS_X:
    	mNavx->SetX(aAcceleration);
    case AXIS_Y:
    	mNavx->SetY(aAcceleration);
    case AXIS_Z:
    	mNavx->SetZ(aAcceleration);
    }
}


double BaseNavxWrapper::AccelerometerWrapper::GetAcceleration()
{
	if(!mNavx)
	{
		return 0;
	}

    switch(mAxisType)
    {
    case AXIS_X:
        return mNavx->GetX();
    case AXIS_Y:
        return mNavx->GetY();
    case AXIS_Z:
        return mNavx->GetZ();
    }
	return 0;
}


BaseNavxWrapper::GyroWrapper::GyroWrapper(AxisType aAxisType, const std::shared_ptr<NavxSimulator>& aNavx) :
	IGyroWrapper("Hello"),
	mAxisType(aAxisType),
	mNavx(aNavx)
{

}

void BaseNavxWrapper::GyroWrapper::SetAngle(double aAngle)
{
    switch(mAxisType)
    {
    case AXIS_YAW:
    	mNavx->SetYaw(aAngle);
    case AXIS_PITCH:
    	mNavx->SetPitch(aAngle);
    case AXIS_ROLL:
    	mNavx->SetRoll(aAngle);
    }
}


double BaseNavxWrapper::GyroWrapper::GetAngle()
{
    switch(mAxisType)
    {
    case AXIS_YAW:
        return mNavx->GetYaw();
    case AXIS_PITCH:
        return mNavx->GetPitch();
    case AXIS_ROLL:
        return mNavx->GetRoll();
    }
	return 0;
}




