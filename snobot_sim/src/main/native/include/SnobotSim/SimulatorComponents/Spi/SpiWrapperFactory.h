/*
 * SpiWrapperFactory.h
 *
 *  Created on: Sep 11, 2017
 *      Author: PJ
 */

#ifndef SPIWRAPPERFACTORY_H_
#define SPIWRAPPERFACTORY_H_

#include "SnobotSim/SimulatorComponents/Spi/ISpiWrapper.h"
#include <string>
#include <memory>
#include <map>


class EXPORT_ SpiWrapperFactory
{

private:
    SpiWrapperFactory();
    virtual ~SpiWrapperFactory();

public:
    static const std::string AUTO_DISCOVER_NAME;
    static const std::string ADXRS450_GYRO_NAME;
    static const std::string ADXL345_ACCELEROMETER_NAME;
    static const std::string ADXL362_ACCELEROMETER_NAME;
    static const std::string NAVX;

    static SpiWrapperFactory& Get();

    std::shared_ptr<ISpiWrapper> GetSpiWrapper(int aPort);

    void RegisterDefaultWrapperType(int aPort, const std::string& aWrapperType);
    void ResetDefaults();

protected:


    std::shared_ptr<ISpiWrapper> CreateWrapper(int aPort, const std::string& aType);

    std::map<int, std::string> mDefaultsMap;

    static SpiWrapperFactory sINSTANCE;
};

#endif /* SPIWRAPPERFACTORY_H_ */
