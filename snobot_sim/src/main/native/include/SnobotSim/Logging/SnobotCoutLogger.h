/*
 * SnobotCoutLogger.h
 *
 *  Created on: Jun 21, 2017
 *      Author: PJ
 */

#ifndef SNOBOTCOUTLOGGER_H_
#define SNOBOTCOUTLOGGER_H_

#include "SnobotSim/Logging/SnobotLogger.h"

namespace SnobotLogging
{
    class EXPORT_ SnobotCoutLogger : public ISnobotLogger
    {
    public:
        SnobotCoutLogger();
        virtual ~SnobotCoutLogger();

        void Log(
                LogLevel aLogLevel,
                int aLineNumber,
                const std::string& aFileName,
                const std::string& aMessage);

    protected:

        std::string FixWindowsSlashes(const std::string& aInput);

        std::string mDirectorySubstring;
    };
}
#endif /* SNOBOTCOUTLOGGER_H_ */
