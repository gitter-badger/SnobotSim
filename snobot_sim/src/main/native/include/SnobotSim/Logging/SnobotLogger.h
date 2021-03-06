/*
 * SnobotLogger.h
 *
 *  Created on: Jun 21, 2017
 *      Author: PJ
 */

#ifndef SNOBOTLOGGER_H_
#define SNOBOTLOGGER_H_

#include <string>
#include <sstream>
#include "SnobotSim/ExportHelper.h"

namespace SnobotLogging
{
    enum LogLevel
    {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        CRITICAL = 3,
        NONE = 4,
    };

    class EXPORT_ ISnobotLogger
    {
    public:

        virtual ~ISnobotLogger() {}

        virtual void SetLogLevel(LogLevel aLevel)
        {
            mLogLevel = aLevel;
        }

        virtual void Log(
                LogLevel aLogLevel,
                int aLineNumber,
                const std::string& aFileName,
                const std::string& aMessage) = 0;

        LogLevel mLogLevel;
    };

    void EXPORT_ SetLogger(ISnobotLogger* aLogger);

    void EXPORT_ Log(
            LogLevel aLogLevel,
            int aLineNumber,
            const std::string& aFileName,
            const std::string& aMessage);
}


#ifndef __FUNCTION_NAME__
    #ifdef WIN32   //WINDOWS
        #define __FUNCTION_NAME__   __FUNCTION__
    #else          //*NIX
        #define __FUNCTION_NAME__   __func__
    #endif
#endif

#define SNOBOT_LOG(logLevel, messageStream)                           \
{                                                                     \
    std::stringstream message;                                        \
    message << messageStream;                                         \
    SnobotLogging::Log(logLevel, __LINE__, __FILE__, message.str());  \
}

#define LOG_UNSUPPORTED()                                   SNOBOT_LOG(SnobotLogging::CRITICAL, "Unsupported function " << __FUNCTION_NAME__);
#define LOG_UNSUPPORTED_WITH_MESSAGE(message)               SNOBOT_LOG(SnobotLogging::CRITICAL, "Unsupported function " << __FUNCTION_NAME__ << " " << message);
#define LOG_UNSUPPORTED_WITH_LEVEL(level)                   SNOBOT_LOG(level,                   "Unsupported function " << __FUNCTION_NAME__);
#define LOG_UNSUPPORTED_WITH_LEVEL_AND_MSG(level, message)  SNOBOT_LOG(level,                   "Unsupported function " << __FUNCTION_NAME__ << " " << message);


#endif /* SNOBOTLOGGER_H_ */
