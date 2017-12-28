
#include <jni.h>
#include <assert.h>

#include "com_ctre_phoenix_MotorControl_CAN_MotControllerJNI.h"
#include "ctre/phoenix/CCI/MotController_CCI.h"


/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    Create
 * Signature: (I)J
 */
JNIEXPORT jlong JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_Create
  (JNIEnv *, jclass, jint baseArbId)
{
	return (jlong) c_MotController_Create1(baseArbId);
//    return 0;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetDeviceNumber
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetDeviceNumber
  (JNIEnv *, jclass, jlong handle)
{
	int deviceNumber = 0;

	c_MotController_GetDeviceNumber(&handle, &deviceNumber);


	return deviceNumber;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetDemand
 * Signature: (JIII)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetDemand
  (JNIEnv *, jclass, jlong handle, jint mode, jint demand0, jint demand1)
{
	c_MotController_SetDemand(&handle, mode, demand0, demand1);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetNeutralMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetNeutralMode
  (JNIEnv *, jclass, jlong handle, jint neutralMode)
{
	c_MotController_SetNeutralMode(&handle, neutralMode);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetSensorPhase
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetSensorPhase
  (JNIEnv *, jclass, jlong handle, jboolean phaseSensor)
{
	c_MotController_SetSensorPhase(&handle, phaseSensor);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetInverted
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetInverted
  (JNIEnv *, jclass, jlong handle, jboolean inverted)
{
	c_MotController_SetInverted(&handle, inverted);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigOpenLoopRamp
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigOpenLoopRamp
  (JNIEnv *, jclass, jlong handle, jfloat secondsFromNeutralToFull, jint timeoutMs)
{
	return (jint) c_MotController_ConfigOpenLoopRamp(&handle, secondsFromNeutralToFull, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigClosedLoopRamp
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigClosedLoopRamp
  (JNIEnv *, jclass, jlong handle, jfloat secondsFromNeutralToFull, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
//	return (jint) c_MotController_ConfigClosedLoopRamp(&handle, secondsFromNeutralToFull, timeoutMs);
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigPeakOutputForward
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigPeakOutputForward
  (JNIEnv *, jclass, jlong handle, jfloat percentOut, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
//	return (jint) c_MotController_ConfigPeakOutputForward(&handle, percentOut, timeoutMs);
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigPeakOutputReverse
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigPeakOutputReverse
  (JNIEnv *, jclass, jlong handle, jfloat percentOut, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
//	return (jint) c_MotController_ConfigPeakOutputReverse(&handle, percentOut, timeoutMs);
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigNominalOutputForward
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigNominalOutputForward
  (JNIEnv *, jclass, jlong handle, jfloat, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
	return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigNominalOutputReverse
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigNominalOutputReverse
  (JNIEnv *, jclass, jlong handle, jfloat, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigNeutralDeadband
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigNeutralDeadband
  (JNIEnv *, jclass, jlong handle, jfloat, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigVoltageCompSaturation
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigVoltageCompSaturation
  (JNIEnv *, jclass, jlong handle, jfloat, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
	return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigVoltageMeasurementFilter
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigVoltageMeasurementFilter
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
	return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    EnableVoltageCompensation
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_EnableVoltageCompensation
  (JNIEnv *, jclass, jlong handle, jboolean)
{
    LOG_UNSUPPORTED_CAN_FUNC("");

}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetBusVoltage
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetBusVoltage
  (JNIEnv *, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetMotorOutputPercent
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetMotorOutputPercent
  (JNIEnv *, jclass, jlong handle)
{
    float percentage = 0;
    c_MotController_GetMotorOutputPercent(&handle, &percentage);

    return percentage;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetOutputCurrent
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetOutputCurrent
  (JNIEnv *, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetTemperature
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetTemperature
  (JNIEnv *, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigRemoteFeedbackFilter
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigRemoteFeedbackFilter
  (JNIEnv *, jclass, jlong handle, jint, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigSelectedFeedbackSensor
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigSelectedFeedbackSensor
  (JNIEnv *, jclass, jlong handle, jint feedbackDevice, jint timeoutMs)
{
    return (jint) c_MotController_ConfigSelectedFeedbackSensor(&handle, feedbackDevice, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetSelectedSensorPosition
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetSelectedSensorPosition
  (JNIEnv *, jclass, jlong handle)
{
	int output = 0;
	c_MotController_GetSelectedSensorPosition(&handle, &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetSelectedSensorVelocity
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetSelectedSensorVelocity
  (JNIEnv *, jclass, jlong handle)
{
	int output = 0;
	c_MotController_GetSelectedSensorVelocity(&handle, &output);
    return output;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetSelectedSensorPosition
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetSelectedSensorPosition
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetControlFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetControlFramePeriod
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetStatusFramePeriod
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetStatusFramePeriod
  (JNIEnv *, jclass, jlong handle, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetStatusFramePeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetStatusFramePeriod
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigVelocityMeasurementPeriod
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigVelocityMeasurementPeriod
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs);

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigVelocityMeasurementWindow
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigVelocityMeasurementWindow
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigForwardLimitSwitchSource
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigForwardLimitSwitchSource
  (JNIEnv *, jclass, jlong handle, jint, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigReverseLimitSwitchSource
 * Signature: (JIIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigReverseLimitSwitchSource
  (JNIEnv *, jclass, jlong handle, jint, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    EnableLimitSwitches
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_EnableLimitSwitches
  (JNIEnv *, jclass, jlong handle, jboolean)
{
    LOG_UNSUPPORTED_CAN_FUNC("");

}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigForwardSoftLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigForwardSoftLimit
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigReverseSoftLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigReverseSoftLimit
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    EnableSoftLimits
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_EnableSoftLimits
  (JNIEnv *, jclass, jlong handle, jboolean)
{
    LOG_UNSUPPORTED_CAN_FUNC("");

}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    Config_kP
 * Signature: (JIFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_Config_1kP
  (JNIEnv *, jclass, jlong handle, jint slot, jfloat value, jint timeoutMs)
{
    return (jint) c_MotController_Config_kP(&handle, slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    Config_kI
 * Signature: (JIFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_Config_1kI
  (JNIEnv *, jclass, jlong handle, jint slot, jfloat value, jint timeoutMs)
{
    return (jint) c_MotController_Config_kI(&handle, slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    Config_kD
 * Signature: (JIFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_Config_1kD
  (JNIEnv *, jclass, jlong handle, jint slot, jfloat value, jint timeoutMs)
{
    return (jint) c_MotController_Config_kD(&handle, slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    Config_kF
 * Signature: (JIFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_Config_1kF
  (JNIEnv *, jclass, jlong handle, jint slot, jfloat value, jint timeoutMs)
{
    return (jint) c_MotController_Config_kF(&handle, slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    Config_IntegralZone
 * Signature: (JIFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_Config_1IntegralZone
  (JNIEnv *, jclass, jlong handle, jint slot, jfloat value, jint timeoutMs)
{
    return (jint) c_MotController_Config_IntegralZone(&handle, slot, value, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigAllowableClosedloopError
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigAllowableClosedloopError
  (JNIEnv *, jclass, jlong handle, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigMaxIntegralAccumulator
 * Signature: (JIFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigMaxIntegralAccumulator
  (JNIEnv *, jclass, jlong handle, jint, jfloat, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SetIntegralAccumulator
 * Signature: (JFI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SetIntegralAccumulator
  (JNIEnv *, jclass, jlong handle, jfloat, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetClosedLoopError
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetClosedLoopError
  (JNIEnv *, jclass, jlong handle, jint slotIdx)
{
	int closedLoopError = 0;
	c_MotController_GetClosedLoopError(&handle, &closedLoopError, slotIdx);
    return closedLoopError;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetIntegralAccumulator
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetIntegralAccumulator
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetErrorDerivative
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetErrorDerivative
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    SelectProfileSlot
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_SelectProfileSlot
  (JNIEnv *, jclass, jlong handle, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");

}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigMotionCruiseVelocity
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigMotionCruiseVelocity
  (JNIEnv *, jclass, jlong handle, jint sensorUnitsPer100ms, jint timeoutMs)
{
    return (jint) c_MotController_ConfigMotionCruiseVelocity(&handle, sensorUnitsPer100ms, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigMotionAcceleration
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigMotionAcceleration
  (JNIEnv *, jclass, jlong handle, jint sensorUnitsPer100msPerSec, jint timeoutMs)
{
    return (jint) c_MotController_ConfigMotionAcceleration(&handle, sensorUnitsPer100msPerSec, timeoutMs);
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetLastError
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetLastError
  (JNIEnv *, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    GetFirmwareVersion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_GetFirmwareVersion
  (JNIEnv *, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    HasResetOccurred
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_HasResetOccurred
  (JNIEnv *, jclass, jlong handle)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigSetCustomParam
 * Signature: (JIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigSetCustomParam
  (JNIEnv *, jclass, jlong handle, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigGetCustomParam
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigGetCustomParam
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs);

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigSetParameter
 * Signature: (JIFIII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigSetParameter
  (JNIEnv *, jclass, jlong handle, jint, jfloat, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigGetParameter
 * Signature: (JIII)F
 */
JNIEXPORT jfloat JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigGetParameter
  (JNIEnv *, jclass, jlong handle, jint, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigPeakCurrentLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigPeakCurrentLimit
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigPeakCurrentDuration
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigPeakCurrentDuration
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    ConfigContinuousCurrentLimit
 * Signature: (JII)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_ConfigContinuousCurrentLimit
  (JNIEnv *, jclass, jlong handle, jint, jint timeoutMs)
{
    LOG_UNSUPPORTED_CAN_FUNC("");
    return (jint) NotImplemented;
}

/*
 * Class:     com_ctre_phoenix_MotorControl_CAN_MotControllerJNI
 * Method:    EnableCurrentLimit
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_ctre_phoenix_MotorControl_CAN_MotControllerJNI_EnableCurrentLimit
  (JNIEnv *, jclass, jlong handle, jboolean)
{
    LOG_UNSUPPORTED_CAN_FUNC("");

}