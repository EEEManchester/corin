/**************************************************************************/
/*! 
    @file     Adafruit_INA219.h
    @author   W. Cheah (University of Manchester)
	
	This is a library for the Corin hexapod
  v1.0  - First release
*/
/**************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

/*=========================================================================
    LEG LILNK LENGTHS
    -----------------------------------------------------------------------*/
    #define LL_LENG_1                               (0.0445)
    #define LL_LENG_2                               (0.192)
    #define LL_LENG_3                               (0.19526)
    #define LL_LENG_d                               (0.02839)
/*=========================================================================*/

/*=========================================================================
    GAIT PARAMETERS
    -----------------------------------------------------------------------*/
    #define CORIN_BODY_HEIGHT                     (0.08508494)    //robot ochestra: -0.058, nominal: 0.068 # arm demo: 0.17
    #define CORIN_STANCE_WIDTH                    (0.20881583)
    #define CORIN_GAIT_TYPE                       (4)
    #define CORIN_STEP_STROKE                     (0.15)
    #define CORIN_STEP_HEIGHT                     (0.05)
    #define CORIN_WALKING_SPEED                   (1.1)
    #define CORIN_CYCLE_TIME                      (1.1)
    #define CORIN_DIRECTION                       (1.0)
    #define CORIN_ACTIVATE                        (1)
    #define CORIN_DEMO                            (0)
/*=========================================================================*/

/*=========================================================================
    BODY POSE
    -----------------------------------------------------------------------*/
    #define CORIN_BODY_POSE_X                     (0)
    #define CORIN_BODY_POSE_Y                     (0)
    #define CORIN_BODY_POSE_Z                     (CORIN_BODY_HEIGHT)
    #define CORIN_BODY_POSE_R                     (0)
    #define CORIN_BODY_POSE_P                     (0)
    #define CORIN_BODY_POSE_Y                     (0)
/*=========================================================================*/

/*=========================================================================
    OFFSET FROM COB TO LEG USING BODY FRAME
    -----------------------------------------------------------------------*/
    #define CORIN_COXA_X                          (0.2)
    #define CORIN_COXA_Y                          (0.125)
    #define CORIN_COXA_Z                          (0.0)
/*=========================================================================*/

/*=========================================================================
    NOMINAL STANCE
    -----------------------------------------------------------------------*/
    #define CORIN_NOM_X                          (0.0)
    #define CORIN_NOM_Y                          (CORIN_STANCE_WIDTH)
    #define CORIN_NOM_Z                          (-CORIN_BODY_HEIGHT)
/*=========================================================================*/

/*=========================================================================
    TRANSFORMATION MATRICES
    w = world frame  b = body frame  numbering = leg number
    -----------------------------------------------------------------------*/
    #define INA219_REG_CURRENT                     (0x04)
/*=========================================================================*/

/*=========================================================================
    Y-AXIS CONVERSION
    -----------------------------------------------------------------------*/
    #define INA219_REG_CALIBRATION                 (0x05)
/*=========================================================================*/
/*
class Adafruit_INA219{
 public:
  Adafruit_INA219(uint8_t addr = INA219_ADDRESS);
  void begin(void);
  void begin(uint8_t addr);
  void setCalibration_32V_2A(void);
  void setCalibration_32V_1A(void);
  void setCalibration_16V_400mA(void);
  float getBusVoltage_V(void);
  float getShuntVoltage_mV(void);
  float getCurrent_mA(void);

 private:
  uint8_t ina219_i2caddr;
  uint32_t ina219_calValue;
  // The following multipliers are used to convert raw current and power
  // values to mA and mW, taking into account the current config settings
  uint32_t ina219_currentDivider_mA;
  uint32_t ina219_powerDivider_mW;
  
  void wireWriteRegister(uint8_t reg, uint16_t value);
  void wireReadRegister(uint8_t reg, uint16_t *value);
  int16_t getBusVoltage_raw(void);
  int16_t getShuntVoltage_raw(void);
  int16_t getCurrent_raw(void);
};
*/
