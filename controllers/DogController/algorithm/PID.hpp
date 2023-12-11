/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    PID.h
  * @author  BigeYoung 851756890@qq.com & M3chD09
  * @brief   PID controller sets. This file provides traditional PID controller 
  *			     and modern Fuzzy PID controller.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <algorithm/filters.hpp>
#include <bind.hpp>
#include <limits>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
/* Private macros ------------------------------------------------------------*/
// typedef uint32_t (*SystemTick_Fun)(void);
#define Get_SystemTick() robot->getTime()
/* Private type --------------------------------------------------------------*/
namespace std_lib
{
template<class T>
T constrain(T input,T min,T max)
{
  if (input <= min)
    return min;
  else if(input >= max)
    return max;
  return input;
}
}

/* Exported types ------------------------------------------------------------*/
class myPIDTimer
{
public:
    static uint8_t getMicroTick_regist(double (*getTick_fun)(void));
                                            /*<! Regist get time function */
protected:
    // static SystemTick_Fun Get_SystemTick;   /*<! Pointer of function to get system tick */
    double dt;				                        /*!< Differentiation of real time*/
    double last_time; 	                  /*!< Last recorded real time from systick*/
    double UpdataTimeStamp(void);                                 
};

/** 
* @brief Class for open loop control.
*/
class OpenLoop
{
public:
    OpenLoop(float gain = 1.0f) : Gain(gain) {}

    float Adjust()
    {
        Out = Gain * Target;
        return Out;
    }
    float Target = 0, Current = 0;
    float Out = 0;

    float Gain = 1.0f;
};

/** 
* @brief Class for traditional PID control.
*/
class myPID : public myPIDTimer
{
public:
    myPID() {}
    myPID(float _Kp, float _Ki, float _Kd) : Kp(_Kp), Ki(_Ki), Kd(_Kd){}
    void SetPIDParam(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max)
    {
      Kp = _Kp;
      Ki = _Ki;
      Kd = _Kd;
      I_Term_Max = _I_Term_Max;
      Out_Max = _Out_Max;
    };
    float Adjust();
    float Target = 0, Current = 0, Error = 0;
    float Out = 0;

    float Kp = 0, Ki = 0, Kd = 0;
    float I_Term_Max = 0;        /*<! I项限幅 */
    float Out_Max = 0;           /*<! 输出限幅 */
	
	float I_Term = 0;			/* 积分器输出 */
    float P_Term = 0;			/* 比例器输出 */
    float D_Term = 0;			/* 微分器输出 */

    float I_SeparThresh = 400;   /*!< 积分分离阈值，需为正数。std::abs(error)大于该阈值取消积分作用。*/


    float VarSpeed_I_A = ULONG_MAX; /*!< 变速积分 A，需为正数。*/
    float VarSpeed_I_B = ULONG_MAX; /*!< 变速积分 B，需为正数， \n
                                     在 error<=B 的区间内，为普通积分效果， \n
                                     在 B<error<=A+B 的区间内，为变速积分效果， \n
                                     在 A+B<error 的区间内，不继续积分。*/

    float DeadZone = 0.0001; 		    /*!< 死区，需为整数，std::abs(error)小于DeadZone时，输出为0。 */

    LowPassFilter LowPass_error = LowPassFilter(1);
    LowPassFilter LowPass_d_err = LowPassFilter(1); /*!< 不完全微分。 */

    bool D_of_Current = false; /*!< 启用微分先行，文献中Current多译作Process Variable(PV)。 */

private:
    float pre_error = 0;
    float integral_e = 0;

    float pre_Current = 0;
};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
double myPIDTimer::UpdataTimeStamp(void)
{
  double now_time;
  
  /*Check `Get_SystemTick` */
  if(Get_SystemTick() != 0)
  {
    /*Convert to system time*/
    if (last_time == 0)
    {
      last_time = Get_SystemTick();
      return 1;
    }
    now_time = Get_SystemTick();

    /*Overflow*/
    if (now_time < last_time)
      dt = (double)(now_time + (0xFFFFFFFF - last_time));
    else
      dt = (double)(now_time - last_time);

    last_time = now_time;
    
    return 0;
  }
  else{
    dt = 0;
    return 1;
  }
}


float myPID::Adjust()
{
  /*Error time*/
  if (UpdataTimeStamp())
    return 0;

  Error = Target - Current;

  if (std::abs(Error) < DeadZone)
  {
    Out = 0;
    return Out;
  }

  /* Using Low-Pass Filter to preprocess*/
  Error = LowPass_error.f(Error);

  P_Term = Kp * Error;

	/* PID with Changing integration rate */
	float I_VarSpeedf = 0;
	if (std::abs(Error) <= VarSpeed_I_B)
	  I_VarSpeedf = 1;
	else if (std::abs(Error) <= double(VarSpeed_I_A) + VarSpeed_I_B)
	  I_VarSpeedf = (VarSpeed_I_A - (std::abs(Error)) + VarSpeed_I_B) / VarSpeed_I_A;
  
  if(std::abs(Ki) > std::numeric_limits<float>::epsilon()){
    integral_e += I_VarSpeedf * Error * dt;
    /*Constrain*/
    integral_e = std_lib::constrain(integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
  }
  else{
    integral_e = 0;
  }
  
  /* Using Integral separation */
  if (std::abs(Error) < I_SeparThresh)
  { 
    I_Term = Ki * integral_e;
    /*Constarin*/
    I_Term = std_lib::constrain(I_Term, -I_Term_Max, I_Term_Max);
  }
  else{
    /*Clear*/
    I_Term = 0;
  }
  
  float d_err = 0;
  if (D_of_Current)
    d_err = (Current - pre_Current) / dt;
  else
    d_err = (Error - pre_error) / dt;
  d_err = LowPass_d_err.f(d_err);
  D_Term = Kd * d_err;

  pre_error = Error;

  Out = P_Term + I_Term + D_Term;
  
  /* Constarin */
  Out = std_lib::constrain(Out, -Out_Max, Out_Max);
  
  return Out;
}
