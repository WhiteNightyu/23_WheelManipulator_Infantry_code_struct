/*! @file myMiddleware.h
 *
 *  @brief header file for myMiddleware.cpp
 *
 */
#ifndef MYMIDDLEWARE_H
#define MYMIDDLEWARE_H

#include <FreeRTOS.h>
#include <queue.h>
#include "abstractMotor.h"
#include "AbstractIMU.h"
#include "boardComProtocol.h"

enum jointID
{
  RF = 0,
  RB = 1,
  LF = 2,
  LB = 3
};

#define RF_JOINT_ID 1
#define RB_JOINT_ID 2
#define LF_JOINT_ID 3
#define LB_JOINT_ID 4

#define JOINT_RESET_DELAY 1000

class MyMiddlewareClassdef
{
private:
#pragma pack(1)
  struct recDataStructdef
  {
    /*左右轮当前速度*/
    float linerSpeed = 0;
    /*速度向量*/
    float target_speed_y = 0;
    float target_speed_z = 0;
    float target_height = 0.14;
    /*功率系数*/
    float power_limit_scale = 0.;
    /*向心力系数（v^2/r)*/
    float centripetal_force_scale = 0.f;

    chassis_state_structdef chassis_state = {};
  } rec_data;
#pragma pack()
public:
  MyMiddlewareClassdef();
  void init(QueueHandle_t _USART_TxPort, uint8_t _port_num[2], LPMS_BE2_Typedef *_lmps);
  //连接状态控制
  void Link_Check();

  void setJointTorque(float tarTorque[4]);
  void sendDigitalBoard(float totalTorque[2], float _angularSpeed, uint16_t _is_balance, uint16_t _is_reset, uint16_t _is_falldown, uint16_t _is_overturn, float _f_angle, float _b_angle, float _end_x, float _end_y);

  //关节初始化
  void jointInit(MotorHT04Classdef _motor[4]);
  //关节自动复位
  void jointReset();
  //关节自动标定
  void setJointOffset();
  //关节指令清零
  void jointCommandClear();

  float *getMotorAngle();

  float *getMotorTorque();

  void processRecData(Send2ControllerStructdef *recData);
  const recDataStructdef &getRecData() { return rec_data; }

  LinearDataStructdef *getAccData();
  AngularDataStructdef *getEularData();
  AngularDataStructdef *getAngleVelData();

  abstractMotor<MotorHT04Classdef> jointMotor[4];

  int32_t crcErrCnt = 0;

private:
  abstractIMUClassdef<LPMS_BE2_Typedef> imu;

  RecFromControllerStructdef sendMessage;
  float jointAngle[4];
  float jointTorque[4];

  QueueHandle_t USART_TxPort;
  USART_COB Usart_TxCOB;
  uint8_t port_num[2];

  uint8_t LinkCount = 90;
  uint8_t continueError = 0;
  int16_t packetLossRate = 0;
};
#endif
