/*! @file myMiddleware.cpp
 *
 *  @brief Intermediate layer that coordinates communication, data conversion, connectivity and other functions
 *          统筹通信，数据转换，连接等功能的中间层
 */
#include "myMiddleware.h"
#include "user_data.h"
#include "Middlewares/Algorithm/filters.h"
#include "internal.h"

#define MF9025_TORQUE_CONSTANT 0.32f

MyMiddlewareClassdef::MyMiddlewareClassdef()
{
    jointMotor[RF].baseAngle = -30.f / ratio_rad_to_degree;
    jointMotor[RF].Polarity = 1;

    jointMotor[RB].baseAngle = 210.f / ratio_rad_to_degree;
    jointMotor[RB].Polarity = 1;

    jointMotor[LF].baseAngle = -30.f / ratio_rad_to_degree;
    jointMotor[LF].Polarity = -1;

    jointMotor[LB].baseAngle = 210.f / ratio_rad_to_degree;
    jointMotor[LB].Polarity = -1;

    imu.bindAccCoordinate(abstractIMU::imuWorldAccZ, abstractIMU::imuWorldAccY, abstractIMU::imuWorldAccX);
    imu.bindEulerCoordinate(abstractIMU::imuWorldPitch, abstractIMU::imuWorldYaw, abstractIMU::imuWorldRoll);

    imu.accPolarity.x = 1;
    imu.accPolarity.y = 1;
    imu.accPolarity.z = 1;

    imu.eularPolarity.pitch = - 1;
    imu.eularPolarity.roll = -1;
    imu.eularPolarity.yaw = 1;

    imu.eularBaseData.pitch = -1.6f / ratio_rad_to_degree;
    imu.eularBaseData.roll = -0.0f / ratio_rad_to_degree;
    imu.eularBaseData.yaw = 0;
}

void MyMiddlewareClassdef::init(QueueHandle_t _USART_TxPort, uint8_t _port_num[2], LPMS_BE2_Typedef *_lmps)
{
    USART_TxPort = _USART_TxPort;
    Usart_TxCOB.address = &(this->sendMessage);
    Usart_TxCOB.len = sizeof(RecFromControllerStructdef);

    this->port_num[0] = _port_num[0];
    this->port_num[1] = _port_num[1];

    imu.bindIMU(_lmps);
}

void MyMiddlewareClassdef::setJointTorque(float tarTorque[4])
{
    for (int i = 0; i < 4; i++)
    {
        jointMotor[i].setMotorCurrentOut(tarTorque[i]);
    }
}

float *MyMiddlewareClassdef::getMotorAngle()
{
    for (int i = 0; i < 4; i++)
    {
        jointAngle[i] = jointMotor[i].getMotorAngle();
    }
    return jointAngle;
}

float *MyMiddlewareClassdef::getMotorTorque()
{
    for(int i = 0;i<4;i++)
    {
        jointTorque[i] = jointMotor[i].getMotorTorque();
    }
    return jointTorque;
}

void MyMiddlewareClassdef::jointInit(MotorHT04Classdef _motor[4])
{
    vTaskDelay(10);
    jointMotor[RF].init(CAN1_TxPort, &_motor[RF]);
    vTaskDelay(10);
    jointMotor[RB].init(CAN1_TxPort, &_motor[RB]);
    vTaskDelay(10);
    jointMotor[LF].init(CAN2_TxPort, &_motor[LF]);
    vTaskDelay(10);
    jointMotor[LB].init(CAN2_TxPort, &_motor[LB]);
    vTaskDelay(10);
}

void MyMiddlewareClassdef::jointReset()
{
    jointMotor[RF].setMotorSpeed(-4, 1.5, -0);
    jointMotor[RB].setMotorSpeed(4, 1.5, 0);
    jointMotor[LF].setMotorSpeed(-4, 1.5, -0);
    jointMotor[LB].setMotorSpeed(4, 1.5, 0);
    vTaskDelay(JOINT_RESET_DELAY);
}

void MyMiddlewareClassdef::setJointOffset()
{
    for (int i = 0; i < 4; i++)
    {
        jointMotor[i].setZeroPosition();
        vTaskDelay(5);
    }
}

void MyMiddlewareClassdef::jointCommandClear()
{
    float clear[4] = {0, 0, 0, 0};
    setJointTorque(clear);
}

void MyMiddlewareClassdef::sendDigitalBoard(float totalTorque[2], float _angularSpeed, uint16_t _is_balance, uint16_t _is_reset, uint16_t _is_falldown, uint16_t _is_overturn, float _f_angle, float _b_angle,float _end_x,float _end_y)
{
    sendMessage.wheelCurrent[R] = (int16_t)(totalTorque[R] * COM_9025_TORQUE_RATIO);
    sendMessage.wheelCurrent[L] = (int16_t)(totalTorque[L] * COM_9025_TORQUE_RATIO);

    sendMessage.currentAngularSpeed = _angularSpeed * COM_SPEED_Z_RATIO;

    sendMessage.controller_state.Balance_State = _is_balance;
    sendMessage.controller_state.Reset_State = _is_reset;
    sendMessage.controller_state.Falldown_State = _is_falldown;
    sendMessage.controller_state.Overturn_State = _is_overturn;

    sendMessage.f_angle = (int16_t)(_f_angle*100.f);
    sendMessage.b_angle = (int16_t)(_b_angle*100.f);
    sendMessage.end_x = (int16_t)(_end_x*100.f);
    sendMessage.end_y = (int16_t)(_end_y*100.f);

    sendMessage.fcs = crc16(&sendMessage, sizeof(RecFromControllerStructdef) - 2);
    Usart_TxCOB.port_num = port_num[0];
    xQueueSend(USART_TxPort, &Usart_TxCOB, 0);
    Usart_TxCOB.port_num = port_num[1];
    xQueueSend(USART_TxPort, &Usart_TxCOB, 0);
}

LinearDataStructdef *MyMiddlewareClassdef::getAccData()
{
    return imu.getAccData();
}

AngularDataStructdef *MyMiddlewareClassdef::getEularData()
{
    return imu.getEularData();
}

AngularDataStructdef *MyMiddlewareClassdef::getAngleVelData()
{
    return imu.getAngleVelData();
}

void MyMiddlewareClassdef::Link_Check()
{
    LinkCount++;
    LinkCount = upper::constrain(LinkCount, 0, 100);
    if (packetLossRate > 250 || continueError > 25) //连续丢包25次或者丢包率达到75%
    {
        do
        {
            HAL_UART_DeInit(&huart3);
            vTaskDelay(10);
        } while (HAL_UART_Init(&huart3) != HAL_OK);
        continueError = 0;
        packetLossRate = 0;
    }

    if (LinkCount >= 200)
    {
        rec_data.linerSpeed = 0;
        rec_data.target_speed_y = 0;
        rec_data.target_speed_z = 0;
        rec_data.target_height = 0.15f;
    }

    jointMotor[RF].motorCheckLink();
    jointMotor[RB].motorCheckLink();
    jointMotor[LF].motorCheckLink();
    jointMotor[LB].motorCheckLink();

}

void MyMiddlewareClassdef::processRecData(Send2ControllerStructdef *_recData)
{
    static MeanFilter<5> meanFilter[2];
    static MeanFilter<25> meanFilter_yaw;
    static MeanFilter<10> meanFilter_cf;
    uint8_t crcRes = crc16(_recData, sizeof(Send2ControllerStructdef));
    if (crcRes == 0 && _recData->id == SEND_2_CTRL_ID)
    {
        LinkCount = 0;
        continueError = 0;
        packetLossRate--;
        float wheelRPM[2];

        for (int i = 0; i < 2; i++)
        {
            wheelRPM[i] = _recData->wheel_speed[i] / COM_9025_SPEED_RATIO;
            wheelRPM[i] = meanFilter[i].f(wheelRPM[i]);
        }

        rec_data.linerSpeed = (wheelRPM[R] + wheelRPM[L]) / 2.0f * ratio_rpm_to_rps * 2 * PI * wheelRadius;

        rec_data.chassis_state = _recData->chassis_state;
        rec_data.target_speed_y = _recData->target_speed_y / COM_SPEED_Y_RATIO;
        rec_data.target_speed_z = meanFilter_yaw.f(_recData->target_speed_z / COM_SPEED_Z_RATIO);
        
        rec_data.target_height = _recData->target_height / COM_HEIGHT_RATIO;

        rec_data.target_height = upper::constrain(rec_data.target_height, 0.15f, 0.40f);

        rec_data.power_limit_scale = (float)_recData->power_scale / 1000.f;
		rec_data.power_limit_scale = 1;

        rec_data.centripetal_force_scale = meanFilter_cf.f(pow(rec_data.linerSpeed, 2) / (trackWidth / (abs(abs(wheelRPM[R]) - abs(wheelRPM[L])) / 2.0f * ratio_rpm_to_rps * 2 * PI * wheelRadius)));

        if (rec_data.chassis_state.gg_flag)
        {
            __set_FAULTMASK(1);
            NVIC_SystemReset();
        }
    }
    else
    {
        continueError++;  //串口数据有误连续错误自增
        packetLossRate++; //串口数据有误丢包率增加
        crcErrCnt++;
    }
    packetLossRate = upper::constrain(packetLossRate, 0, 500);
    continueError = upper::constrain(continueError, 0, 255);
}
