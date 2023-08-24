/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file abstractMotor.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 抽象电机库，用于对接顶层计算得控制量与底层电机的实际输出
 *        方便将仿真用的理想顶层，对接到实车
 *        目前支持电机：MF9025_v2、HT04、2006（C610）、3508（C620）、GM6020
 * @version 1.0
 * @date 2023-03-04
 *
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _ABSTRACTMOTOR_H_
#define _ABSTRACTMOTOR_H_

#define USE_DJI_MOTOR 0
#define USE_MF9025 1
#define USE_HT04 1

#if USE_DJI_MOTOR
#include "SRML.h"
#endif

#if USE_HT04
#include "HT04.h"
#endif

#if USE_MF9025
#include <MF9025_v2.h>
#endif

/* 匿名命名空间，使基类无法被外部链接 */
namespace
{
    /**
     * @brief 电机抽象基类，用于对接顶层计算得控制量与底层电机的实际输出
     * 在继承类中，加入电机类的指针成员，绑定电机变量，定义几个纯虚函数，即可正常使用
     */
    template <class motorType>
    class abstractMotorBase
    {
    protected:
        //通过纯虚函数的形式，强制要求每个电机都二次封装对应的api
        inline virtual float getRawMotorTotalAngle() = 0;//获取电机原生多圈角度数据，单位为degree
        inline virtual float getRawMotorAngle() = 0;//获取电机原生单圈角度数据，单位为degree
        inline virtual float getRawMotorSpeed() = 0;//获取电机原生速度数据，单位RPM
        inline virtual void setRawMotorCurrentOut(float out) = 0;//下发电机实际电流输出
        inline virtual bool isInit()
        {
            if(this->motor == nullptr)
                return 0;
            else
                return 1;
        }; // 检测是否导入了电机指针、队列句柄等等，防止未导入时，因为指针为空指针，调用函数而进入硬件中断

    public:
        motorType *motor = nullptr;
        inline void bindMotor(motorType *_motor) { motor = _motor; }//绑定电机指针
        //提供给顶层的接口，将电机原生多圈角度进行单位转换、极性对齐、基础角度偏移，再返回给顶层
        inline virtual float getMotorTotalAngle()
        {
            if (isInit() == 0)
                return 0;
            return getRawMotorTotalAngle() * angle_unit_convert * Polarity + baseAngle;
        }
        //提供给顶层的接口，将电机原生单圈角度进行单位转换、极性对齐、基础角度偏移，再返回给顶层
        inline virtual float getMotorAngle()
        {
            if (isInit() == 0)
                return 0;
            return getRawMotorAngle() * angle_unit_convert * Polarity + baseAngle;
        }
        //提供给顶层的接口，将电机原生速度进行单位转换、极性对齐，再返回给顶层
        inline virtual float getMotorSpeed()
        {
            if (isInit() == 0)
                return 0;
            return getRawMotorSpeed() * speed_unit_convert * Polarity;
        }
        //提供给顶层的接口，将顶层下发的力矩，进行单位转换、极性修正，再下发到底层
        inline virtual void setMotorCurrentOut(float out)
        {
            if (isInit() == 0)
                return;
            setRawMotorCurrentOut(out * out_unit_convert * Polarity);
        }

        // 极性，将电机实际转动方向的正负，对齐到人为设置的方向，自行设置
        // 对于绝大部分电机来说，输出正力矩时，速度增加，角度增加，所以三者极性是具有统一性的
        // 因此，只需要设置一个极性即可
        float Polarity = 1;

        float speed_unit_convert = 1; // 获取数据的单位转换，自行设置

        float angle_unit_convert = 1; // 获取数据的单位转换，自行设置
        float baseAngle = 0;          // 角度基础值，会在极性对齐、单位转换后，再加上这个值

        // 如果需要将顶层下发的力矩，转换为电流，再用电机类下发，就需要设置这个值
        // 如果不需要单位转换，就不用管
        float out_unit_convert = 1;
    };
}

/**
 * @brief 电机抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 用模板是为了类名统一，实现泛化
 * 模板主类不写实现，是为了避免在传入没有特化过的电机类型时，会出现无法设想的错误
 * @tparam motorType
 */
template <class motorType>
class abstractMotor
{
};

#if USE_HT04
/**
 * @brief HT04电机抽象类
 */
template <>
class abstractMotor<MotorHT04Classdef> : public abstractMotorBase<MotorHT04Classdef>
{
protected:
    CAN_COB Tx_Buff;
    QueueHandle_t CAN_TxPort = nullptr;
    inline virtual float getRawMotorTotalAngle() { return 0; }
    inline virtual float getRawMotorAngle()
    {
        return motor->getRecData().position;
    }
    inline virtual float getRawMotorSpeed()
    {
        return motor->getRecData().velocity;
    }
    inline virtual float getRawMotorTorque()
    {
        return motor->getRecData().tau;
    }
    inline virtual void setRawMotorCurrentOut(float out)
    {
        motor->setTorque(out);
        MotorMsgPack(Tx_Buff, *motor);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    inline virtual void setRawMotorAngle(float angle, float kp, float kd)
    {
        motor->setPosition(angle, kp, kd);
        MotorMsgPack(Tx_Buff, *motor);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    inline virtual void setRawMotorSpeed(float speed, float kd, float torque)
    {
        motor->setVelocity(speed, kd, torque);
        MotorMsgPack(Tx_Buff, *motor);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    inline virtual bool isInit() override
    {
        if (CAN_TxPort == nullptr || motor == nullptr)
            return 0;
        else
            return 1;
    }

public:
    float torque_unit_convert = 1;

    inline void init(QueueHandle_t _CAN_TxPort, MotorHT04Classdef *_motor)
    {
        CAN_TxPort = _CAN_TxPort;
        motor = _motor;
        if (isInit() == 0)
            return;
        motor->cmd_motor_mode(Tx_Buff);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    inline virtual void setMotorSpeed(float speed, float kd, float torque)
    {
        if (isInit() == 0)
            return;
        setRawMotorSpeed(speed / speed_unit_convert / Polarity, kd, torque / out_unit_convert / Polarity);
    }
    inline virtual void setMotorAngle(float angle, float kp, float kd)
    {
        if (isInit() == 0)
            return;
        setRawMotorAngle((angle - baseAngle) / angle_unit_convert / Polarity, kp, kd);
    }

    inline virtual float getMotorTorque()
    {
        if (isInit() == 0)
            return 0;
        return getRawMotorTorque() * torque_unit_convert * Polarity;
    }

    inline void setZeroPosition()
    {
        if (isInit() == 0)
            return;
        motor->cmd_zero_position(Tx_Buff);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }

    inline void motorCheckLink()
    {
        if (isInit() == 0)
            return;
        motor->checkLink();
    }

    inline uint16_t getLinkCount()
    {
        if (isInit() == 0)
            return 0;
        return motor->getLinkCount();
    }
};
#endif

#if USE_MF9025
/**
 * @brief MF9025_v2电机抽象类
 */
template <>
class abstractMotor<MotorMF9025v2Classdef> : public abstractMotorBase<MotorMF9025v2Classdef>
{
protected:
    inline virtual float getRawMotorTotalAngle()
    {
        return motor->getData().totalAngleLocal;
    }
    inline virtual float getRawMotorAngle()
    {
        return motor->getData().singleAngle;
    }
    inline virtual float getRawMotorSpeed()
    {
        return motor->getData().speed;
    }
    inline virtual void setRawMotorCurrentOut(float out)
    {
        motor->iqCloseControl_Current(out);
    }
};
#endif

#if USE_DJI_MOTOR
template <>
class abstractMotor<Motor_C620> : public abstractMotorBase<Motor_C620>
{
public:
    inline bool CheckID(uint32_t id)
    {
        if (isInit() == 0)
            return 0;
        return motor->CheckID(id);
    }
    inline void update(uint8_t can_rx_data[])
    {
        if (isInit() == 0)
            return;
        motor->update(can_rx_data);
    }

protected:
    inline virtual float getRawMotorTotalAngle()
    {
        return motor->getAngle();
    };
    inline virtual float getRawMotorAngle()
    {
        return motor->getEncoder() / 8192.f * 360;
    };
    inline virtual float getRawMotorSpeed()
    {
        return motor->getSpeed();
    };
    inline virtual void setRawMotorCurrentOut(float out)
    {
        motor->Out = out;
    };
};

template <>
class abstractMotor<Motor_C610> : public abstractMotorBase<Motor_C610>
{
public:
    inline bool CheckID(uint32_t id)
    {
        if (isInit() == 0)
            return 0;
        return motor->CheckID(id);
    }
    inline void update(uint8_t can_rx_data[])
    {
        if (isInit() == 0)
            return;
        motor->update(can_rx_data);
    }

protected:
    inline virtual float getRawMotorTotalAngle()
    {
        return motor->getAngle();
    }
    inline virtual float getRawMotorAngle()
    {
        return motor->getEncoder() / 8192.f * 360;
    }
    inline virtual float getRawMotorSpeed()
    {
        return motor->getSpeed();
    }
    inline virtual void setRawMotorCurrentOut(float out)
    {
        motor->Out = out;
    }
};

template <>
class abstractMotor<Motor_GM6020> : public abstractMotorBase<Motor_GM6020>
{
protected:
    myPID current_loop;
    float current_loop_feedback = 55;
    float TorqueRatio = 13495.277;
    inline float current_loop_adjust(float target_current)
    {
        current_loop.Target = target_current;
        current_loop.Current = motor->getGivenCurrent();
        return current_loop.Adjust() + (current_loop_feedback * motor->getSpeed() + current_loop.Target)/ 0.75f;
    }

    inline virtual float getRawMotorTotalAngle()
    {
        return motor->getAngle();
    };
    inline virtual float getRawMotorAngle()
    {
        return motor->getEncoder() / 8192.f * 360;
    };
    inline virtual float getRawMotorSpeed()
    {
        return motor->getSpeed();
    };
    inline virtual void setRawMotorCurrentOut(float out)
    {
        motor->Out = current_loop_adjust(out);
    };
public:
    abstractMotor<Motor_GM6020>() : abMotor::abstractMotorBase<Motor_GM6020>()
    {
        current_loop.SetPIDParam(0.25, 50, 0, 30000, 30000);
        current_loop.I_SeparThresh = 60000;
    }
    inline bool CheckID(uint32_t id)
    {
        if (isInit() == 0)
            return 0;
        return motor->CheckID(id);
    }
    inline void update(uint8_t can_rx_data[])
    {
        if (isInit() == 0)
            return;
        motor->update(can_rx_data);
    }
    inline void setEncoderOffset(uint16_t offset)
    {
        if (isInit() == 0)
            return;
        motor->setEncoderOffset(offset);
    }

    inline void setMotorVoltageOut(float out)
    {
        if (isInit() == 0)
            return;
        motor->Out = (out * out_unit_convert * Polarity);
    }

    inline void setMotorTorqueOut(float torque)
    {
        if (isInit() == 0)
            return;
        motor->Out = current_loop_adjust(torque * TorqueRatio * this->Polarity);
    }
};
#endif

#endif
