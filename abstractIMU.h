/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file abstractIMU.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 抽象IMU库，用于对接顶层认为定义坐标系，与imu的实际坐标系之间的数据转化
 *        方便将仿真用的理想顶层，对接到实车
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
#ifndef ABSTRACTIMU_H
#define ABSTRACTIMU_H

#define USE_LPMS_BE2 1

#if USE_LPMS_BE2
#include <LPMS_BE2.h>
#endif

/*线性参数*/
struct LinearDataStructdef
{
    float x;
    float y;
    float z;
};
/*角度参数*/
struct AngularDataStructdef
{
    float pitch;
    float yaw;
    float roll;
};

namespace abstractIMU
{
    /**
     * @brief 线性加速度极性结构体
     *
     */
    struct linerPolarityStruct
    {
        int8_t x = 1;
        int8_t y = 1;
        int8_t z = 1;
    };

    /**
     * @brief 角度极性结构体
     *
     */
    struct anglePolarityStruct
    {
        int8_t pitch = 1;
        int8_t roll = 1;
        int8_t yaw = 1;
    };
    /**
     * @brief 抽象加速度坐标结构体
     *
     */
    struct abstractAccCoordinateStruct
    {
        uint8_t x = 0;
        uint8_t y = 1;
        uint8_t z = 2;
    };
    /**
     * @brief 抽象欧拉角坐标结构体
     *
     */
    struct abstractEulerCoordinateStruct
    {
        uint8_t Pitch = 0;
        uint8_t Yaw = 1;
        uint8_t Roll = 2;
    };
    /**
     * @brief IMU欧拉角世界（现实）坐标系枚举
     *
     */
    enum imuEulerCoordinate
    {
        imuWorldPitch = 0,
        imuWorldYaw = 1,
        imuWorldRoll = 2
    };
    /**
     * @brief IMU直角坐标系世界（现实）坐标系枚举
     *
     */
    enum imuAccCoordinate
    {
        imuWorldAccX = 0,
        imuWorldAccY = 1,
        imuWorldAccZ = 2
    };
    /**
     * @brief IMU抽象类基类，完成了绝大部分的逻辑操作
     * 在继承类中，加入imu类的指针成员，绑定imu变量，定义9个获取imu数据的纯虚函数，即可正常使用
     */
    class abstractIMUBaseClassdef
    {
    private:
        LinearDataStructdef accData;
        AngularDataStructdef eularData, angleVelData;

        // 抽象坐标系结构体，用于存储人定坐标系与imu坐标系的对应关系
        abstractAccCoordinateStruct abstractAccData;
        abstractEulerCoordinateStruct abstractEularData;

        // 获取imu在世界（现实）坐标系下的加速度
        inline virtual float getACCX() = 0;
        inline virtual float getACCY() = 0;
        inline virtual float getACCZ() = 0;

        // 获取imu在世界（现实）坐标系下的欧拉角、及对应的角速度
        inline virtual float getPitch() = 0;
        inline virtual float getYaw() = 0;
        inline virtual float getRoll() = 0;

        inline virtual float getPitchVel() = 0;
        inline virtual float getYawVel() = 0;
        inline virtual float getRollVel() = 0;

        inline virtual bool isInit() = 0; // 检测是否导入了imu指针，防止未导入时，因为指针为空指针，调用函数而进入硬件中断
    public:
        // 加速度极性，传入1或-1，用于将imu现实的极性对齐到人为定义的坐标系
        linerPolarityStruct accPolarity;
        // 欧拉角极性，传入1或-1，用于将imu现实的极性对齐到人为定义的坐标系
        anglePolarityStruct eularPolarity;
        // 欧拉角基准值，用于校准，会在校准极性后，加上此基准值
        AngularDataStructdef eularBaseData = {0, 0, 0};
        float angle_unit_convert = 1;

        // 绑定加速度坐标系，传入三个枚举值，分别代表x,y,z轴对应的imu坐标系
        // 若传入imuWorldAccY,imuWorldAccX,imuWorldAccZ
        // 则代表将imu的y轴对应到人为定义的x轴，imu的x轴对应到人为定义的y轴，imu的z轴对应到人为定义的z轴
        inline void bindEulerCoordinate(imuEulerCoordinate abstractPitch, imuEulerCoordinate abstractYaw, imuEulerCoordinate abstractRoll)
        {
            abstractEularData.Pitch = abstractPitch;
            abstractEularData.Yaw = abstractYaw;
            abstractEularData.Roll = abstractRoll;
        }

        // 绑定加速度坐标系，传入三个枚举值，分别代表pitch, yaw, roll轴对应的imu坐标系
        // 若传入imuWorldPitch,imuWorldRoll,imuWorldYaw
        // 则代表将imu的pitch轴对应到人为定义的pitch轴，imu的Roll轴对应到人为定义的yaw轴，imu的Yaw轴对应到人为定义的roll轴
        inline void bindAccCoordinate(imuAccCoordinate abstractX, imuAccCoordinate abstractY, imuAccCoordinate abstractZ)
        {
            abstractAccData.x = abstractX;
            abstractAccData.y = abstractY;
            abstractAccData.z = abstractZ;
        }

        // 获取imu世界坐标系数据，转化为人为定义的坐标系、对齐极性后的数据
        // 功能基本实现，继承类不用再写
        inline LinearDataStructdef *getAccData()
        {
            if (isInit() == 0)
                return &accData;
            float tempAccData[3];
            tempAccData[imuWorldAccX] = this->getACCX();
            tempAccData[imuWorldAccY] = this->getACCY();
            tempAccData[imuWorldAccZ] = this->getACCZ();

            accData.x = tempAccData[abstractAccData.x] * accPolarity.x * angle_unit_convert;
            accData.y = tempAccData[abstractAccData.y] * accPolarity.y * angle_unit_convert;
            accData.z = tempAccData[abstractAccData.z] * accPolarity.z * angle_unit_convert;
            return &accData;
        }
        inline AngularDataStructdef *getEularData()
        {
            if (isInit() == 0)
                return &eularData;
            float tempGyroData[3];
            tempGyroData[imuWorldPitch] = this->getPitch();
            tempGyroData[imuWorldYaw] = this->getYaw();
            tempGyroData[imuWorldRoll] = this->getRoll();

            eularData.pitch = tempGyroData[abstractEularData.Pitch] * eularPolarity.pitch * angle_unit_convert + eularBaseData.pitch;
            eularData.yaw = tempGyroData[abstractEularData.Yaw] * eularPolarity.yaw * angle_unit_convert + eularBaseData.yaw;
            eularData.roll = tempGyroData[abstractEularData.Roll] * eularPolarity.roll * angle_unit_convert + eularBaseData.roll;
            return &eularData;
        }
        inline AngularDataStructdef *getAngleVelData()
        {
            if (isInit() == 0)
                return &angleVelData;
            float tempGyroData[3];
            tempGyroData[imuWorldPitch] = this->getPitchVel();
            tempGyroData[imuWorldYaw] = this->getYawVel();
            tempGyroData[imuWorldRoll] = this->getRollVel();

            angleVelData.pitch = tempGyroData[abstractEularData.Pitch] * angle_unit_convert * eularPolarity.pitch;
            angleVelData.yaw = tempGyroData[abstractEularData.Yaw] * angle_unit_convert * eularPolarity.yaw;
            angleVelData.roll = tempGyroData[abstractEularData.Roll] * angle_unit_convert * eularPolarity.roll;
            return &angleVelData;
        }
    };
};
/**
 * @brief IMU抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 模板主类不写实现，是为了避免在传入没有特化过的IMU类型时，会出现无法设想的错误
 * @tparam motorType
 */
template <class IMUtype>
class abstractIMUClassdef
{
};

#if USE_LPMS_BE2
/**
 * @brief 特化模板类，用于阿路比imu
 */
template <>
class abstractIMUClassdef<LPMS_BE2_Typedef> : public abstractIMU::abstractIMUBaseClassdef
{
private:
    LPMS_BE2_Typedef *lpms = nullptr;

    inline virtual float getACCX()
    {
        return lpms->get_data().linearAccX;
    }
    inline virtual float getACCY()
    {
        return lpms->get_data().linearAccY;
    }
    inline virtual float getACCZ()
    {
        return lpms->get_data().linearAccZ;
    }

    inline virtual float getPitch()
    {
        return lpms->get_data().Euler_Pitch;
    }
    inline virtual float getYaw()
    {
        return lpms->get_data().Euler_Yaw;
    }
    inline virtual float getRoll()
    {
        return lpms->get_data().Euler_Roll;
    }

    /* 对于阿路比imu */
    /* pitch绑定caliGyroY */
    /* yaw绑定caliGyroZ */
    /* roll绑定caliGyroX */
    inline virtual float getPitchVel()
    {
        return lpms->get_data().caliGyroY;
    }
    inline virtual float getYawVel()
    {
        return lpms->get_data().caliGyroZ;
    }
    inline virtual float getRollVel()
    {
        return lpms->get_data().caliGyroX;
    }

    inline virtual bool isInit()
    {
        if (lpms == nullptr)
            return 0;
        else
            return 1;
    }

public:
    inline void bindIMU(LPMS_BE2_Typedef *_lpms)
    {
        this->lpms = _lpms;
    }
    inline void update(uint8_t *data)
    {
        if (isInit() == 0)
            return;
        lpms->LPMS_BE2_Get_Data(data);
    }
};
#endif

#endif
