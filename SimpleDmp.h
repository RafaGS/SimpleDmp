/*
    SimpleDmp.h - Clase para lectura de valores DMP de IMU MPU6050
    Created by Rafa Gomez, 2014
    Licensed under GPL v3

    http://minibots.wordpress.com/2014/12/16/libreria-para-lectura-de-angulos-de-inclinacion-de-un-mpu-6050-con-dmp/

    Parts from I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
    (MPU6050_9Axis_MotionApps41.h) by Jeff Rowberg
*/


#ifndef SimpleDmp_h_
#define SimpleDmp_h_

// Defines incluidos en MPU6050_9Axis_MotionApps41.h
//
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]
//
// Fin de defines incluidos en MPU6050_9Axis_MotionApps41.h

#include <Arduino.h>
#include "helper_3dmath.h"
#include "MPU6050.h"
#include "Wire.h"


class SimpleDmp : public MPU6050 { 
    private:
        bool _dmpReady;  // set true if DMP init was successful
        uint8_t _mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t _devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t _packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t _fifoCount;     // count of all bytes currently in FIFO
        uint8_t _fifoBuffer[64]; // FIFO storage buffer

        Quaternion _q;           // [w, x, y, z]         quaternion container
        VectorFloat _gravity;    // [x, y, z]            gravity vector
        float _ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        void dmpDataReady();
        static SimpleDmp* sDmp;
        static void dmpDataReadyISR();

        volatile bool _mpuInterrupt;

        // Funciones definidas en MPU6050_9Axis_MotionApps41.h
        //
        uint8_t dmpInitialize();
        uint16_t dmpGetFIFOPacketSize();

        uint8_t dmpGetQuaternion(int32_t *data, const uint8_t* packet);
        uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet);
        uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet);

        uint8_t dmpGetGravity(VectorFloat *v, Quaternion *q);
        uint8_t dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
        //
        // Fin de funciones definidas en MPU6050_9Axis_MotionApps41.h

    public:
        SimpleDmp();
        bool initDmp(int xg=3, int yg=-7, int zg=13, int xa=-4452, int ya=-804, int za=1081);
        void readMPU();
        void getAngles(float &yaw, float &pitch, float &roll);
};

#endif	//SimpleDmp.h
