#ifndef MLX90621_H
#define MLX90621_H

#include <stdio.h>
#include <stdlib.h>

typedef unsigned char byte;

// Debug options
#define PINFO

// General
const int    NB_PIXELS   = 64;
const int    FRAME_SIZE  = 2 * NB_PIXELS;
const float  WRONG_T     = 9999.f;
const float  NORMAL_TA_L = -1.f; // normal low sensor case temperature (C)
const float  NORMAL_TA_H = 50.f; // normal high sensor case temperature (C)
                                  // these two values are normal for my experiments

// RAM Address map
const byte   RAM_I2C_ADDRESS       = 0x60;
const byte   RAM_READ_CMD          = 0x02;
const byte   RAM_WRITE_CFG_CMD     = 0x03;
const byte   RAM_WRITE_TRIM_CMD    = 0x04;
const byte   RAM_IR_DATA_ADDRESS   = 0x00;
const byte   RAM_PTAT_ADDRESS      = 0x40;
const byte   RAM_CP_ADDRESS        = 0x41;
const byte   RAM_CFG_ADDRESS       = 0x92;
const byte   RAM_TRIM_ADDRESS      = 0x93;

const byte   RAM_CFG_POR_MASK      = 0x04;
const byte   RAM_CFG_REFRESH_512HZ = 0x00;
const byte   RAM_CFG_REFRESH_256HZ = 0x06;
const byte   RAM_CFG_REFRESH_128HZ = 0x07;
const byte   RAM_CFG_REFRESH_64HZ  = 0x08;
const byte   RAM_CFG_REFRESH_32HZ  = 0x09;
const byte   RAM_CFG_REFRESH_16HZ  = 0x0A;
const byte   RAM_CFG_REFRESH_8HZ   = 0x0B;
const byte   RAM_CFG_REFRESH_4HZ   = 0x0C;
const byte   RAM_CFG_REFRESH_2HZ   = 0x0D;
const byte   RAM_CFG_REFRESH_1HZ   = 0x0E;
const byte   RAM_CFG_REFRESH_0HZ   = 0x0F; // 0.5HZ

// EEPROM address map
const int    EEPROM_SIZE           = 256;
const byte   EEPROM_I2C_ADDRESS    = 0x50;
const byte   EEPROM_READ_CMD       = 0x00;

const byte   EEPROM_RESOLUTION     = 0xF5;
const byte   EEPROM_RESOLUTION_MASK= 0x30;

// EEEPROM Ta related registers
const byte   EEPROM_VTH_L        = 0xDA;
const byte   EEPROM_VTH_H        = 0xDB;
const byte   EEPROM_KT1_L        = 0xDC;
const byte   EEPROM_KT1_H        = 0xDD;
const byte   EEPROM_KT2_L        = 0xDE;
const byte   EEPROM_KT2_H        = 0xDF;
const byte   EEPROM_KT_SCALE     = 0xD2;

// EEEPROM To related registers
const byte   EEPROM_BI_START     = 0x40;
const byte   EEPROM_dALPHA_START = 0x80;
const byte   EEPROM_A_COMMON_L   = 0xD0;
const byte   EEPROM_A_COMMON_H   = 0xD1;
const byte   EEPROM_A_CP_L       = 0xD3;
const byte   EEPROM_A_CP_H       = 0xD4;
const byte   EEPROM_B_CP         = 0xD5;
const byte   EEPROM_ALPHA_CP_L   = 0xD6;
const byte   EEPROM_ALPHA_CP_H   = 0xD7;
const byte   EEPROM_TGC          = 0xD8;
const byte   EEPROM_A_B_SCALING  = 0xD9;
const byte   EEPROM_ALPHA0_L     = 0xE0;
const byte   EEPROM_ALPHA0_H     = 0xE1;
const byte   EEPROM_ALPHA0_SCALE = 0xE2;
const byte   EEPROM_dALPHA_SCALE = 0xE3;
const byte   EEPROM_EMISIVITY_L  = 0xE4;
const byte   EEPROM_EMISIVITY_H  = 0xE5;
const byte   EEPROM_KSTA_L       = 0xE6;
const byte   EEPROM_KSTA_H       = 0xE7;
const byte   EEPROM_CFG_REG_L    = 0xF5;
const byte   EEPROM_CFG_REG_H    = 0xF6;
const byte   EEPROM_TRIM_VALUE   = 0xF7;


struct mlx_consts {
    short resolution; // ADC resolution
    float ta0;        // Ambient calibration reference temperature
    float alpha0;     // Common sensitivity coefficient of IR pixels
    float alpha_cp;   // Sensitivity coefficient of the compensation pixel
    short delta_alpha_scale; // Scaling coefficient for individual sensitivity
    float KsTa;       // Ta dependece of sensivity coefficient 
    float tgc;        // Thermal Gradient Coefficient
    short a_common;   // IR pixel common offser coeficient
    short dai_scale;  // Scale coefficient for individual sensivity
    short bi_scale;   // Scaling coefficient for slope of IR pixels offset
    float a_cp;       // Compensation pixel individual offset coefficients
    float b_cp;       // Individual Ta dependence (slope) of the compensation pixel offset
    float emisivity;  // surface emisivity
    short vcp;        // compensation pixel
};

class MLX90621 {

public:
    bool Init();
    float GetTo();
    bool Deinit();
    
private:
    // Read from IR sensor's registers
    bool ReadEEPROM();
    bool ReadFrame();
    bool ReadConstCoeff();
    bool ReadConfig(byte *lsb, byte *msb);
    bool ReadTrim(byte *lsb, byte *msb);
    bool ReadPTAT(short *ptat);
    bool ReadCP(short *cp);
    bool CheckPOR();

    // Write IR sensor's registers
    bool WriteConfig(byte lsb, byte msb);
    bool WriteTrim(byte lsb, byte msb);
    bool WriteRefreshRate(int hz);

    // Computation of Ta and To
    float ComputeTa();
    float ComputeTo();
    float ComputeToWithRead();

    // MLX class attributes
    byte EEPROM[EEPROM_SIZE];   // EEPROM map buffer
    byte ir_pixels[FRAME_SIZE]; // IR frame raw buffer
    float frame_temp[NB_PIXELS];// IR frame temperature buffer
    float Ta;                   // Sensor case temperature (C)
    float To;                   // Object tempearture (C)
    mlx_consts kmlx;		// MLX90621 sensor constants
};

#endif