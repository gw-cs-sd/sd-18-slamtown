//============================================================================
// Name        : LeptonUtils.h
// Author      : CosmaC
// Version     : V2.0
// Copyright   : GWU Research
// Description : Lepton util Functions (low level communication with sensor) 
//============================================================================

#ifndef __LEPTON_UTILS__
#define __LEPTON_UTILS__

// C/C++ Tools
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

// Coloring scheme
extern const int colormap_rainbow[];
extern const int colormap_grayscale[];
extern const int colormap_ironblack[];


// SPI Config
extern int spi_cs0_fd;
extern int spi_cs1_fd;


// SPI utlis function
int leptonSPI_OpenPort(int spi_device);
int leptonSPI_ClosePort(int spi_device);


// I2C utils functions
int leptonI2C_connect();         // Sets I2C communication parameters
void leptonI2C_ShutterManual();  // Sets shutter mode to manual
void leptonI2C_ShutterOpen();    // Open shutter
void leptonI2C_FFC();            // Perform flat field correction
void leptonI2C_ShutterClose();   // Close shutter
int leptonI2C_InternalTemp();    // Get internal temperature



#endif
