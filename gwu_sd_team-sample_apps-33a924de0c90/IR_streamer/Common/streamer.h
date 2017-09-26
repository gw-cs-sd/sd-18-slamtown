//============================================================================
// Name        : streamer.h
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Main types and defines for the streamer application
//============================================================================

#ifndef _STREAMER_H_
#define _STREAMER_H_

// Client message
enum STREAMER_MSG { FRAME_REQUEST, // Request a frame [IN/OUT]
                      I2C_CMD,      // Sends an I2C command [IN/OUT]
                      UNKNOWN_MSG    // Response msg for an unknown [OUT]
                      };

// Server response
enum SERVER_RESP { FRAME_READY,   // The response contains a valid frame
                     NO_FRAME,      // No frame ready to be read [We can avoid this by repeating last frame]
                     I2C_SUCCEED,   // I2C command was applied with success
                     I2C_FAILED,    // I2C command failed
                     RESEND         // If the client message was something unknown, ask for a resend
                     };

// I2C commands
enum I2C_CMD { RESET,          // Sensor reset
                FFC,            // Flat Field Correction
                SENSOR_TEMP_K,  // Get sensor internal temperature in kelvin
                SHUTTER_OPEN,   // Open camera shutter
                SHUTTER_CLOSE,  // Close camera shutter
                VOID            // No command (just for frame request msg)
                };

// Frame type
enum FRAME_TYPE { U8,    // 8 bit per pixel
                    U16    // 16 bit per pixel
                    };

#endif
