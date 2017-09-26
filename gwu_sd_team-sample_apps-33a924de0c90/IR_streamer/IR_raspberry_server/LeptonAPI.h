//============================================================================
// Name        : LeptonAPI.h
// Author      : CosmaC
// Version     : V2.0
// Copyright   : GWU Research
// Description : Custom Lepton sensor API. Support Lepton v2 and v3.
//============================================================================

#ifndef __LEPTON_API__
#define __LEPTON_API__

// C/C++ Tools
#include <ctime>
#include <stdint.h>

// User defines
#define LEPTON_V3
#define LEPTON_DEBUG //printf
#define PACKET_SIZE         164                                     // SPI packet size in bytes
#define PACKET_SIZE_UINT16  (PACKET_SIZE/2)                         // SPI packet size in words
#define PACKETS_PER_FRAME   60                                      // SPI packets per segment
#define FRAME_SIZE_UINT16   (PACKET_SIZE_UINT16 * PACKETS_PER_FRAME)// segment size in words
#define FRAME_SIZE          (PACKET_SIZE * PACKETS_PER_FRAME)       // segment size in bytes
#define PACKETS_PER_READ    5                                       // number of packets read in on SPI read call
#define RESET_WAIT_TIME     1000                                    // wait time in microseconds for each reset

#ifdef LEPTON_V3
#define NUM_SEGMENTS                4                               // number of segments per frame
#define SEGMENT_NUMBER_PACKET_INDEX 20                              // packet index for the packet containing the sgment number
#define SPI_SPEED                   32000000                        // SPI speed 16 MHz
#define FRAME_WIDTH                 160                             // IR frame width
#define FRAME_HEIGHT                120                             // IR frame height
#else // Lepton V2
#define NUM_SEGMENTS                1
#define SEGMENT_NUMBER_PACKET_INDEX 0
#define SPI_SPEED                   10000000 //10MHz
#define FRAME_WIDTH                 80                              // IR frame width
#define FRAME_HEIGHT                60                              // IR frame height
#endif

// Custom Lepton API
int leptonOpenConnection();                            // Open communication with Lepton sensor
int leptonCloseConnection();                           // Close communication with Lepton sensor
int leptonGetFrame( char *frame, int pixelDepth = 8);  // Get a frame from sensor
int leptonCommand(char *frame, char msg[2]);           // Run I2C command

#endif
