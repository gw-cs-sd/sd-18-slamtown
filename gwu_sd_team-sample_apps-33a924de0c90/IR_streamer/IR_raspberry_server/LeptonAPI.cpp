//============================================================================
// Name        : LeptonAPI.cpp
// Author      : CosmaC
// Version     : V2.0
// Copyright   : GWU Research
// Description : Simple app for communicating with the Lepton sensor(Custom API)
//============================================================================

// C/C++ Tools
#include <iostream>

// Local Modules
#include "LeptonAPI.h"
#include "LeptonUtils.h"
#include "streamer.h"

// Frame buffers
uint8_t result[FRAME_SIZE * NUM_SEGMENTS];
static int count = 0;

//============================================================================
// Open communication with Lepton
//============================================================================
int leptonOpenConnection()
{
	// Open spi port
	leptonSPI_OpenPort(0);

	// Open I2C
	leptonI2C_connect();

	return 0;
}


//============================================================================
// Close communication with Lepton
//============================================================================
int leptonCloseConnection()
{
	// Close SPI port
	leptonSPI_ClosePort(0);

	return 0;
}


//============================================================================
// Lepton convert frame from sensor to IR image
//============================================================================
void leptonCvtFrame8 ( uint8_t *frame )
{
	uint16_t *frameBuffer = (uint16_t *)result;
	uint16_t value, temp;
	uint16_t minValue = 65535;
	uint16_t maxValue = 0;

	// Compute min and max
	for(int i = 0; i < NUM_SEGMENTS * FRAME_SIZE_UINT16; i++) 
	{
		// Skip the first 2 uint16_t's of every packet, they're 4 header bytes
		if(i % PACKET_SIZE_UINT16 < 2) 
			continue;
			
		// Flip the MSB and LSB at the last second
		temp = result[i*2];
		result[i*2] = result[i*2+1];
		result[i*2+1] = temp;
		
		value = frameBuffer[i];
		if(value > maxValue) 
			maxValue = value;
		if(value < minValue) 
			minValue = value;
	}

	// Scale frame range
	float diff = maxValue - minValue;
	float scale = 255.f / diff;
	int idx = 0;
	LEPTON_DEBUG("[%d]Min: %d Max: %d \n", count, minValue, maxValue);
	for(int i = 0; i < NUM_SEGMENTS * FRAME_SIZE_UINT16; i++) 
	{
		if(i % PACKET_SIZE_UINT16 < 2) 
			continue;

		frame[idx++] = (uint8_t)((frameBuffer[i] - minValue) * scale);
	}
}

void leptonCvtFrame16 ( uint16_t *frame )
{
	uint8_t *dst = (uint8_t *)frame;
	int idx = 0;
	for(int i = 0; i < NUM_SEGMENTS*FRAME_SIZE; i+=2) {
		
		// Skip the first 2 uint16_t's of every packet, they're 4 header bytes
		if(i % PACKET_SIZE < 4) 
			continue;

		// Flip the MSB and LSB at the last second
		dst[idx++] = result[i+1];
		dst[idx++] = result[i];
	}
}


//============================================================================
// Lepton read frame from sensor
//============================================================================
int leptonReadSegment(const int max_resets, uint8_t* data_buffer)
{
	int resets = -1;
	const int step = PACKETS_PER_READ;
	const int bytes_per_SPI_read = step * PACKET_SIZE;
	const int segmentId_packet_idx = SEGMENT_NUMBER_PACKET_INDEX * PACKET_SIZE; // 20th packet will tell you the segment number
	//const int step_minus_1 = step - 1;
	for(int j = 0; j < PACKETS_PER_FRAME; j+=step) 
	{
		if (j == 0) {
			// reach sync
			uint8_t packetNumber = 255;
			uint8_t discard_packet = 0x0F;
			while (packetNumber != 0 || discard_packet == 0x0F) { // while packet id is not 0, or the packet is a discard packet
				++resets;
				if(resets == max_resets) {
					return resets;
				}

				usleep(RESET_WAIT_TIME);
				read(spi_cs0_fd, data_buffer, PACKET_SIZE);
				packetNumber = data_buffer[1];
				discard_packet = data_buffer[0] & 0x0F;
			}
			read(spi_cs0_fd, data_buffer + PACKET_SIZE, (step - 1) * PACKET_SIZE);
			continue;
		}

		// Checks the need to reset SPI connection
		if(resets == max_resets) {
			return resets;
		}

		// Read a packet
		read(spi_cs0_fd, data_buffer + j * PACKET_SIZE, bytes_per_SPI_read);
	
		// Checks discard packet
		uint8_t discard_packet = data_buffer[j * PACKET_SIZE] & 0x0F;
		if (discard_packet == 0x0F) {
		    usleep(RESET_WAIT_TIME);
		    ++resets;
		    j = -step;
		    continue;
		}
		
		// Checks first packet id
		uint8_t packetNumber = data_buffer[j * PACKET_SIZE + 1];
		if (packetNumber != j) {
			LEPTON_DEBUG("Packet %d \n", j);
			usleep(RESET_WAIT_TIME);
			++resets;
			j = -step; // reset just the segment
			continue;
		}
		
		// Checks last packet id
		/*int last_idx_in_packet = j + step_minus_1;
		uint8_t packetNumber_last = data_buffer[last_idx_in_packet * PACKET_SIZE + 1];
		if (packetNumber_last != last_idx_in_packet) {
			LEPTON_DEBUG("Packet %d \n", j);
			usleep(RESET_WAIT_TIME);
			++resets;
			j = -step; // reset just the segment
			continue;
		}*/
	}

	return resets;
}

int leptonReadFrame()
{
	// Read data packets from lepton over SPI
	int resets = 0;
	const int segmentId_packet_idx = SEGMENT_NUMBER_PACKET_INDEX * PACKET_SIZE; // 20th packet will tell you the segment number
	const int segment_max_resets = 100;
	for(int16_t segment = 0; segment < NUM_SEGMENTS; ++segment) 
	{
		// Checks the need to reset SPI connection
		if(resets > 400) {
			resets = 0;
			segment = -1;
			leptonCloseConnection();
			usleep(100000);  // 100 ms
			leptonOpenConnection();
			continue;
		}

		// Read segment
		uint8_t* data_buffer = result + segment * FRAME_SIZE;
		int new_resets = leptonReadSegment(segment_max_resets, data_buffer);
		if (new_resets == segment_max_resets) {
			resets += new_resets;	
			segment = -1;
			continue;
		}
		resets += new_resets;

		#ifdef LEPTON_V3
		// Checks segment number
		int16_t segmentNumber = (data_buffer[segmentId_packet_idx] >> 4) - 1;
		if (segmentNumber != segment) {
			LEPTON_DEBUG("Segment %d \n", segment);
			++resets;
			segment = -1; // reset all segments
			continue;
		}
		#endif
	}

	return 0;
}


//============================================================================
// Lepton get IR frame from sensor
//============================================================================
int leptonGetFrame( char *frame, int pixelDepth )
{
	// Read data packets from lepton over SPI
	leptonReadFrame();

	// Read sensor temperature
	int temp = leptonI2C_InternalTemp();
	((int *)(frame))[0] = temp;

	// Convert Lepton frame to IR frame
	char *frame_start = frame + 4; // skip sensor temperature
	if (pixelDepth == 8)
		leptonCvtFrame8 ((uint8_t*)frame_start);
	else
		leptonCvtFrame16 ((uint16_t*)frame_start);
	count++;

	return 0;
}


//============================================================================
// Lepton I2C command
//============================================================================
int leptonCommand(char *frame, char msg[2])
{
	// Run FFC
	if (msg[1] == FFC)
	{
		leptonI2C_FFC();
		frame[1] = I2C_SUCCEED;
	}
	// Get sensor temp in K
	else if(msg[1] == SENSOR_TEMP_K)
	{
		int temp = leptonI2C_InternalTemp();
		frame[1] = I2C_SUCCEED;
		int *frame_int = (int *)(frame+2);
		frame_int[0] = temp;
	}
	// Open camera shutter
	else if(msg[1] == SHUTTER_OPEN) {
		leptonI2C_ShutterOpen();
		frame[1] = I2C_SUCCEED;
	}
	// Close camera shutter
	else if(msg[1] == SHUTTER_CLOSE) {
		leptonI2C_ShutterClose();
		frame[1] = I2C_SUCCEED;
	}
	// Unknown I2C command
	else
	{
		frame[1] = I2C_FAILED;
	}

	return 0;
}
