/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, David Pietrocola
*  Modifies roomba_500_series ROS package by 
*  - Reduces number of applicable opcodes based on iRobot SCI spec
*  - Modifies sensor packet handling based on iRobot SCI spec
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Original Author: Gonçalo Cabrita
* Roomba_400_series Author: David Pietrocola
*********************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include "roomba_400_series/SerialCommInterface.h"
#include <ros/ros.h>

// *****************************************************************************
// Constructor
irobot::SerialCommInterface::SerialCommInterface(const char * new_serial_port)
{	
	port_name_ = new_serial_port;

	OImode_ = OI_MODE_OFF;
	
	this->resetOdometry();
	
	encoder_counts_[LEFT] = -1;
	encoder_counts_[RIGHT] = -1;
	
	last_encoder_counts_[LEFT] = 0;
	last_encoder_counts_[RIGHT] = 0;
	
	num_of_packets_ = 0;
	//sensor_packets_ = 0;
	packets_size_ = 0;
	
	// Default packets
	OI_Packet_ID default_packet = OI_PACKET_GROUP_2;
	this->setSensorPackets(default_packet, OI_PACKET_GROUP_2_SIZE);

	serial_port_ = new cereal::CerealPort();
}


// *****************************************************************************
// Destructor
irobot::SerialCommInterface::~SerialCommInterface()
{
	// Clean up!
	delete serial_port_;
}


// *****************************************************************************
// Open the serial port
int irobot::SerialCommInterface::openSerialPort(bool full_control)
{
	try{ serial_port_->open(port_name_.c_str(), 57600); } //change baud rate from 115200
	catch(cereal::Exception& e){ return(-1); }

	this->startOI(full_control);

	return(0);
}


// *****************************************************************************
// Set the mode
int irobot::SerialCommInterface::startOI(bool full_control)
{	
	char buffer[1];

	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	buffer[0] = (char)OI_OPCODE_START;
	try{ serial_port_->write(buffer, 1); }
	catch(cereal::Exception& e){ return(-1); }
	OImode_ = OI_MODE_PASSIVE;

	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	buffer[0] = (char)OI_OPCODE_CONTROL;
	try{ serial_port_->write(buffer, 1); }
	catch(cereal::Exception& e){ return(-1); }
	OImode_ = OI_MODE_SAFE;
	
	if(full_control)
	{
		usleep(OI_DELAY_MODECHANGE_MS * 1e3);
		buffer[0] = (char)OI_OPCODE_FULL;
		try{ serial_port_->write(buffer, 1); }
		catch(cereal::Exception& e){ return(-1); }
		OImode_ = OI_MODE_FULL;
	}
	return(0);
}


// *****************************************************************************
// Close the serial port
int irobot::SerialCommInterface::closeSerialPort()
{
	this->drive(0.0, 0.0);
	usleep(OI_DELAY_MODECHANGE_MS * 1e3);

	try{ serial_port_->close(); }
	catch(cereal::Exception& e){ return(-1); }

	return(0);
}


// *****************************************************************************
// Send an OP code to the roomba
int irobot::SerialCommInterface::sendOpcode(OI_Opcode code)
{
	char to_send = code;
	try{ serial_port_->write(&to_send, 1); }
	catch(cereal::Exception& e){ return(-1); }
	return(0);
}


// *****************************************************************************
// Power down the roomba
int irobot::SerialCommInterface::powerDown()
{
	return sendOpcode(OI_OPCODE_POWER);
}


// *****************************************************************************
// Set the speeds
int irobot::SerialCommInterface::drive(int linear_speed, int angular_speed)
{
	//convert params to iRobot SCI specs
	/*
	linear_speed *= 1e3;
	if (linear_speed >= 0) linear_speed = MIN(ROOMBA_MAX_LIN_VEL_MM_S, linear_speed);
	else linear_speed = MAX(-ROOMBA_MAX_LIN_VEL_MM_S, linear_speed);

	float radius;
	//check 0 angular speed special cases:
	if (angular_speed == 0) radius = 32768;
	else {

	if (angular_speed >= 0) angular_speed = MIN(ROOMBA_MAX_ANG_VEL_RAD_S, angular_speed);
	else angular_speed = MAX(-ROOMBA_MAX_ANG_VEL_RAD_S, angular_speed);
	}
	float angularVelLeft = linear_speed / (ROOMBA_WHEEL_RADIUS*1e3);
	float angularVelRight = (linear_speed + angular_speed*ROOMBA_AXLE_LENGTH*1e3)/(ROOMBA_WHEEL_RADIUS*1e3);

	radius = (-ROOMBA_AXLE_LENGTH*1e3*0.5)*(angularVelLeft+angularVelRight)/(angularVelLeft-angularVelRight);

	//check turn-in-place special cases:
	if (radius < 10 && radius > -10) {
		if (radius > 0) radius = 1;
		else radius = -1;
	}

	if (radius > 0) radius = MIN(ROOMBA_MAX_RADIUS_MM, radius);
	else radius = MAX(ROOMBA_MAX_RADIUS_MM, radius);
	*/
	ROS_INFO_STREAM("Linear: " << linear_speed << " Angular: " << angular_speed);
	// Compose comand
	char cmd_buffer[5];
	cmd_buffer[0] = (char)OI_OPCODE_DRIVE;
	cmd_buffer[1] = (char)(linear_speed >> 8);
	cmd_buffer[2] = (char)(linear_speed & 0xFF);
	cmd_buffer[3] = (char)(angular_speed >> 8);
	cmd_buffer[4] = (char)(angular_speed & 0xFF);

	try{ serial_port_->write(cmd_buffer, 5); }
	catch(cereal::Exception& e){ return(-1); }

	return(0);


/* SCI DOES NOT HAVE DRIVE_DIRECT OPCODE
	int left_speed_mm_s = (int)((linear_speed-ROOMBA_AXLE_LENGTH*angular_speed/2)*1e3);		// Left wheel velocity in mm/s
	int right_speed_mm_s = (int)((linear_speed+ROOMBA_AXLE_LENGTH*angular_speed/2)*1e3);	// Right wheel velocity in mm/s
	
	return this->driveDirect(left_speed_mm_s, right_speed_mm_s);
*/
}


// *****************************************************************************
// Set the motor speeds
int irobot::SerialCommInterface::driveDirect(int left_speed, int right_speed)
{
	// Limit velocity
	int16_t left_speed_mm_s = MAX(left_speed, -ROOMBA_MAX_LIN_VEL_MM_S);
	left_speed_mm_s = MIN(left_speed, ROOMBA_MAX_LIN_VEL_MM_S);
	int16_t right_speed_mm_s = MAX(right_speed, -ROOMBA_MAX_LIN_VEL_MM_S);
	right_speed_mm_s = MIN(right_speed, ROOMBA_MAX_LIN_VEL_MM_S);
	
	// Compose comand
	char cmd_buffer[5];
	cmd_buffer[0] = (char)OI_OPCODE_DRIVE; //TODO: revise for Drive opcode, is now DriveDirect
	cmd_buffer[1] = (char)(right_speed_mm_s >> 8);
	cmd_buffer[2] = (char)(right_speed_mm_s & 0xFF);
	cmd_buffer[3] = (char)(left_speed_mm_s >> 8);
	cmd_buffer[4] = (char)(left_speed_mm_s & 0xFF);

	try{ serial_port_->write(cmd_buffer, 5); }
	catch(cereal::Exception& e){ return(-1); }

	return(0);
}




// *****************************************************************************
// Set the brushes motors status
int irobot::SerialCommInterface::brushes(unsigned char side_brush, unsigned char vacuum, unsigned char main_brush, unsigned char side_brush_clockwise, unsigned char main_brush_dir)
{
	unsigned char cmd_buffer[2];
	cmd_buffer[0] = OI_OPCODE_MOTORS;
	cmd_buffer[1] = side_brush | vacuum<<1 | main_brush<<2 | side_brush_clockwise<<3 | main_brush_dir<<4;
	
	try{ serial_port_->write((char*)cmd_buffer, 2); }
	catch(cereal::Exception& e){ return(-1); }
	return(0);
}




// *****************************************************************************
// Set the sensors to read
int irobot::SerialCommInterface::setSensorPackets(OI_Packet_ID &new_sensor_packets, size_t new_buffer_size)
{
	/*if(sensor_packets_ == NULL)
	{
		delete sensor_packets_;
	}*/
	
	//num_of_packets_ = new_num_of_packets;
	sensor_packets_ = new_sensor_packets;
	
	stream_defined_ = false;
	packets_size_ = new_buffer_size;
	return(0);
}


// *****************************************************************************
// Read the sensors
int irobot::SerialCommInterface::getSensorPackets(int timeout)
{
	char cmd_buffer[num_of_packets_+2];
	char data_buffer[packets_size_];

	// Fill in the command buffer to send to the robot
	cmd_buffer[0] = (char)OI_OPCODE_SENSORS;			// Sensors command [142]
		
	//cmd_buffer[1] = num_of_packets_;				// Number of packets
	cmd_buffer[1] = sensor_packets_;		// The packet IDs
	
	try{ serial_port_->write(cmd_buffer, 2); }
	catch(cereal::Exception& e){ return(-1); }
	
	try{ serial_port_->readBytes(data_buffer, packets_size_, timeout); }
	catch(cereal::Exception& e){ return(-1); }
	
	return this->parseSensorPackets((unsigned char*)data_buffer, packets_size_);
}



// *****************************************************************************
// Parse sensor data
int irobot::SerialCommInterface::parseSensorPackets(unsigned char * buffer , size_t buffer_lenght)
{	
	if(buffer_lenght != packets_size_)
	{
		// Error wrong packet size
		return(-1);
	}

	int i = 0;
	unsigned int index = 0;
	while(index < packets_size_)
	{
		if(sensor_packets_==OI_PACKET_GROUP_0)		// bytes 0-25
		{
			index += parseBumpersAndWheeldrops(buffer, index);
			index += parseWall(buffer, index);
			index += parseLeftCliff(buffer, index);
			index += parseFrontLeftCliff(buffer, index);
			index += parseFrontRightCliff(buffer, index);
			index += parseRightCliff(buffer, index);
			index += parseVirtualWall(buffer, index);
			index += parseOvercurrents(buffer, index);
			index += parseDirtDetectorLeft(buffer, index); //was parseDirtDetector
			index += parseDirtDetectorRight(buffer, index); //added
			index += parseRemote(buffer, index);
			index += parseButtons(buffer, index);
			index += parseDistance(buffer, index);
			index += parseAngle(buffer, index);
			index += parseChargingState(buffer, index);
			index += parseVoltage(buffer, index);
			index += parseCurrent(buffer, index);
			index += parseTemperature(buffer, index);
			index += parseBatteryCharge(buffer, index);
			index += parseBatteryCapacity(buffer, index);
			i++;
		}
		if(sensor_packets_==OI_PACKET_GROUP_1)		// bytes 0-9
		{
			index += parseBumpersAndWheeldrops(buffer, index);
			index += parseWall(buffer, index);
			index += parseLeftCliff(buffer, index);
			index += parseFrontLeftCliff(buffer, index);
			index += parseFrontRightCliff(buffer, index);
			index += parseRightCliff(buffer, index);
			index += parseVirtualWall(buffer, index);
			index += parseOvercurrents(buffer, index);
			index += parseDirtDetectorLeft(buffer, index); //was parseDirtDetector
			index += parseDirtDetectorRight(buffer, index); //added
			i++;
		}
		if(sensor_packets_==OI_PACKET_GROUP_2)		// bytes 10-15
		{
			index += parseRemote(buffer, index);
			index += parseButtons(buffer, index);
			index += parseDistance(buffer, index);
			index += parseAngle(buffer, index);
			i++;
		}
		if(sensor_packets_==OI_PACKET_GROUP_3)		// bytes 16-25
		{
			index += parseChargingState(buffer, index);
			index += parseVoltage(buffer, index);
			index += parseCurrent(buffer, index);
			index += parseTemperature(buffer, index);
			index += parseBatteryCharge(buffer, index);
			index += parseBatteryCapacity(buffer, index);
			i++;
		}
	}
	return(0);
}

int irobot::SerialCommInterface::parseBumpersAndWheeldrops(unsigned char * buffer, int index)
{
	// Bumps, wheeldrops	
	this->bumper_[RIGHT] = (buffer[index]) & 0x01;
	this->bumper_[LEFT] = (buffer[index] >> 1) & 0x01;
	this->wheel_drop_[RIGHT] = (buffer[index] >> 2) & 0x01;
	this->wheel_drop_[LEFT] = (buffer[index] >> 3) & 0x01;
	
	return OI_PACKET_BUMPS_DROPS_SIZE;
}

int irobot::SerialCommInterface::parseWall(unsigned char * buffer, int index)
{
	// Wall
	this->wall_ = buffer[index] & 0x01;
	
	return OI_PACKET_WALL_SIZE;
}
	
int irobot::SerialCommInterface::parseLeftCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[LEFT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_LEFT_SIZE;
}

int irobot::SerialCommInterface::parseFrontLeftCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[FRONT_LEFT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_FRONT_LEFT_SIZE;
}

int irobot::SerialCommInterface::parseFrontRightCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[FRONT_RIGHT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_FRONT_RIGHT_SIZE;
}

int irobot::SerialCommInterface::parseRightCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[RIGHT] = buffer[index] & 0x01;
	
	return OI_PACKET_CLIFF_RIGHT_SIZE;
}

int irobot::SerialCommInterface::parseVirtualWall(unsigned char * buffer, int index)
{
	// Virtual Wall
	this->virtual_wall_ = buffer[index] & 0x01;
	
	return OI_PACKET_VIRTUAL_WALL_SIZE;
}
	
int irobot::SerialCommInterface::parseOvercurrents(unsigned char * buffer, int index)
{
	// Overcurrent
	unsigned char byte = buffer[index];
	
	this->overcurrent_[SIDE_BRUSH] = (byte >> 0) & 0x01;
	this->overcurrent_[MAIN_BRUSH] = (byte >> 2) & 0x01;
	this->overcurrent_[RIGHT] = (byte >> 3) & 0x01;
	this->overcurrent_[LEFT] = (byte >> 4) & 0x01;
	
	return OI_PACKET_WHEEL_OVERCURRENTS_SIZE;
}

int irobot::SerialCommInterface::parseDirtDetectorLeft(unsigned char * buffer, int index)
{
	// Dirt Detector
	this->dirt_detect_left = buffer[index];

	return OI_PACKET_DIRT_DETECT_SIZE;
}

int irobot::SerialCommInterface::parseDirtDetectorRight(unsigned char * buffer, int index)
{
	// Dirt Detector
	this->dirt_detect_right = buffer[index];

	return OI_PACKET_DIRT_DETECT_SIZE;
}
	
int irobot::SerialCommInterface::parseRemote(unsigned char * buffer, int index)
{
	// Infrared Character Omni
	this->ir_char_[OMNI] = buffer[index];

	return OI_PACKET_IR_CHAR_OMNI_SIZE;
}

int irobot::SerialCommInterface::parseButtons(unsigned char * buffer, int index)
{
	// Buttons
	for(int i=0 ; i<8 ; i++) this->buttons_[i] = (buffer[index] >> i) & 0x01;
	
	return OI_PACKET_BUTTONS_SIZE;
}
	
int irobot::SerialCommInterface::parseDistance(unsigned char * buffer, int index)
{
	// Distance
	this->distance_ = buffer2signed_int(buffer, index);
	
	return OI_PACKET_DISTANCE_SIZE;
}

int irobot::SerialCommInterface::parseAngle(unsigned char * buffer, int index)
{
	// Angle
	this->angle_ = buffer2signed_int(buffer, index);

	return OI_PACKET_ANGLE_SIZE;
}
	
int irobot::SerialCommInterface::parseChargingState(unsigned char * buffer, int index)
{
	// Charging State
	unsigned char byte = buffer[index];
	
	this->power_cord_ = (byte >> 0) & 0x01;
	this->dock_ = (byte >> 1) & 0x01;

	return OI_PACKET_CHARGING_STATE_SIZE;
}

int irobot::SerialCommInterface::parseVoltage(unsigned char * buffer, int index)
{
	// Voltage
	this->voltage_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_VOLTAGE_SIZE;
}

int irobot::SerialCommInterface::parseCurrent(unsigned char * buffer, int index)
{
	// Current
	this->current_ = (float)(buffer2signed_int(buffer, index) / 1000.0);

	return OI_PACKET_CURRENT_SIZE;
}

int irobot::SerialCommInterface::parseTemperature(unsigned char * buffer, int index)
{
	// Temperature
	this->temperature_ = (char)(buffer[index]);

	return OI_PACKET_TEMPERATURE_SIZE;
}

int irobot::SerialCommInterface::parseBatteryCharge(unsigned char * buffer, int index)
{
	// Charge
	this->charge_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_BATTERY_CHARGE_SIZE;
}

int irobot::SerialCommInterface::parseBatteryCapacity(unsigned char * buffer, int index)
{
	// Capacity
	this->capacity_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_BATTERY_CAPACITY_SIZE;
}
	
int irobot::SerialCommInterface::parseWallSignal(unsigned char * buffer, int index)
{
	// Wall signal
	this->wall_signal_ = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_WALL_SIGNAL_SIZE;
}
	
int irobot::SerialCommInterface::parseLeftCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_LEFT_SIGNAL_SIZE;
}

int irobot::SerialCommInterface::parseFrontLeftCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[FRONT_LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_FRONT_LEFT_SIGNAL_SIZE;
}
	
int irobot::SerialCommInterface::parseFontRightCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[FRONT_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL_SIZE;
}

int irobot::SerialCommInterface::parseRightCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_CLIFF_RIGHT_SIGNAL_SIZE;
}	
	
int irobot::SerialCommInterface::parseChargingSource(unsigned char * buffer, int index)
{
	// Charging soruces available
	this->power_cord_ = (buffer[index] >> 0) & 0x01;
	this->dock_ = (buffer[index] >> 1) & 0x01;

	return OI_PACKET_CHARGE_SOURCES_SIZE;
}

int irobot::SerialCommInterface::parseOiMode(unsigned char * buffer, int index)
{
	this->OImode_ = buffer[index];

	return OI_PACKET_OI_MODE_SIZE;
}

int irobot::SerialCommInterface::parseSongNumber(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_SONG_NUMBER_SIZE;
}

int irobot::SerialCommInterface::parseSongPlaying(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_SONG_PLAYING_SIZE;
}

int irobot::SerialCommInterface::parseNumberOfStreamPackets(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_STREAM_PACKETS_SIZE;
}

int irobot::SerialCommInterface::parseRequestedVelocity(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_REQ_VELOCITY_SIZE;
}

int irobot::SerialCommInterface::parseRequestedRadius(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_REQ_RADIUS_SIZE;
}

int irobot::SerialCommInterface::parseRequestedRightVelocity(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_REQ_RIGHT_VELOCITY_SIZE;
}

int irobot::SerialCommInterface::parseRequestedLeftVelocity(unsigned char * buffer, int index)
{
	// TODO
	return OI_PACKET_REQ_LEFT_VELOCITY_SIZE;
}

int irobot::SerialCommInterface::parseRightEncoderCounts(unsigned char * buffer, int index)
{
	// Right encoder counts
	uint16_t right_encoder_counts = buffer2unsigned_int(buffer, index);

	//printf("Right Encoder: %d\n", rightEncoderCounts);

	if(encoder_counts_[RIGHT] == -1 || right_encoder_counts == last_encoder_counts_[RIGHT])	// First time, we need 2 to make it work!
	{
		encoder_counts_[RIGHT] = 0;
	}
	else
	{
		encoder_counts_[RIGHT] = (int)(right_encoder_counts - last_encoder_counts_[RIGHT]);
		
		if(encoder_counts_[RIGHT] > ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[RIGHT] = encoder_counts_[RIGHT] - ROOMBA_MAX_ENCODER_COUNTS;
		if(encoder_counts_[RIGHT] < -ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[RIGHT] = ROOMBA_MAX_ENCODER_COUNTS + encoder_counts_[RIGHT];
	}
	last_encoder_counts_[RIGHT] = right_encoder_counts;
	
	return OI_PACKET_RIGHT_ENCODER_SIZE;
}

int irobot::SerialCommInterface::parseLeftEncoderCounts(unsigned char * buffer, int index)
{
	// Left encoder counts
	uint16_t left_encoder_counts = buffer2unsigned_int(buffer, index);

	//printf("Left Encoder: %d\n", leftEncoderCounts);

	if(encoder_counts_[LEFT] == -1 || left_encoder_counts == last_encoder_counts_[LEFT])	// First time, we need 2 to make it work!
	{
		encoder_counts_[LEFT] = 0;
	}
	else
	{
		encoder_counts_[LEFT] = (int)(left_encoder_counts - last_encoder_counts_[LEFT]);
		
		if(encoder_counts_[LEFT] > ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[LEFT] = encoder_counts_[LEFT] - ROOMBA_MAX_ENCODER_COUNTS;
		if(encoder_counts_[LEFT] < -ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[LEFT] = ROOMBA_MAX_ENCODER_COUNTS + encoder_counts_[LEFT];
	}
	last_encoder_counts_[LEFT] = left_encoder_counts;
	
	return OI_PACKET_LEFT_ENCODER_SIZE;
}
	
int irobot::SerialCommInterface::parseLightBumper(unsigned char * buffer, int index)
{
	// Light bumper
	this->ir_bumper_[LEFT] = (buffer[index]) & 0x01;
	this->ir_bumper_[FRONT_LEFT] = (buffer[index] >> 1) & 0x01;
	this->ir_bumper_[CENTER_LEFT] = (buffer[index] >> 2) & 0x01;
	this->ir_bumper_[CENTER_RIGHT] = (buffer[index] >> 3) & 0x01;
	this->ir_bumper_[FRONT_RIGHT] = (buffer[index] >> 4) & 0x01;
	this->ir_bumper_[RIGHT] = (buffer[index] >> 5) & 0x01;
	
	return OI_PACKET_LIGHT_BUMPER_SIZE;
}

int irobot::SerialCommInterface::parseLightBumperLeftSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->ir_bumper_signal_[LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_LEFT_SIZE;
}

int irobot::SerialCommInterface::parseLightBumperFrontLeftSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->ir_bumper_signal_[FRONT_LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_FRONT_LEFT_SIZE;
}

int irobot::SerialCommInterface::parseLightBumperCenterLeftSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->ir_bumper_signal_[CENTER_LEFT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_CENTER_LEFT_SIZE;
}

int irobot::SerialCommInterface::parseLightBumperCenterRightSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->ir_bumper_signal_[CENTER_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_CENTER_RIGHT_SIZE;
}

int irobot::SerialCommInterface::parseLightBumperFrontRightSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->ir_bumper_signal_[FRONT_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_FRONT_RIGHT_SIZE;
}
	
int irobot::SerialCommInterface::parseLightBumperRightSignal(unsigned char * buffer, int index)
{
	// Light bumper signal
	this->ir_bumper_signal_[RIGHT] = buffer2unsigned_int(buffer, index);
	
	return OI_PACKET_LIGHT_BUMPER_RIGHT_SIZE;
}

int irobot::SerialCommInterface::parseIrCharLeft(unsigned char * buffer, int index)
{
	// Infrared character left
	this->ir_char_[LEFT] = buffer[index];

	return OI_PACKET_IR_CHAR_LEFT_SIZE;
}
	
int irobot::SerialCommInterface::parseIrCharRight(unsigned char * buffer, int index)
{
	// Infrared character left
	this->ir_char_[RIGHT] = buffer[index];
	
	return OI_PACKET_IR_CHAR_RIGHT_SIZE;
}
	
int irobot::SerialCommInterface::parseLeftMotorCurrent(unsigned char * buffer, int index)
{
	// Left motor current
	this->motor_current_[LEFT] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_LEFT_MOTOR_CURRENT_SIZE;
}

int irobot::SerialCommInterface::parseRightMotorCurrent(unsigned char * buffer, int index)
{
	// Left motor current
	this->motor_current_[RIGHT] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_RIGHT_MOTOR_CURRENT_SIZE;
}

int irobot::SerialCommInterface::parseMainBrushMotorCurrent(unsigned char * buffer, int index)
{
	// Main brush motor current
	this->motor_current_[MAIN_BRUSH] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_BRUSH_MOTOR_CURRENT_SIZE;
}

int irobot::SerialCommInterface::parseSideBrushMotorCurrent(unsigned char * buffer, int index)
{
	// Main brush motor current
	this->motor_current_[SIDE_BRUSH] = buffer2signed_int(buffer, index);
	
	return OI_PACKET_SIDE_BRUSH_MOTOR_CURRENT_SIZE;
}

int irobot::SerialCommInterface::parseStasis(unsigned char * buffer, int index)
{
	// Stasis
	this->stasis_ = (buffer[index] >> 0) & 0x01;

	return OI_PACKET_STASIS_SIZE;
}

int irobot::SerialCommInterface::buffer2signed_int(unsigned char * buffer, int index)
{
	int16_t signed_int;
	
	memcpy(&signed_int, buffer+index, 2);
	signed_int = ntohs(signed_int);
	
	return (int)signed_int;
}

int irobot::SerialCommInterface::buffer2unsigned_int(unsigned char * buffer, int index)
{
	uint16_t unsigned_int;

	memcpy(&unsigned_int, buffer+index, 2);
	unsigned_int = ntohs(unsigned_int);
	
	return (int)unsigned_int;
}


// *****************************************************************************
// Calculate Roomba odometry
void irobot::SerialCommInterface::calculateOdometry()
{	
	double dist = (encoder_counts_[RIGHT]*ROOMBA_PULSES_TO_M + encoder_counts_[LEFT]*ROOMBA_PULSES_TO_M) / 2.0; 
	double ang = (encoder_counts_[RIGHT]*ROOMBA_PULSES_TO_M - encoder_counts_[LEFT]*ROOMBA_PULSES_TO_M) / -ROOMBA_AXLE_LENGTH;

	// Update odometry
	this->odometry_yaw_ = NORMALIZE(this->odometry_yaw_ + ang);			// rad
	this->odometry_x_ = this->odometry_x_ + dist*cos(odometry_yaw_);		// m
	this->odometry_y_ = this->odometry_y_ + dist*sin(odometry_yaw_);		// m
}


// *****************************************************************************
// Reset Roomba odometry
void irobot::SerialCommInterface::resetOdometry()
{
	this->setOdometry(0.0, 0.0, 0.0);
}


// *****************************************************************************
// Set Roomba odometry
void irobot::SerialCommInterface::setOdometry(double new_x, double new_y, double new_yaw)
{
	this->odometry_x_ = new_x;
	this->odometry_y_ = new_y;
	this->odometry_yaw_ = new_yaw;
}


// *****************************************************************************
// Clean
int irobot::SerialCommInterface::clean()
{
	return sendOpcode(OI_OPCODE_CLEAN);
}


// *****************************************************************************
// Max
int irobot::SerialCommInterface::max()
{
	return sendOpcode(OI_OPCODE_MAX);
}


// *****************************************************************************
// Spot
int irobot::SerialCommInterface::spot()
{
	return sendOpcode(OI_OPCODE_SPOT);
}


// *****************************************************************************
// Go to the dock
int irobot::SerialCommInterface::goDock()
{
	return sendOpcode(OI_OPCODE_FORCE_DOCK);
}


// *****************************************************************************
// Compose a song
int irobot::SerialCommInterface::setSong(unsigned char song_number, unsigned char song_length, unsigned char *notes, unsigned char *note_lengths)
{
	int size = 2*song_length+3;
	unsigned char cmd_buffer[size];
	unsigned char i;
	
	cmd_buffer[0] = (unsigned char)OI_OPCODE_SONG;
	cmd_buffer[1] = song_number;
	cmd_buffer[2] = song_length;
	
	for(i=0 ; i < song_length ; i++)
	{
		cmd_buffer[3+(2*i)] = notes[i];
		cmd_buffer[3+(2*i)+1] = note_lengths[i];
	}
	
	try{ serial_port_->write((char*)cmd_buffer, size); }
	catch(cereal::Exception& e){ return(-1); }
	return(0);
}


// *****************************************************************************
// Play a song from the list
int irobot::SerialCommInterface::playSong(unsigned char song_number)
{
	unsigned char cmd_buffer[2];
	
	cmd_buffer[0] = (unsigned char)OI_OPCODE_PLAY;
	cmd_buffer[1] = song_number;
	
	try{ serial_port_->write((char*)cmd_buffer, 2); }
	catch(cereal::Exception& e){ return(-1); }
	return(0);
}


// *****************************************************************************
// Set the LEDs
int irobot::SerialCommInterface::setLeds(unsigned char check_robot, unsigned char dock, unsigned char spot, unsigned char debris, unsigned char power_color, unsigned char power_intensity)
{
	unsigned char cmd_buffer[4];
	cmd_buffer[0] = (unsigned char)OI_OPCODE_LEDS;
	cmd_buffer[1] = debris | spot<<1 | dock<<2 | check_robot<<3;
	cmd_buffer[2] = power_color;
	cmd_buffer[3] = power_intensity;
	
	try{ serial_port_->write((char*)cmd_buffer, 4); }
	catch(cereal::Exception& e){ return(-1); }
	return(0);
}


// EOF
