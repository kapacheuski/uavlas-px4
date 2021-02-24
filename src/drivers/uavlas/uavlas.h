/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file uavlas.h
 * @author Yury Kapacheuski
 *
 * Driver for an ULS-QR1 UAVLAS sensor connected via I2C.
 *
 * Created on: Febryary 21, 2021
 **/

#include <string.h>

#include <drivers/device/i2c.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/uavlas_report.h>
#include <uORB/topics/landing_target_pose.h>

//PARAM_DEFINE_INT32(ULS_EN_SENSOR, 0);

#define UAVLAS_I2C_ADDRESS	  0x55
#define UAVLAS_I2C_HMI_ADDR	  0x80 // Who am i register
#define UAVLAS_I2C_ULSQR1R1_HMI	  0x01 // Who am i register
#define UAVLAS_I2C_DATA_ADDR  	  0x00 // data read register


#define UAVLAS_UPDATE_INTERVAL_US 50000

namespace uavlas {

struct uavlas_target_s {
	uint64_t timestamp;
	float pos_x;
	float pos_y;
	float pos_z;
	float vel_x;
	float vel_y;
	float vel_z;
	float gu_roll;
	float gu_pitch;
	float gu_yaw;
	float bro_yaw;
	float sq;
	float cl;
	float sl;
	uint16_t id;
	uint16_t status;
};

class UAVLAS : public device::I2C, public I2CSPIDriver<UAVLAS>
{
public:
        UAVLAS(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address);
        ~UAVLAS() override = default;

        static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);

	    static void print_usage();

	    int init() override;
	    int probe() override;
	    void print_status() override;
      //  void update();
        void RunImpl();
private:
        int 		read_device();
	    int 		read_device_block(struct uavlas_report_s *block);
	    uORB::Publication<uavlas_report_s> _uavlas_report_topic{ORB_ID(uavlas_report)};

        struct uavlas_report_s orb_report;
        bool _vehicleLocalPosition_valid;
        //vehicle_local_position_s	_vehicleLocalPosition;
        int _read_failures;
        int _crc_failures;
};
}

