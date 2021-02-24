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
 * @file uavlas.cpp
 * @author Yury Kapacheuski
 *
 * Driver for an ULS-QR1 UAVLAS sensor connected via I2C.
 *
 * Created on: Febryary 21, 2021
 **/
#include "uavlas.h"
namespace uavlas
{

UAVLAS::UAVLAS(I2CSPIBusOption bus_option, const int bus, int bus_frequency, const int address) :
	I2C(DRV_SENS_DEVTYPE_UAVLAS, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
	_vehicleLocalPosition_valid(false),
	_read_failures(0),
	_crc_failures(0)
{
}
int UAVLAS::init()
{
	int rez = I2C::init();

	if (rez == OK) {
		ScheduleNow();
	}

	return rez;
}
int UAVLAS::probe()
{
	uint8_t hmi;
	uint8_t reg = UAVLAS_I2C_HMI_ADDR;

	if (transfer(&reg, 1, &hmi, 1) != OK) {
		return -EIO;
	}

	switch (hmi) {
	case UAVLAS_I2C_ULSQR1R1_HMI:
		PX4_INFO("ULS-QR1-R1 connected.");
		break;

	default:
		PX4_INFO("Unsupported UAVLAS device connected.");
	}

	return OK;
}
void UAVLAS::print_status()
{
	PX4_INFO("id:%u status:%u x:%.2f y:%.2f z:%.2f vx:%.2f vy:%.2f sq:%.2f cs:%f sl:%f RE:%d CRCE:%d",
		 orb_report.id,
		 orb_report.status,
		 (double)orb_report.pos_x,
		 (double)orb_report.pos_y,
		 (double)orb_report.pos_z,
		 (double)orb_report.vel_x,
		 (double)orb_report.vel_y,
		 (double)orb_report.sq,
		 (double)orb_report.ss,
		 (double)orb_report.sl,
		 _read_failures,
		 _crc_failures);
}
void UAVLAS::RunImpl()
{
	read_device();
	ScheduleDelayed(UAVLAS_UPDATE_INTERVAL_US);
}
int UAVLAS::read_device()
{
	orb_report.timestamp = hrt_absolute_time();

	if (read_device_block(&orb_report) != OK) {
		return -ENOTTY;
	}

	_uavlas_report_topic.publish(orb_report);
	return OK;
}
int UAVLAS::read_device_block(struct uavlas_report_s *block)
{
	uint8_t bytes[28];
	memset(bytes, 0, sizeof bytes);
	uint8_t reg = UAVLAS_I2C_DATA_ADDR;
	int status = transfer(&reg, 1, &bytes[0], sizeof bytes);

	if (status != OK) {
		_read_failures++;
		return status;
	}

	uint16_t id     = bytes[1] << 8 | bytes[0];
	uint16_t stat   = bytes[3] << 8 | bytes[2];
	int16_t  pos_y  = bytes[5] << 8 | bytes[4];
	int16_t  pos_x  = bytes[7] << 8 | bytes[6];
	int16_t  pos_z  = bytes[9] << 8 | bytes[8];
	int16_t  vel_y  = bytes[11] << 8 | bytes[10];
	int16_t  vel_x  = bytes[13] << 8 | bytes[12];
	int16_t  vel_z  = bytes[15] << 8 | bytes[14];
	int16_t  gimu_roll  = bytes[17] << 8 | bytes[16];
	int16_t  gimu_pitch = bytes[19] << 8 | bytes[18];
	int16_t  gimu_yaw   = bytes[21] << 8 | bytes[20];
	int16_t  bro_yaw    = bytes[23] << 8 | bytes[22];
	uint8_t  sq     = bytes[24]; // Sigla quality
	uint8_t  ss     = bytes[25]; // Signal saturation
	uint8_t  sl     = bytes[26]; // Signal Level
	uint8_t  crc    = bytes[27];

	uint8_t tcrc = 0;
	for (uint i = 0; i < sizeof(bytes) - 1; i++) {
		tcrc += bytes[i];
	}

	if (tcrc != crc) {
		_crc_failures++;
		return -EIO;
	}

	block->id = id;
	block->status = stat;
	block->pos_x = pos_x / 100.f;
	block->pos_y = pos_y / 100.f;
	block->pos_z = pos_z / 100.f;
	block->vel_x = vel_x / 100.f;
	block->vel_y = vel_y / 100.f;
	block->vel_z = vel_z / 100.f;
	block->gimu_roll = gimu_roll / 10.f;
	block->gimu_pitch = gimu_pitch / 10.f;
	block->gimu_yaw = gimu_yaw / 10.f;
	block->bro_yaw = bro_yaw / 10.f;
	block->sq   = ((float32)sq) / 2.55f;
	block->ss   = ss;
	block->sl   = ((float32)sl) / 2.55f;

	return status;
}
I2CSPIDriverBase *UAVLAS::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	UAVLAS *instance = new UAVLAS(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}
void UAVLAS::print_usage()
{
	PRINT_MODULE_USAGE_NAME("uavlas", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(UAVLAS_I2C_ADDRESS);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}
}
