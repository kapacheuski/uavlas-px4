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
	ModuleParams(nullptr),
	_vehicleLocalPosition_valid(false),
	_read_failures(0),
	_crc_failures(0)
{
}
int UAVLAS::init()
{
	int rez = I2C::init();

	if (rez == OK) {
		_parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
		parameters_update(_parameter_update_sub, true);
		// TODO: add on stop - orb_unsubscribe(parameter_update_sub);
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
	parameters_update(_parameter_update_sub);

	if (_uls_enabled.get() != 0) {
		if (read_device() == OK) { updateNavigation(); }
	}

	ScheduleDelayed(UAVLAS_UPDATE_INTERVAL_US);
}
void UAVLAS::updateNavigation()
{
	update_topics();
	// publish report
	orb_report.timestamp = hrt_absolute_time();
	_uavlas_report_topic.publish(orb_report);

	//0 - Throttle drop - set thr drop procedure if configured.
	// if started do drop.

	if (!_vehicleLocalPosition_valid) {
		return ;
	}

	//1 - Check signal received - if not -> exit
	if (!(orb_report.status & ULS_STATUS_MASK_CARRIER_OK)) { return; }

	//2 - Check Offset calculated (distance valid) -> if not ->exit
	if (!(orb_report.status & ULS_STATUS_MASK_POS_OK)) { return; }

	matrix::Vector3f pos(orb_report.pos_x, orb_report.pos_y, orb_report.pos_z);
	matrix::Vector3f vel(orb_report.vel_x, orb_report.vel_y, orb_report.vel_z);

	//3 - Check MultiRX orientation -> if valid use to rotate offset. if not try use compass.
	if (orb_report.status & ULS_STATUS_MASK_MRX_OK) {
		// pos and vel rotete with orb_report.bro_yaw
		// 8
		// matrix::Quaternion<float> q_att(&_vehicleAttitude.q[0]);
		// matrix::Euler<float>  euler(orb_report.gimu_yaw, 0, 0);
		// matrix::Dcmf r_rel = matrix::Dcm<float>(q_att);
		// matrix::Dcmf r_abs = matrix::Dcm<float>(euler);

		// pos = r_abs * pos;
		// vel = r_abs * vel;
		// pos = r_rel * pos;
		// vel = r_rel * vel;

		if (!(orb_report.status & ULS_STATUS_MASK_GU_IMU_OK)) {
			// Calculate compass deviation to use in future

		} else {
			// reset deviation value
		}
	} else {
		//4 - Check Compass -> if valid rotate offset. if not skip target pos.
		if (orb_report.status & ULS_STATUS_MASK_GU_IMU_OK) {
			// pos and vel rotate with orb_report.gimu[2] - with deviation applyed (if has)
			matrix::Dcmf r_abs = matrix::Dcmf(matrix::Eulerf{0, 0, math::radians((float) - orb_report.gimu_yaw)});

			pos = r_abs * pos;
			vel = r_abs * vel;
		} else {
			// No rotation provided unable to calculate new position.
			return;
		}
	}

	// 6- apply information to landing position

	_target_pose.timestamp = orb_report.timestamp;
	_target_pose.is_static = _uls_mode.get();

	_target_pose.rel_pos_valid = true;
	_target_pose.rel_vel_valid = true;
	_target_pose.x_rel = pos(0);
	_target_pose.y_rel = pos(1);
	_target_pose.z_rel = pos(2);
	_target_pose.vx_rel = vel(0);
	_target_pose.vy_rel = vel(1);

	_target_pose.cov_x_rel = orb_report.pos_z / 20.0f; // Cov approximation
	_target_pose.cov_y_rel = orb_report.pos_z / 20.0f; // Cov approximation

	_target_pose.cov_vx_rel = orb_report.pos_z / 20.0f; // Cov approximation
	_target_pose.cov_vy_rel = orb_report.pos_z / 20.0f; // Cov approximation

	if (_vehicleLocalPosition.xy_valid) {
		_target_pose.x_abs = pos(0) + _vehicleLocalPosition.x;
		_target_pose.y_abs = pos(1) + _vehicleLocalPosition.y;
		_target_pose.z_abs = pos(2) + _vehicleLocalPosition.z;
		_target_pose.abs_pos_valid = true;

	} else {
		_target_pose.abs_pos_valid = false;
	}

	_targetPosePub.publish(_target_pose);

	//6 - Provide AGL - set AGL of the system if configured
	if (_uls_provide_agl.get() != 0) {
		distance_sensor_s distance_report{};
		distance_report.timestamp = orb_report.timestamp;
		distance_report.min_distance = 0.1;
		distance_report.max_distance = 20.0;
		distance_report.current_distance = orb_report.pos_z;
		distance_report.variance = orb_report.pos_z / 10.f;
		distance_report.signal_quality = 100;
		distance_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED;
		/* TODO: the ID needs to be properly set */
		distance_report.id = 0;
		distance_report.orientation = ROTATION_PITCH_270;

		_distance_sensor_topic.publish(distance_report);
	}

	//7 - start Thr Drop if configured
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
	__cpx_uls_data_packet packet;
	memset(&packet, 0, sizeof packet);
	uint8_t reg = UAVLAS_I2C_DATA_ADDR;
	int status = transfer(&reg, 1, (uint8_t *)&packet, sizeof packet);

	if (status != OK) {
		_read_failures++;
		return status;
	}

	uint16_t tcrc = 0;
	uint8_t *pxPack = (uint8_t *)&packet;

	for (uint i = 0; i < sizeof(packet) - 2; i++) {
		tcrc += *pxPack++;
	}

	if (tcrc != packet.crc) {
		_crc_failures++;
		return -EIO;
	}

	block->id = packet.guid;
	block->status = packet.status;
	block->pos_x = packet.pos[0] / 100.f;
	block->pos_y = packet.pos[1] / 100.f;
	block->pos_z = packet.pos[2] / 100.f;
	block->vel_x = packet.vel[0] / 100.f;
	block->vel_y = packet.vel[1] / 100.f;
	block->vel_z = packet.vel[2] / 100.f;
	block->gimu_roll = packet.gimu[0] / 10.f;
	block->gimu_pitch = packet.gimu[1] / 10.f;
	block->gimu_yaw = packet.gimu[2] / 10.f;
	block->bro_yaw = packet.bro_yaw / 10.f;
	block->rx_x = packet.rx_xy[0] / 10.f;
	block->rx_y = packet.rx_xy[1] / 10.f;
	block->sq   = ((float32)packet.sq);
	block->ss   = packet.ss;
	block->sl   = ((float32)packet.sl);

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

void UAVLAS::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	// Check if any parameter updated
	orb_check(parameter_update_sub, &updated);

	// If any parameter updated copy it to: param_upd
	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

void UAVLAS::update_topics()
{
	_vehicleLocalPosition_valid = _vehicleLocalPositionSub.update(&_vehicleLocalPosition);
	_vehicleAttitude_valid = _attitudeSub.update(&_vehicleAttitude);

}

}
