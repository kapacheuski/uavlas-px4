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
 * Created on: March 28, 2019
 **/

#include "uavlas.h"
namespace uavlas{
UAVLAS::UAVLAS(int bus, int address) :
    I2C("uavlas", UAVLAS0_DEVICE_PATH, bus, address, 400000),
    _vehicleLocalPosition_valid(false),
    _reports(nullptr),
    _sensor_ok(false),
    _read_failures(0),
    _orb_class_instance(-1),
    _uavlasReportPub(nullptr),
   _targetPosePub(nullptr)
{
    _paramHandle.mode = param_find("LTEST_MODE");
    _paramHandle.scale_x = param_find("LTEST_SCALE_X");
    _paramHandle.scale_y = param_find("LTEST_SCALE_Y");

    memset(&_work, 0, sizeof(_work));

    initialize_topics();
    check_params(true);
}

UAVLAS::~UAVLAS()
{
    //stop();
    if (_reports != nullptr) {
        delete _reports;
    }
}

int UAVLAS::init()
{
    int ret = I2C::init();

    if (ret != OK) {
        /*warnx("I2C::init - failure");*/
        return ret;
    }

    _reports = new ringbuffer::RingBuffer(2, sizeof(struct uavlas_report_s));

    if (_reports == nullptr) {
        return ENOTTY;

    }
    else {
        _sensor_ok = true;
        //start();
        return OK;
    }
}
/** probe the device is on the I2C bus **/
int UAVLAS::probe()
{
    /*
     * device defaults to sending 0x00 when there is no block
     * data to return, so really all we can do is check to make
     * sure a transfer completes successfully.
     **/
    uint8_t byte;

    if (transfer(nullptr, 0, &byte, 1) != OK) {
        return -EIO;
    }

    return OK;
}
/** get device status information **/
int UAVLAS::info()
{

    /** display reports in queue **/
    if (_sensor_ok) {
        _reports->print_info("report queue: ");
        PX4_WARN("read errors:%lu", (unsigned long)_read_failures);

    }
    else {
        PX4_WARN("sensor is not healthy");
    }

    return OK;
}
///** device test **/
int UAVLAS::test(uint16_t secs)
{
    if (!_sensor_ok) {
        errx(1, "sensor not ready");
    }
//    warnx("Display sensor data %u sec.",secs);

//    struct uavlas_report_s report;
//    uint64_t start_time = hrt_absolute_time();

//    while ((hrt_absolute_time() - start_time) < (1000000*secs)) {
//        if (_reports->get(&report)) {
//            warnx("id:%u status:%u x:%f y:%f z:%f vx:%f vy:%f snr:%f cl:%f",
//                  report.id,
//                  report.status,
//                  (double)report.pos_x,
//                  (double)report.pos_y,
//                  (double)report.pos_z,
//                  (double)report.vel_x,
//                  (double)report.vel_y,
//                  (double)report.snr,
//                  (double)report.cl);

//        }
//        /** sleep for 0.1 seconds **/
//        usleep(100000);
//    }
    return OK;
}
///** device start **/
//void UAVLAS::start()
//{
//    _reports->flush();
//    work_queue(HPWORK, &_work, (worker_t)&UAVLAS::cycle_trampoline, this, 1);
//}

/** device work cycle implementation**/
void UAVLAS::update()
{
    check_params(false);
    update_topics();
    read_device();
}

void UAVLAS::status()
{
    struct uavlas_report_s report;
    if (_reports->get(&report)) {
        PX4_INFO("id:%u status:%u x:%f y:%f z:%f vx:%f vy:%f snr:%f cl:%f",
              report.id,
              report.status,
              (double)report.pos_x,
              (double)report.pos_y,
              (double)report.pos_z,
              (double)report.vel_x,
              (double)report.vel_y,
              (double)report.snr,
              (double)report.cl);
    }
}
/** device read lolevel implementation**/
//ssize_t UAVLAS::read(struct file *filp, char *buffer, size_t buflen)
//{
//    unsigned count = buflen / sizeof(struct uavlas_report_s);
//    //struct uavlas_report_s *rbuf = reinterpret_cast<struct uavlas_report_s *>(buffer);
//    struct uavlas_report_s rbuf;
//    int ret = 0;

//    if (count < 1) {
//        return -ENOSPC;
//    }
//    while (count--) {
//        if (_reports->get(&rbuf)) {
//            memcpy(buffer, &rbuf, sizeof(rbuf));
//            ret += sizeof(struct uavlas_report_s);
//            buffer+=sizeof(struct uavlas_report_s);
//        }
//    }
//    return ret ? ret : -EAGAIN;
//}
int UAVLAS::read_device_word(uint16_t *word)
{
    uint8_t bytes[2];
    memset(bytes, 0, sizeof bytes);

    int status = transfer(nullptr, 0, &bytes[0], 2);
    *word = bytes[1] << 8 | bytes[0];

    return status;
}

int UAVLAS::read_device_block(struct uavlas_report_s *block)
{
    uint8_t bytes[17];
    memset(bytes, 0, sizeof bytes);

    int status = transfer(nullptr, 0, &bytes[0], sizeof bytes);
    uint16_t id     = bytes[1] << 8 | bytes[0];
    uint16_t stat   = bytes[3] << 8 | bytes[2];
    int16_t  pos_y  = bytes[5] << 8 | bytes[4];
    int16_t  pos_x  = bytes[7] << 8 | bytes[6];
    int16_t  pos_z  = bytes[9] << 8 | bytes[8];
    int16_t  vel_y  = bytes[11] << 8 | bytes[10];
    int16_t  vel_x  = bytes[13] << 8 | bytes[12];
    uint8_t  snr    = bytes[14];
    uint8_t  cl     = bytes[15];
    uint8_t  crc    = bytes[16];

    uint8_t tcrc = 0;
    for (uint i=0; i<sizeof(bytes)-1; i++) {
        tcrc+=bytes[i];
    }
    /** crc check **/
    if (tcrc != crc) {
        _read_failures++;
        return -EIO;
    }

    /** convert to angles **/
    block->id = id;
    block->status = stat;

    block->pos_x = pos_x/100.0f * _params.scale_x;
    block->pos_y = pos_y/100.0f * _params.scale_y;
    block->pos_z = pos_z/100.0f;
    block->vel_x = vel_x/100.0f * _params.scale_x;
    block->vel_y = vel_y/100.0f * _params.scale_y;
    block->snr   = snr;
    block->cl    = cl;

    return status;
}
int UAVLAS::read_device()
{
    //struct uavlas_target_s report;
    struct uavlas_report_s orb_report;
    struct landing_target_pose_s _target_pose;


    orb_report.timestamp = hrt_absolute_time();

    if (read_device_block(&orb_report) != OK) {
        return -ENOTTY;
    }

    _reports->force(&orb_report);

//    orb_report.timestamp = report.timestamp;
//    orb_report.id = report.id;
//    orb_report.status = report.status;
//    orb_report.snr = report.snr;
//    orb_report.cl = report.cl;


//    orb_report.pos_x     = (float32)report.pos_x/100.0f;
//    orb_report.pos_y     = (float32)report.pos_y/100.0f;
//    orb_report.pos_z     = (float32)report.pos_z/100.0f;
//    orb_report.vel_x     = (float32)report.vel_x/100.0f;
//    orb_report.vel_y     = (float32)report.vel_y/100.0f;

    if (_uavlasReportPub != nullptr) {
        orb_publish(ORB_ID(uavlas_report), _uavlasReportPub, &orb_report);

    }
    else {
        _uavlasReportPub = orb_advertise_multi(ORB_ID(uavlas_report), &orb_report, &_orb_class_instance, ORB_PRIO_LOW);

        if (_uavlasReportPub == nullptr) {
            DEVICE_LOG("failed to create uavlas_report object. Did you start uOrb?");
        }

    }
    // Update landing_target_pose
    if ((!_vehicleLocalPosition_valid) ||
        (!PX4_ISFINITE(orb_report.pos_y) || !PX4_ISFINITE(orb_report.pos_x)|| !PX4_ISFINITE(orb_report.pos_z))||
        (orb_report.status != 7)) {
        // unable to update - no all values good.
        return OK;
    }

    _target_pose.timestamp = orb_report.timestamp;

    _target_pose.is_static = (_params.mode == TargetMode::Stationary);

    _target_pose.rel_pos_valid = true;
    _target_pose.rel_vel_valid = true;
    _target_pose.x_rel = orb_report.pos_x;
    _target_pose.y_rel = orb_report.pos_y;
    _target_pose.z_rel = orb_report.pos_z;
    _target_pose.vx_rel = orb_report.vel_x;
    _target_pose.vy_rel = orb_report.vel_y;

    _target_pose.cov_x_rel = orb_report.pos_z/20.0f; // Cov approximation
    _target_pose.cov_y_rel = orb_report.pos_z/20.0f; // Cov approximation

    _target_pose.cov_vx_rel = orb_report.pos_z/20.0f; // Cov approximation
    _target_pose.cov_vy_rel = orb_report.pos_z/20.0f; // Cov approximation

    if (_vehicleLocalPosition_valid && _vehicleLocalPosition.xy_valid) {
        _target_pose.x_abs = orb_report.pos_x + _vehicleLocalPosition.x;
        _target_pose.y_abs = orb_report.pos_y + _vehicleLocalPosition.y;
        _target_pose.z_abs = orb_report.pos_z + _vehicleLocalPosition.z;
        _target_pose.abs_pos_valid = true;

    }
    else {
        _target_pose.abs_pos_valid = false;
    }

    if (_targetPosePub == nullptr) {
        _targetPosePub = orb_advertise(ORB_ID(landing_target_pose), &_target_pose);

    }
    else {
        orb_publish(ORB_ID(landing_target_pose), _targetPosePub, &_target_pose);
    }

    return OK;
}


void UAVLAS::check_params(const bool force)
{
    bool updated;
    parameter_update_s paramUpdate;

    orb_check(_parameterSub, &updated);

    if (updated) {
        orb_copy(ORB_ID(parameter_update), _parameterSub, &paramUpdate);
    }

    if (updated || force) {
        update_params();
    }
}
void UAVLAS::initialize_topics()
{
    _vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
    _parameterSub = orb_subscribe(ORB_ID(parameter_update));
}

void UAVLAS::update_topics()
{
    _vehicleLocalPosition_valid = orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub,
                                  &_vehicleLocalPosition);
}

bool UAVLAS::orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
    bool newData = false;
    // check if there is new data to grab
    if (orb_check(handle, &newData) != OK) {
        return false;
    }
    if (!newData) {
        return false;
    }
    if (orb_copy(meta, handle, buffer) != OK) {
        return false;
    }
    return true;
}

void UAVLAS::update_params()
{
    int32_t mode = 0;
    param_get(_paramHandle.mode, &mode);
    _params.mode = (TargetMode)mode;
    param_get(_paramHandle.scale_x, &_params.scale_x);
    param_get(_paramHandle.scale_y, &_params.scale_y);
}

}
