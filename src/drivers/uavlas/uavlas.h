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
 * Created on: July 18, 2019
 **/

#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <board_config.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>

#include <px4_getopt.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/uavlas_report.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>


namespace uavlas{
// Defines section
#define UAVLAS_I2C_BUS			PX4_I2C_BUS_EXPANSION
#define UAVLAS_I2C_ADDRESS		0x55 /** 7-bit address (non shifted) **/
#define UAVLAS_UPDATE_RATE   	10U /** 10Hz **/

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define UAVLAS_BASE_DEVICE_PATH	"/dev/uavlas"
#define UAVLAS0_DEVICE_PATH	"/dev/uavlas0"

// Global const definitions
const int busses_to_try[] = {
    PX4_I2C_BUS_EXPANSION,
#ifdef PX4_I2C_BUS_ESC
    PX4_I2C_BUS_ESC,
#endif
#ifdef PX4_I2C_BUS_ONBOARD
    PX4_I2C_BUS_ONBOARD,
#endif
#ifdef PX4_I2C_BUS_EXPANSION1
    PX4_I2C_BUS_EXPANSION1,
#endif
#ifdef PX4_I2C_BUS_EXPANSION2
    PX4_I2C_BUS_EXPANSION2,
#endif
    -1
};

//// Global definitions
//struct uavlas_target_s {
//    uint64_t timestamp;
//    uint16_t id;	 /** target id **/
//    uint16_t status; /** target status **/

//    int16_t  pos_x;  /** x-axis distance (NE) from beacon **/
//    int16_t  pos_y;	 /** y-axis distance (NE) from beacon **/
//    uint16_t pos_z;	 /** z-axis distance (Altitude) from beacon **/
//    int16_t  vel_x;  /** x-axis velocity (NE) from beacon **/
//    int16_t  vel_y;	 /** y-axis velocity (NE) from beacon **/

//    uint8_t  snr;    /** Signal to noise ratio in db **/
//    uint8_t  cl;     /** Commin lighting level in db **/
//}__attribute__((packed));

// Device class
class UAVLAS : public device::I2C {
public:
    UAVLAS(int bus = UAVLAS_I2C_BUS, int address = UAVLAS_I2C_ADDRESS);
    virtual ~UAVLAS();

    virtual int init();
    virtual int probe();
    virtual int info();
    virtual int test(uint16_t secs);

   // virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);

    void		update();
    void		status();
private:
    // Device Control Functions
    //void 		start();
    //void 		stop();


    int 		read_device();
    int 		read_device_word(uint16_t *word);
    int 		read_device_block(struct uavlas_report_s *block);

    void check_params(const bool force);
    void initialize_topics();
    void update_topics();
    void update_params();
    /*
     * Convenience function for polling uORB subscriptions.
     *
     * @return true if there was new data and it was successfully copied
     */
    static bool orb_update(const struct orb_metadata *meta, int handle, void *buffer);

    // Parameters section

    int _parameterSub;
    struct {
        param_t mode;
        param_t scale_x;
        param_t scale_y;
    } _paramHandle;
    enum class TargetMode {
        Moving = 0,
        Stationary
    };
    struct {
        float acc_unc;
        float meas_unc;
        float pos_unc_init;
        float vel_unc_init;
        TargetMode mode;
        float scale_x;
        float scale_y;
    } _params;


    int _vehicleLocalPositionSub;
    bool _vehicleLocalPosition_valid;
    struct vehicle_local_position_s	_vehicleLocalPosition;

    // Device data section
    ringbuffer::RingBuffer *_reports;
    bool _sensor_ok;
    uint32_t _read_failures;
    int _orb_class_instance;
    orb_advert_t _uavlasReportPub;
    orb_advert_t _targetPosePub;

    work_s _work;
};

}

