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
 * Driver for an ULS-Qx1 sensor connected via I2C.
 *
 * Created on: March 28, 2019
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

#include <px4_getopt.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/uavlas_report.h>

#define UAVLAS_I2C_BUS			PX4_I2C_BUS_EXPANSION
#define UAVLAS_I2C_ADDRESS		0x55 /** 7-bit address (non shifted) **/
#define UAVLAS_CONVERSION_INTERVAL_US	100000U /** us = 100ms = 10Hz **/


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define UAVLAS_BASE_DEVICE_PATH	"/dev/uavlas"
#define UAVLAS0_DEVICE_PATH	"/dev/uavlas0"

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

struct uavlas_target_s {
    uint64_t timestamp;
    uint16_t id;	 /** target id **/
    uint16_t status; /** target status **/

    int16_t  pos_x;  /** x-axis distance (NE) from beacon **/
    int16_t  pos_y;	/** y-axis distance (NE) from beacon **/
    uint16_t pos_z;	/** z-axis distance (Altitude) from beacon **/
    uint8_t  snr;    /** Signal to noise ratio in db **/
    uint8_t  cl;     /** Commin lighting level in db **/

}__attribute__((packed));


class UAVLAS : public device::I2C {
public:
    UAVLAS(int bus = UAVLAS_I2C_BUS, int address = UAVLAS_I2C_ADDRESS);
    virtual ~UAVLAS();

    virtual int init();
    virtual int probe();
    virtual int info();
    virtual int test(uint16_t secs);

    virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);

private:

    void 		start();
    void 		stop();
    static void	cycle_trampoline(void *arg);
    void		cycle();

    int 		read_device();
    int 		read_device_word(uint16_t *word);
    int 		read_device_block(struct uavlas_target_s *block);

    ringbuffer::RingBuffer *_reports;
    bool _sensor_ok;
    work_s _work;
    uint32_t _read_failures;

    int _orb_class_instance;
    orb_advert_t _uavlas_report_topic;
};

namespace {
UAVLAS *g_uavlas = nullptr;
}

void uavlas_usage();
bool check_device(int bus);

extern "C" __EXPORT int uavlas_main(int argc, char *argv[]);

/** constructor **/
UAVLAS::UAVLAS(int bus, int address) :
    I2C("uavlas", UAVLAS0_DEVICE_PATH, bus, address, 400000),
    _reports(nullptr),
    _sensor_ok(false),
    _read_failures(0),
    _orb_class_instance(-1),
    _uavlas_report_topic(nullptr)
{
    memset(&_work, 0, sizeof(_work));
}

UAVLAS::~UAVLAS()
{
    stop();

    /** clear reports queue **/
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

    _reports = new ringbuffer::RingBuffer(2, sizeof(struct uavlas_target_s));

    if (_reports == nullptr) {
        return ENOTTY;

    }
    else {
        _sensor_ok = true;
        start();
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

int UAVLAS::info()
{
    if (g_uavlas == nullptr) {
        errx(1, "uavlas device driver is not running");
    }

    /** display reports in queue **/
    if (_sensor_ok) {
        _reports->print_info("report queue: ");
        warnx("read errors:%lu", (unsigned long)_read_failures);

    }
    else {
        warnx("sensor is not healthy");
    }

    return OK;
}

int UAVLAS::test(uint16_t secs)
{
    if (g_uavlas == nullptr) {
        errx(1, "uavlas device driver is not running");
    }

    if (!_sensor_ok) {
        errx(1, "sensor not ready");
    }

    warnx("Display sensor data %u sec.",secs);

    struct uavlas_target_s report;
    uint64_t start_time = hrt_absolute_time();

    while ((hrt_absolute_time() - start_time) < (1000000*secs)) {
        if (_reports->get(&report)) {
            warnx("id:%u status:%u x:%d y:%d z:%u snr:%u cl:%u",
                  report.id,
                  report.status,
                  report.pos_x,
                  report.pos_y,
                  report.pos_z,
                  report.snr,
                  report.cl);


        }
        /** sleep for 0.1 seconds **/
        usleep(100000);
    }

    return OK;
}

void UAVLAS::start()
{
    _reports->flush();


    work_queue(HPWORK, &_work, (worker_t)&UAVLAS::cycle_trampoline, this, 1);
}

void UAVLAS::stop()
{
    work_cancel(HPWORK, &_work);
}

void UAVLAS::cycle_trampoline(void *arg)
{
    UAVLAS *device = (UAVLAS *)arg;

    if (g_uavlas != nullptr) {
        device->cycle();
    }
}

void UAVLAS::cycle()
{
    read_device();

    work_queue(HPWORK, &_work, (worker_t)&UAVLAS::cycle_trampoline, this, USEC2TICK(UAVLAS_CONVERSION_INTERVAL_US));
}

ssize_t UAVLAS::read(struct file *filp, char *buffer, size_t buflen)
{
    unsigned count = buflen / sizeof(struct uavlas_target_s);
    struct uavlas_target_s *rbuf = reinterpret_cast<struct uavlas_target_s *>(buffer);
    int ret = 0;

    if (count < 1) {
        return -ENOSPC;
    }

    while (count--) {
        if (_reports->get(rbuf)) {
            ret += sizeof(*rbuf);
            ++rbuf;
        }
    }

    return ret ? ret : -EAGAIN;
    return ret;
}


int UAVLAS::read_device()
{
    struct uavlas_target_s report;

    report.timestamp = hrt_absolute_time();

    if (read_device_block(&report) != OK) {
        return -ENOTTY;
    }

    _reports->force(&report);

    struct uavlas_report_s orb_report;

    orb_report.timestamp = report.timestamp;
    orb_report.id = report.id;
    orb_report.status = report.status;

    orb_report.pos_x     = (float32)report.pos_x/100.0f;
    orb_report.pos_y     = (float32)report.pos_y/100.0f;
    orb_report.pos_z     = (float32)report.pos_z/100.0f;

    if (_uavlas_report_topic != nullptr) {
        orb_publish(ORB_ID(uavlas_report), _uavlas_report_topic, &orb_report);

    }
    else {
        _uavlas_report_topic = orb_advertise_multi(ORB_ID(uavlas_report), &orb_report, &_orb_class_instance, ORB_PRIO_LOW);

        if (_uavlas_report_topic == nullptr) {
            DEVICE_LOG("failed to create uavlas_report object. Did you start uOrb?");
        }

    }

    return OK;
}

int UAVLAS::read_device_word(uint16_t *word)
{
    uint8_t bytes[2];
    memset(bytes, 0, sizeof bytes);

    int status = transfer(nullptr, 0, &bytes[0], 2);
    *word = bytes[1] << 8 | bytes[0];

    return status;
}

int UAVLAS::read_device_block(struct uavlas_target_s *block)
{
    uint8_t bytes[13];
    memset(bytes, 0, sizeof bytes);

    int status = transfer(nullptr, 0, &bytes[0], sizeof bytes);
    uint16_t id     = bytes[1] << 8 | bytes[0];
    uint16_t stat   = bytes[3] << 8 | bytes[2];
    int16_t  pos_y  = bytes[5] << 8 | bytes[4];
    int16_t  pos_x  = bytes[7] << 8 | bytes[6];
    uint16_t pos_z  = bytes[9] << 8 | bytes[8];
    uint8_t  snr    = bytes[10];
    uint8_t  cl     = bytes[11];
    uint8_t  crc    = bytes[12];

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

    block->pos_x = pos_x;
    block->pos_y = pos_y;
    block->pos_z = pos_z;
    block->snr   = snr;
    block->cl    = cl;

    return status;
}

void uavlas_usage()
{
    warnx("missing command: try 'start', 'stop', 'info', 'test'");
    warnx("options:");
    warnx("    -b i2cbus");
    warnx("    -t test time (sec)");

}
bool check_device(int bus)
{
    /* create the driver */
    /* UAVLAS("trying bus %d", *cur_bus); */
    g_uavlas = new UAVLAS(bus, UAVLAS_I2C_ADDRESS);

    if (g_uavlas == nullptr) {
        /* this is a fatal error */
        warnx("failed to allocated memory for driver");

    }
    else if (OK == g_uavlas->init()) {
        /* success! */
        return true;
    }
    else {
       /* warnx("failed to initialize device, on bus %d",bus);*/
    }

    /* destroy it again because it failed. */
    UAVLAS *tmp_uavlas = g_uavlas;
    g_uavlas = nullptr;
    delete tmp_uavlas;
    return false;
}

int uavlas_main(int argc, char *argv[])
{
    int i2cdevice = -1;// UAVLAS_I2C_BUS;
    int secs = 10;
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "b:t:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'b':
            i2cdevice = (uint8_t)atoi(myoptarg);
            break;
        case 't':
            secs = (uint8_t)atoi(myoptarg);
            break;

        default:
            PX4_WARN("Unknown option!");
            return -1;
        }
    }

    if (myoptind >= argc) {
        uavlas_usage();
        exit(1);
    }

    const char *command = argv[myoptind];

    /** start driver **/
    if (!strcmp(command, "start")) {
        if (g_uavlas != nullptr) {
            errx(1, "driver has already been started");
        }

        if (i2cdevice == -1) {
            const int *cur_bus = busses_to_try;
            while (*cur_bus != -1) {
                /* create the driver */
                /* UAVLAS("trying bus %d", *cur_bus); */
                if (check_device(*cur_bus)) {
                    i2cdevice = *cur_bus;
                    break;
                }
                /* try next! */
                cur_bus++;
            }
        }
        else if (!check_device(i2cdevice)) {
            i2cdevice = -1;
        }

        if (i2cdevice == -1) {
            errx(1, "unable to find device");
        }
        PX4_INFO("Started on bus#:%d",i2cdevice);
        exit(0);
    }

    /** need the driver past this point **/
    if (g_uavlas == nullptr) {
        warnx("not started");
        uavlas_usage();
        exit(1);
    }

    /** stop the driver **/
    if (!strcmp(command, "stop")) {
        UAVLAS *tmp_uavlas = g_uavlas;
        g_uavlas = nullptr;
        delete tmp_uavlas;
        warnx("uavlas stopped");
        exit(OK);
    }

    /** Print driver information **/
    if (!strcmp(command, "info")) {
        g_uavlas->info();
        exit(OK);
    }

    /** test driver **/
    if (!strcmp(command, "test")) {
        g_uavlas->test(secs);
        exit(OK);
    }

    /** display usage info **/
    uavlas_usage();
    exit(0);
}
