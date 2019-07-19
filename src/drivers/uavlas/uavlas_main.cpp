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
 * @file uavlas_main.cpp
 * @author Yury Kapacheuski
 *
 * Driver for an ULS-QR1 UAVLAS sensor connected via I2C.
 *
 * Created on: July 18, 2019
 **/

#include "uavlas.h"

#define STATUS_UPDATE_RATE 2 /*status update freq */

namespace uavlas {
// Global functions
void uavlas_usage();
UAVLAS * create_device(int bus);
UAVLAS *g_uavlas = nullptr;
static bool thread_should_exit = false;	/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;			/**< Handle of daemon task / thread */
static bool sensor_ready = false;			/**< Handle of daemon task / thread */

static int statusDisplayTime = 0;
static bool infoDisplay = false;

extern "C" __EXPORT int uavlas_main(int argc, char *argv[]);

UAVLAS *create_device(int bus)
{
    /* create the driver */
    /* UAVLAS("trying bus %d", *cur_bus); */
    UAVLAS *u = new UAVLAS(bus, UAVLAS_I2C_ADDRESS);

    if (u == nullptr) {
        /* this is a fatal error */
        PX4_WARN("failed to allocated memory for driver");
    }
    else if (OK == u->init()) {
        /* success! */
        return u;
    }
    else {
        // PX4_WARN("failed to initialize device, on bus %d",bus);
    }
    /* destroy it again because it failed. */
    UAVLAS *tmp_uavlas = u;
    u = nullptr;
    delete tmp_uavlas;
    return nullptr;
}

int uavlas_thread(int argc, char *argv[])
{

    UAVLAS *uavlas;

    thread_running = true;
    thread_should_exit = true;

    const int *cur_bus = busses_to_try;
    px4_usleep(5000000); // Wait 5 sec for sensor start up
    while ((*cur_bus != -1) && (thread_should_exit)) {
        uavlas = create_device(*cur_bus);
        if (uavlas != nullptr) {
            PX4_INFO("Started on bus#:%d",*cur_bus);
            thread_should_exit = false;
            break;
        }
        /* try next! */
        cur_bus++;
    }
    if(thread_should_exit){
        PX4_INFO("uavlas unable to connect.");
        thread_running = false;
        return 0;
    };

    int statusUpdateTime = 0;
    while (!thread_should_exit) {

        uavlas->update();
        if (statusDisplayTime && (++statusUpdateTime >= (10/STATUS_UPDATE_RATE))) {
            statusDisplayTime--;
            statusUpdateTime = 0;
            uavlas->status();
        }
        if (infoDisplay) {
            infoDisplay = false;
            uavlas->info();
        }
        sensor_ready = true;
        px4_usleep(1000000 / UAVLAS_UPDATE_RATE);
    }
    sensor_ready = false;
    if (uavlas != nullptr) {
        delete uavlas;
    }

    PX4_INFO("exiting");

    thread_running = false;

    return 0;
}

void uavlas_usage()
{
    PX4_INFO("missing command: try 'start', 'stop', 'info', 'status'");
    PX4_INFO("options:");
    PX4_INFO("    -t status display time time (sec)");
}

int uavlas_main(int argc, char *argv[])
{

    int secs = 10;
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "t:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
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
        if (thread_running) {
            PX4_INFO("already running");
            /* this is not an error */
            return true;
        }
        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("uavlas",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_DEFAULT,
                                         2000,
                                         uavlas_thread,
                                         (argv) ? (char *const *)&argv[2] : nullptr);
        exit(OK);
    }
    if (!thread_running) {
        PX4_WARN("uavlas not running");
        uavlas_usage();
        exit(0);
    }
    if (!sensor_ready) {
        PX4_WARN("uavlas sensor not ready");
        uavlas_usage();
        exit(0);
    }

    /** stop the driver **/
    if (!strcmp(command, "stop")) {
        PX4_INFO("stopping driver");
        thread_should_exit = true;
        exit(OK);
    }
    /** Print driver information **/
    if (!strcmp(command, "info")) {
        PX4_INFO("device info:");
        infoDisplay = true;
        exit(OK);
    }
    /** test driver **/
    if (!strcmp(command, "status")) {
        PX4_INFO("device status:");
        statusDisplayTime = secs * STATUS_UPDATE_RATE;
        exit(OK);
    }
    /** display usage info **/
    uavlas_usage();
    exit(0);
}
}
