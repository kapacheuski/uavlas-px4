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
// Global functions
void uavlas_usage();
bool check_device(int bus);

extern "C" __EXPORT int uavlas_main(int argc, char *argv[]);

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
