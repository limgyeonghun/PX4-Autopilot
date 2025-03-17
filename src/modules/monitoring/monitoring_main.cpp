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
 * @file monitoring_main.c
 * monitoring module
 * @author SungTae Moon <munhoney@gmail.com>
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <ctype.h>
#include <systemlib/err.h>

//#include <px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <pthread.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/piksi_rtk.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/monitoring.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_preflight_mag.h>

#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <drivers/drv_hrt.h>



namespace monitoring
{

    extern "C" __EXPORT int monitoring_main(int argc, char *argv[]);


    class Monitoring
    {
    private:
        enum {
            SAFETY_LOCK_STATUS = 0,
            ARM_STATUS,
            OFFBOARD_MODE,
            MANUAL_MODE,

            AUTO_MODE,
            FAIL_SAFE_MODE,
            BATTERY_PROBLEM,
            RTKGPS_CONNECTION,

            RTKGPS_BASE_RECV,
            RTKGPS_FIXED_MODE,
            RTKGPS_OFFSET,
            COMM_PROBLEM,

            INIT_PITCH_PROBLEM,
            INIT_ROLL_PROBLEM,
            INIT_VELX_PROBLEM,
            INIT_VELY_PROBLEM,

            INIT_VELZ_PROBLEM,
            INIT_EMBEDDED_SC_OFFSET,
            INIT_EMBEDDED_SC_FILE,
            INIT_EMBEDDED_SC_START_TIME,

            PERI_5V_POWER_PROBLEM,
            TEMPERATURE_PROBLEM,

            MAG_INCONSISTENT_PROBLEM,
            ACC_INCONSISTENT_PROBLEM,
            GYR_INCONSISTENT_PROBLEM,
            AGE_CORR_LV1_PROBLEM,

            AGE_CORR_LV2_PROBLEM,

            NUM_OF_MONITORING
        };
    public:
        Monitoring();
        ~Monitoring();

        /* Start the offboard monitoring player
         *
         * @return 0 if successfull, -1 on error. */
        int start();

        /* Stop the load monitoring */
        void stop();

        bool isRunning() { return _task_is_running; }

        void printStatus();

    private:

        /**
         * Shim for calling task_main from task_create.
         */
        static int task_main_trampoline(int argc, char *argv[]);

        /**
         * Main sensor collection task.
         */
        void task_main();

        void update_subscribe();

        void monitoring();

        void publish();

        uint32_t status1();

        uint32_t status2();

        bool comm_problem();
        bool temperature_problem();

    private:

        int		_control_task;			/**< task handle for task */
        bool	_task_should_exit;		/**< if true, task should exit */
        bool	_task_is_running;

        int     _vehicle_status_sub;
        int     _vehicle_attitude_sub;
        int		_local_pos_sub;			/**< vehicle local position */
        int		_cmd_sub;
        int		_vehicle_gps_position_sub;
        int		_piksi_rtk_sub;
        int		_sensors_sub;
        int     _system_power_sub;
        int     _battery_status_sub;

        orb_advert_t                         _monitoring_pub;

        struct vehicle_command_s			_cmd;
        struct vehicle_attitude_s           _att;
        struct sensor_gps_s		_gps;
        struct piksi_rtk_s					_piksi_rtk;
        struct vehicle_local_position_s     _local_pos;
        struct sensor_combined_s			_sensors;
        struct vehicle_command_s			_vcmd;
        struct vehicle_status_s             _status;
        struct monitoring_s                 _monitoring;
        struct system_power_s               _system_power;
        struct battery_status_s             _battery_status;

        hrt_abstime                         _mavlink_recv_time;

        float								_com_arm_imu_acc;
        float								_com_arm_imu_gyr;

    };

    static Monitoring* g_monitoring = nullptr;


    Monitoring::Monitoring() :
            _control_task(-1),
            _task_should_exit(false),
            _task_is_running(false),
            _vehicle_status_sub(-1),
            _vehicle_attitude_sub(-1),
            _local_pos_sub(-1),
            _cmd_sub(-1),
            _vehicle_gps_position_sub(-1),
            _piksi_rtk_sub(-1),
            _sensors_sub(-1),
            _system_power_sub(-1),
            _battery_status_sub(-1),
            _monitoring_pub(nullptr),
            _mavlink_recv_time(0L),
            _com_arm_imu_acc(0.0f),
            _com_arm_imu_gyr(0.0f)
    {

    }

    Monitoring::~Monitoring()
    {
        if (_control_task != -1) {
            /* task wakes up every 100ms or so at the longest */
            _task_should_exit = true;

            /* wait for a second for the task to quit at our request */
            unsigned i = 0;

            do {
                /* wait 20ms */
                usleep(20000);

                /* if we have given up, kill it */
                if (++i > 50) {
                    px4_task_delete(_control_task);
                    break;
                }
            } while (_control_task != -1);
        }
    }

    int Monitoring::start()
    {
//        MAVLINK_ASSERT(_control_task == -1);

        /* start the task */
        _control_task = px4_task_spawn_cmd("monitoring",
                                           SCHED_DEFAULT,
                                           SCHED_PRIORITY_MAX-5,
                                           1900,
                                           Monitoring::task_main_trampoline,
                                           nullptr);

        if (_control_task < 0) {
            PX4_WARN("task start failed");
            return -errno;
        }

        return OK;
    }

    void Monitoring::stop()
    {
        _task_should_exit = true;
    }

    void Monitoring::printStatus()
    {
        hrt_abstime t = hrt_absolute_time();

        PX4_INFO("TOW : %lu", _monitoring.tow);             //sitl : %u, fmu-v3 : %lu
        PX4_INFO("Battery : %d", _monitoring.battery);
        PX4_INFO("Position : %.2f, %.2f, %.2f", (double)_monitoring.pos_x, (double)_monitoring.pos_y, (double)_monitoring.pos_z);
        PX4_INFO("status : %0lx:", _monitoring.status1);    //sitl : %0x, fmu-v3 : %0lx
        PX4_INFO("RTK flag(%d), sat(%d/%d)", _piksi_rtk.flag, _piksi_rtk.nobs_base, _piksi_rtk.nobs_rover);
        PX4_INFO("Velocity: %.2f, %.2f, %.2f", (double)_local_pos.vx, (double)_local_pos.vy, (double)_local_pos.vz);
        PX4_INFO("Mavlink received time: %.2f", (double)(t - _mavlink_recv_time)/1000000.);
    }

    int Monitoring::task_main_trampoline(int argc, char *argv[])
    {
        g_monitoring->task_main();
        return 0;
    }

    void Monitoring::update_subscribe()
    {
        bool updated;

        /* vehicle status */
        orb_check(_vehicle_status_sub, &updated);
        if ( updated ) {
            orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_status);
        }

        orb_check(_vehicle_attitude_sub, &updated);
        if ( updated ) {
            orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);
        }

        /* piksi RTK position */
        orb_check(_piksi_rtk_sub, &updated);
        if ( updated ) {
            orb_copy(ORB_ID(piksi_rtk), _piksi_rtk_sub, &_piksi_rtk);
        }

        /* local position */
        orb_check(_local_pos_sub, &updated);
        if ( updated ) {
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
        }

        /* vehicle command */
        orb_check(_cmd_sub, &updated);
        if (updated) {
            /* got command */
            orb_copy(ORB_ID(vehicle_command), _cmd_sub, &_cmd);
        }

        /* sensor */
        orb_check(_sensors_sub, &updated);
        if ( updated) {
            /* got sensors */
            orb_copy(ORB_ID(sensor_combined), _sensors_sub, &_sensors);
        }

        /* system power status */
        orb_check(_system_power_sub, &updated);
        if ( updated) {
            /* got power status */
            orb_copy(ORB_ID(system_power), _system_power_sub, &_system_power);
        }

        /* battery status */
        orb_check(_battery_status_sub, &updated);
        if ( updated ) {
            /* got mavlink status */
            orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
        }

    }

    void Monitoring::monitoring()
    {
        _monitoring.timestamp = hrt_absolute_time();
        _monitoring.tow  = _piksi_rtk.tow;
        _monitoring.pos_x = _local_pos.x;
        _monitoring.pos_y = _local_pos.y;
        _monitoring.pos_z = _local_pos.z;
        _monitoring.head = _local_pos.heading;
        _monitoring.vx = _local_pos.vx;
        _monitoring.vy = _local_pos.vy;
        _monitoring.vz = _local_pos.vz;

        const matrix::Eulerf euler = matrix::Quatf(_att.q);
        _monitoring.roll = euler.phi();
        _monitoring.pitch = euler.theta();
        _monitoring.yaw = euler.psi();

        _monitoring.status1 = status1();
        _monitoring.status2 = status2();
        _monitoring.rtk_nbase = _piksi_rtk.nobs_base;
        _monitoring.rtk_nrover = _piksi_rtk.nobs_rover;
        _monitoring.battery = _battery_status.remaining > 0.0f ? _battery_status.remaining*100.0f : 0;

        _monitoring.rtk_n = _piksi_rtk.n;
        _monitoring.rtk_e = _piksi_rtk.e;
        _monitoring.rtk_d = _piksi_rtk.d;
    }

    void Monitoring::publish()
    {
        if (_monitoring_pub != nullptr) {
            orb_publish(ORB_ID(monitoring), _monitoring_pub,  &_monitoring);

        } else {
            _monitoring_pub = orb_advertise(ORB_ID(monitoring), &_monitoring);
        }
    }

    uint32_t Monitoring::status1()
    {
        uint32_t status = 0;
        const uint8_t ARMING_STATE_ARMED = 2;

        status += !_status.safety_off ? 1 << SAFETY_LOCK_STATUS : 0;
        status += _status.arming_state == ARMING_STATE_ARMED ? 1 << ARM_STATUS : 0;
        status += _status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL ? 1  << MANUAL_MODE : 0;
        status += _status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD ? 1  << OFFBOARD_MODE : 0;
        status += _status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ? 1  << AUTO_MODE : 0;
        status += _status.failsafe == true ? 1 << FAIL_SAFE_MODE : 0;
        float bat_remain = _monitoring.battery = _battery_status.remaining > 0.0f ? _battery_status.remaining*100.0f : 0;
        status += bat_remain < 25.0f ? 1 << BATTERY_PROBLEM : 0;

        status += 0 == 0 ? 1 << RTKGPS_CONNECTION : 0;
        status += 0 == 0 ? 1 << RTKGPS_BASE_RECV : 0;
        status += _piksi_rtk.flag == 4 ? 1 << RTKGPS_FIXED_MODE : 0;
        status += 0 == 0 ? 1 << RTKGPS_OFFSET : 0;

        if ( this->comm_problem() ) {
            status += 1 << COMM_PROBLEM;
        }

//        if ( _status.arming_state != ARMING_STATE_ARMED ) {
//
//            matrix::Quaternion<float> q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
//            matrix::Matrix<float, 3, 3> _R = q_att.to_dcm();
//            matrix::Vector<float, 3> euler_angles;
//            euler_angles = _R.to_euler();
//            float roll = euler_angles(0);
//            float pitch = euler_angles(1);
//
//            const float RAD2DEG	= 57.0f;
//            status += fabsf(pitch*RAD2DEG) > 10.0f ? 1 << INIT_PITCH_PROBLEM : 0;
//            status += fabsf(roll*RAD2DEG)  > 10.0f ? 1 << INIT_ROLL_PROBLEM : 0;
//            status += fabsf(_local_pos.vx) > 0.3f ? 1 << INIT_VELX_PROBLEM : 0;
//            status += fabsf(_local_pos.vy) > 0.3f ? 1 << INIT_VELY_PROBLEM : 0;
//            status += fabsf(_local_pos.vz) > 0.1f ? 1 << INIT_VELZ_PROBLEM : 0;
//        }

        if ( _system_power.voltage5v_v < 4.9f || _system_power.voltage5v_v > 5.4f ) {
            status += 1 << PERI_5V_POWER_PROBLEM;
        }

        if ( this->temperature_problem() ) {
            status += 1 << TEMPERATURE_PROBLEM;
        }

        if ( _piksi_rtk.age_corr > 100 ) {
            status += 1 << AGE_CORR_LV1_PROBLEM;
        }

        if ( _piksi_rtk.age_corr > 200 ) {
            status += 1 << AGE_CORR_LV2_PROBLEM;
        }

        return status;
    }

    uint32_t Monitoring::status2()
    {
        // RESERVED
        return 0;
    }

    bool Monitoring::comm_problem()
    {
        hrt_abstime t = hrt_absolute_time();

        // if the time of last received mavlink data is postponed by 5 sec,
        // it regards as commnunication problem

        if ( (t - _piksi_rtk.timestamp)/1000000 > 5 ) {
            return true;
        }
        else {
            return false;
        }
    }

    bool Monitoring::temperature_problem()
    {
        // TODO :check it
        //float accel_temp = _sensors.accelerometer_temp[0];
        //float gyro_temp = _sensors.gyro_temp[0];  (not used)
        //float mag_temp = _sensors.magnetometer_temp[0];
        //float baro_temp = _sensors.baro_temp_celcius[0];
        //bool lower_min = accel_temp < 10.0f || mag_temp < 10.0f || baro_temp < 10.0f;
        //bool higher_max = accel_temp > 60.0f || mag_temp > 60.0f || baro_temp > 60.0f;

        //return lower_min || higher_max;
        return  false;
    }

    void Monitoring::task_main()
    {
        /*
         * do subscriptions
         */

        _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
        _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
        _cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
        _vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
        _piksi_rtk_sub = orb_subscribe(ORB_ID(piksi_rtk));
        _sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
        _system_power_sub = orb_subscribe((ORB_ID(system_power)));
        _battery_status_sub = orb_subscribe((ORB_ID(battery_status)));

        /* init variable */
        memset(&_cmd, 0, sizeof(_cmd));
        memset(&_att, 0, sizeof(_att));
        memset(&_gps, 0, sizeof(_gps));
        memset(&_piksi_rtk, 0, sizeof(_piksi_rtk));
        memset(&_local_pos, 0, sizeof(_local_pos));
        memset(&_sensors, 0, sizeof(_sensors));
        memset(&_vcmd, 0, sizeof(_vcmd));
        memset(&_monitoring, 0, sizeof(_monitoring));
        memset(&_system_power, 0, sizeof(_system_power));
        memset(&_battery_status, 0, sizeof(_battery_status));


        // check parameter
        param_get(param_find("COM_ARM_IMU_ACC"), &_com_arm_imu_acc);
        param_get(param_find("COM_ARM_IMU_GYR"), &_com_arm_imu_gyr);

        // wait during 1s to read and buffer monitoring file
        usleep(1000000);

        _task_is_running = true;

        /* wakeup source */
        px4_pollfd_struct_t fds[1] = {};
        fds[0].fd = _sensors_sub;
        fds[0].events = POLLIN;

        while (!_task_should_exit) {

            /* wait for up to 50ms for data */
            int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

            /* timed out - periodic check for _task_should_exit */
            if (pret == 0) {
                // Go through the loop anyway to copy manual input at 50 Hz.
            }
                /* this is undesirable but not much we can do */
            else if (pret < 0) {
                PX4_WARN("poll error %d, %d", pret, errno);
            }
            else {
                /* update subscribe */
                update_subscribe();

                /* monitoring */
                monitoring();

                /* publish */
                publish();
            }
            px4_usleep(1000*1e3); //1hz

        }

        _task_is_running = false;

    }

/**
 * Main function
 */
    int monitoring_thread_main(int argc, char *argv[]);


/**
 * Print the correct usage.
 */
    static void usage(const char *reason);

    static void usage(const char *reason)
    {
        if (reason) {
            PX4_ERR("%s\n", reason);
        }

        PX4_INFO("usage: monitoring {start|stop|status} \n\n");
    }

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
    int monitoring_main(int argc, char *argv[])
    {
        if (argc < 2) {
            usage("missing command");
            return 1;
        }

        if (!strcmp(argv[1], "start")) {

            if (g_monitoring != nullptr && g_monitoring->isRunning()) {
                PX4_WARN("already running");
                /* this is not an error */
                return 0;
            }

            g_monitoring = new Monitoring();

            // Check if alloc worked.
            if (g_monitoring == nullptr) {
                PX4_ERR("alloc failed");
                return -1;
            }

            int ret = g_monitoring->start();

            if (ret != 0) {
                PX4_ERR("start failed");
            }

            return 0;

        }

        if (!strcmp(argv[1], "stop")) {

            if (g_monitoring == nullptr || g_monitoring->isRunning()) { //need check
                PX4_WARN("not running");
                /* this is not an error */
                return 0;
            }

            g_monitoring->stop();

            // Wait for task to die
            int i = 0;

            do {
                /* wait up to 3s */
                usleep(100000);

            } while (g_monitoring->isRunning() && ++i < 30);

            delete g_monitoring;
            g_monitoring = nullptr;

            return 0;
        }


        if (!strcmp(argv[1], "status")) {
            if (g_monitoring != nullptr && g_monitoring->isRunning()) {
                PX4_INFO("running\n");
                if ( g_monitoring != NULL )  {
                    g_monitoring->printStatus();
                }

            } else {
                PX4_INFO("not running\n");
            }


            return 0;
        }

        usage("unrecognized command");
        return 1;
    }

}	// end monitoring namespace
