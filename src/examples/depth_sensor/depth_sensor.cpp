/****************************************************************************
 *
 * Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
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

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

// uORB message headers
#include <uORB/topics/depth_sensor.h> // Custom data source to be published
#include <uORB/topics/sensor_baro.h> // Official MS5837 published data source

extern "C" __EXPORT int depth_sensor_main(int argc, char* argv[]);

/* =========================================================================
 * Global state and configuration variables (use volatile for cross-thread visibility)
 * ========================================================================= */
static volatile bool thread_should_exit = false;
static volatile bool thread_running     = false;
static volatile bool request_tare       = false; // Used to trigger surface zero calibration

// Physical and calibration parameters
static volatile float fluid_density_kg_m3 = 997.0f; // Default freshwater density
static volatile float surface_pressure_pa = 101325.0f; // Baseline absolute pressure in air (default at startup)
static volatile float offset_depth_m      = 0.0f; // Software depth offset
static volatile float offset_temp_c       = 0.0f; // Software temperature offset

// Target sensor instance (default is 1, instance 0 is usually used by the flight controller)
static int target_baro_instance = 1;

/* =========================================================================
 * Filter parameter configuration
 * ========================================================================= */
#define FILTER_WINDOW_SIZE 10 // Sliding window size: removes high-frequency noise (electrical noise)
static const float LPF_ALPHA = 0.15f; // First-order low-pass filter coefficient (0.0~1.0): smoother with smaller value
                                      // but more delay

/* =========================================================================
 * Function declarations
 * ========================================================================= */
int depth_sensor_thread_main(int argc, char* argv[]);
static void usage(const char* reason);

/* =========================================================================
 * Command line help information
 * ========================================================================= */
static void usage(const char* reason)
{
  if (reason)
  {
    PX4_WARN("%s", reason);
  }
  fprintf(stderr, "usage: depth_sensor {start|stop|status|tare|offsetD <val>|offsetT <val>|density <val>}\n");
  fprintf(stderr, "  start [-i <baro_instance>]  (default instance is 1)\n");
  fprintf(stderr, "  tare                        (Set current filtered pressure as 0 meter depth)\n");
  fprintf(stderr, "  offsetD <val>               (Add an offset to calculated depth in meters)\n");
  fprintf(stderr, "  offsetT <val>               (Add an offset to measured temperature in Celsius)\n");
  fprintf(stderr, "  density <val>               (Set fluid density in kg/m^3, e.g., 1025 for seawater)\n");
}

/* =========================================================================
 * Main entry and command-line parsing
 * ========================================================================= */
int depth_sensor_main(int argc, char* argv[])
{
  if (argc < 2)
  {
    usage("missing command");
    return 1;
  }

  /* Start background thread */
  if (!strcmp(argv[1], "start"))
  {
    if (thread_running)
    {
      PX4_INFO("depth_sensor is already running");
      return 0;
    }

    // Parse instance parameter, e.g.: depth_sensor start -i 1
    if (argc >= 4 && !strcmp(argv[2], "-i"))
    {
      target_baro_instance = atoi(argv[3]);
    }

    thread_should_exit = false;
    px4_task_spawn_cmd("depth_sensor", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2000, depth_sensor_thread_main, NULL);
    return 0;
  }

  /* Stop background thread */
  if (!strcmp(argv[1], "stop"))
  {
    if (!thread_running)
    {
      PX4_INFO("depth_sensor is not running");
      return 0;
    }
    thread_should_exit = true;
    return 0;
  }

  /* Check current status and parameters */
  if (!strcmp(argv[1], "status"))
  {
    PX4_INFO("Status: %s", thread_running ? "Running" : "Stopped");
    PX4_INFO("Baro Instance Listening: %d", target_baro_instance);
    PX4_INFO("Config -> Density: %.1f kg/m^3", (double) fluid_density_kg_m3);
    PX4_INFO("Config -> Surface Pressure Zero: %.2f Pa", (double) surface_pressure_pa);
    PX4_INFO("Config -> Offset Depth: %.2f m | Offset Temp: %.2f C", (double) offset_depth_m, (double) offset_temp_c);
    return 0;
  }

  /* Trigger zero calibration (send this command in air / above water surface) */
  if (!strcmp(argv[1], "tare"))
  {
    if (!thread_running)
    {
      PX4_WARN("depth_sensor must be running to perform tare");
      return 1;
    }
    request_tare = true;
    PX4_INFO("Tare requested. Background thread will zero the depth shortly...");
    return 0;
  }

  /* Modify runtime parameters */
  if (argc >= 3)
  {
    if (!strcmp(argv[1], "offsetD"))
    {
      offset_depth_m = strtof(argv[2], NULL);
      PX4_INFO("Depth Offset applied: %.2f m", (double) offset_depth_m);
      return 0;
    }
    else if (!strcmp(argv[1], "offsetT"))
    {
      offset_temp_c = strtof(argv[2], NULL);
      PX4_INFO("Temp Offset applied: %.2f C", (double) offset_temp_c);
      return 0;
    }
    else if (!strcmp(argv[1], "density"))
    {
      fluid_density_kg_m3 = strtof(argv[2], NULL);
      PX4_INFO("Fluid Density applied: %.1f kg/m^3", (double) fluid_density_kg_m3);
      return 0;
    }
  }

  usage("unrecognized command");
  return 1;
}

/* =========================================================================
 * Background core data processing thread
 * ========================================================================= */
int depth_sensor_thread_main(int argc, char* argv[])
{
  thread_running = true;

  /* 1. Subscribe to sensor_baro topic published by MS5837 (specific instance) */
  int baro_sub_fd = orb_subscribe_multi(ORB_ID(sensor_baro), target_baro_instance);
  if (baro_sub_fd < 0)
  {
    PX4_ERR("Failed to subscribe to sensor_baro instance %d", target_baro_instance);
    thread_running = false;
    return -1;
  }

  /* 2. Prepare the custom topic to be published */
  struct depth_sensor_s report;
  memset(&report, 0, sizeof(report));
  orb_advert_t depth_pub = orb_advertise(ORB_ID(depth_sensor), &report);

  /* 3. Configure poll wait handle */
  px4_pollfd_struct_t fds[1];
  fds[0].fd     = baro_sub_fd;
  fds[0].events = POLLIN;

  /* =======================================
   * Filter state variable initialization
   * ======================================= */
  float pressure_history[FILTER_WINDOW_SIZE] = {0};
  int history_idx                            = 0;
  bool filter_initialized                    = false;

  float final_filtered_pressure = 0.0f;
  float final_filtered_temp     = 0.0f;

  PX4_INFO("Depth translation engine started. Target instance: %d", target_baro_instance);

  /* =======================================
   * Thread main loop
   * ======================================= */
  while (!thread_should_exit)
  {
    // Suspend thread and wait for data arrival, timeout set to 500 ms
    int poll_ret = px4_poll(fds, 1, 500);

    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      struct sensor_baro_s baro_data;
      // Safely copy data from kernel pipe
      orb_copy(ORB_ID(sensor_baro), baro_sub_fd, &baro_data);

      /* Get raw data (units: Pascal Pa and Celsius C) */
      float raw_pressure_pa = baro_data.pressure;
      float raw_temp_c      = baro_data.temperature;

      /* =======================================================
       * Filter computation logic
       * ======================================================= */
      // Initialize filter array to avoid ramp-up from zero at startup
      if (!filter_initialized)
      {
        for (int i = 0; i < FILTER_WINDOW_SIZE; i++)
        {
          pressure_history[i] = raw_pressure_pa;
        }
        final_filtered_pressure = raw_pressure_pa;
        final_filtered_temp     = raw_temp_c;
        filter_initialized      = true;
      }

      // Stage 1: Sliding window mean filter (remove extreme noise)
      pressure_history[history_idx] = raw_pressure_pa;
      history_idx                   = (history_idx + 1) % FILTER_WINDOW_SIZE;

      float window_sum = 0.0f;
      for (int i = 0; i < FILTER_WINDOW_SIZE; i++)
      {
        window_sum += pressure_history[i];
      }
      float moving_avg_pressure = window_sum / FILTER_WINDOW_SIZE;

      // Stage 2: First-order low-pass filter EMA (adds inertia, smooth output)
      final_filtered_pressure = (LPF_ALPHA * moving_avg_pressure) + ((1.0f - LPF_ALPHA) * final_filtered_pressure);
      final_filtered_temp     = (LPF_ALPHA * raw_temp_c) + ((1.0f - LPF_ALPHA) * final_filtered_temp);
      /* ======================================================= */

      /* =======================================================
       * Zero calibration logic (Tare)
       * ======================================================= */
      if (request_tare)
      {
        // Lock current filtered pressure as new surface absolute pressure
        surface_pressure_pa = final_filtered_pressure;
        request_tare        = false;
        PX4_INFO("Zeroed! New surface pressure calibrated at: %.2f Pa", (double) surface_pressure_pa);
      }

      /* =======================================================
       * Physical conversion logic (fluid statics)
       * ======================================================= */
      float final_temp = final_filtered_temp + offset_temp_c;

      // h = (P_sensor - P_surface) / (rho * g)
      float depth_calculated = (final_filtered_pressure - surface_pressure_pa) / (fluid_density_kg_m3 * 9.80665f);

      float final_depth = depth_calculated + offset_depth_m;

      /* =======================================================
       * Packaging and publishing
       * ======================================================= */
      report.timestamp     = hrt_absolute_time();
      report.depth_m       = final_depth;
      report.temperature_c = final_temp;

      orb_publish(ORB_ID(depth_sensor), depth_pub, &report);
    }
    else if (poll_ret == 0)
    {
      // Poll timeout: no sensor data received for a long time, useful for debugging wiring issues
      // PX4_WARN("Poll timeout: No data from MS5837 for 500ms");
    }
  }

  /* Cleanup on exit */
  PX4_INFO("Depth translation engine stopping...");
  orb_unsubscribe(baro_sub_fd);
  if (depth_pub != nullptr)
  {
    orb_unadvertise(depth_pub);
  }

  thread_running = false;
  return 0;
}
