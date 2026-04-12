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

// uORB messaging headers
#include <uORB/topics/depth_sensor.h>
#include <uORB/topics/sensor_baro.h>

extern "C" __EXPORT int depth_sensor_main(int argc, char* argv[]);

/* =========================================================================
 * System state and physical configuration parameters (thread-safe)
 * ========================================================================= */
static volatile bool thread_should_exit = false;
static volatile bool thread_running     = false;
static volatile bool request_tare       = false;

// Core feature switch: controls whether advanced composite filtering is enabled
static volatile bool enable_filter = true;

// Fluid physics constants and compensation parameters
static volatile float fluid_density_kg_m3 = 997.0f;
static volatile float surface_pressure_pa = 101325.0f;
static volatile float offset_depth_m      = 0.0f;
static volatile float offset_temp_c       = 0.0f;

// Target barometer instance
static int target_baro_instance = 1;

/* =========================================================================
 * Composite filter configuration (Median rejection + 1D Kalman smoothing)
 * ========================================================================= */
#define MEDIAN_WINDOW_SIZE 5 // Median window size; 5 is optimal for rejecting spikes with minimal computation

static const float KALMAN_Q = 0.5f; // Process noise: expected system dynamics (larger = faster response)
static const float KALMAN_R = 50.0f; // Measurement noise: sensor uncertainty (larger = smoother output)

static const float TEMP_LPF_ALPHA = 0.1f; // Temperature low-pass filter coefficient

/* =========================================================================
 * Function declarations
 * ========================================================================= */
int depth_sensor_thread_main(int argc, char* argv[]);
static void usage(const char* reason);

/* =========================================================================
 * CLI usage information
 * ========================================================================= */
static void usage(const char* reason)
{
  if (reason)
  {
    PX4_WARN("%s", reason);
  }
  fprintf(stderr, "usage: depth_sensor {start|stop|status|tare|filter|offsetD|offsetT|density}\n");
  fprintf(stderr, "  start [-i <baro_instance>]  (default instance is 1)\n");
  fprintf(stderr, "  filter <on|off>             (Enable or bypass the Median+Kalman filter)\n");
  fprintf(stderr, "  tare                        (Set current active pressure as 0 meter depth)\n");
  fprintf(stderr, "  offsetD <val>               (Add offset to calculated depth in meters)\n");
  fprintf(stderr, "  offsetT <val>               (Add offset to measured temperature in Celsius)\n");
  fprintf(stderr, "  density <val>               (Set fluid density in kg/m^3)\n");
}

/* =========================================================================
 * Main entry and command dispatcher
 * ========================================================================= */
int depth_sensor_main(int argc, char* argv[])
{
  if (argc < 2)
  {
    usage("missing command");
    return 1;
  }

  /* Start thread */
  if (!strcmp(argv[1], "start"))
  {
    if (thread_running)
    {
      PX4_INFO("depth_sensor is already running");
      return 0;
    }
    if (argc >= 4 && !strcmp(argv[2], "-i"))
    {
      target_baro_instance = atoi(argv[3]);
    }
    thread_should_exit = false;
    px4_task_spawn_cmd("depth_sensor", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2000, depth_sensor_thread_main, NULL);
    return 0;
  }

  /* Stop thread */
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

  /* Status query */
  if (!strcmp(argv[1], "status"))
  {
    PX4_INFO("Status: %s", thread_running ? "Running" : "Stopped");
    PX4_INFO("Baro Instance: %d | Advanced Filter: %s", target_baro_instance, enable_filter ? "ON" : "OFF");
    PX4_INFO("Config -> Density: %.1f kg/m^3", (double) fluid_density_kg_m3);
    PX4_INFO("Config -> Surface Pressure Zero: %.2f Pa", (double) surface_pressure_pa);
    PX4_INFO("Config -> Offset Depth: %.2f m | Offset Temp: %.2f C", (double) offset_depth_m, (double) offset_temp_c);
    return 0;
  }

  /* Filter bypass control */
  if (!strcmp(argv[1], "filter"))
  {
    if (argc >= 3)
    {
      if (!strcmp(argv[2], "on"))
      {
        enable_filter = true;
        PX4_INFO("Advanced Filter (Median + Kalman) is now ON");
        return 0;
      }
      else if (!strcmp(argv[2], "off"))
      {
        enable_filter = false;
        PX4_INFO("Advanced Filter is now OFF. Publishing RAW data.");
        return 0;
      }
    }
    usage("filter command requires 'on' or 'off'");
    return 1;
  }

  /* Tare trigger */
  if (!strcmp(argv[1], "tare"))
  {
    if (!thread_running)
    {
      PX4_WARN("depth_sensor must be running to perform tare");
      return 1;
    }
    request_tare = true;
    PX4_INFO("Tare requested. Zeroing depth against current environmental pressure...");
    return 0;
  }

  /* Parameter adjustments */
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
 * Core processing thread: subscription, filtering, and physical modeling
 * ========================================================================= */
int depth_sensor_thread_main(int argc, char* argv[])
{
  thread_running = true;

  /* 1. Subscribe to uORB topic */
  int baro_sub_fd = orb_subscribe_multi(ORB_ID(sensor_baro), target_baro_instance);
  if (baro_sub_fd < 0)
  {
    PX4_ERR("Failed to subscribe to sensor_baro instance %d", target_baro_instance);
    thread_running = false;
    return -1;
  }

  /* 2. Prepare publication structure */
  struct depth_sensor_s report;
  memset(&report, 0, sizeof(report));
  orb_advert_t depth_pub = orb_advertise(ORB_ID(depth_sensor), &report);

  /* 3. Setup polling */
  px4_pollfd_struct_t fds[1];
  fds[0].fd     = baro_sub_fd;
  fds[0].events = POLLIN;

  /* =======================================
   * Filter internal state allocation
   * ======================================= */
  float median_buffer[MEDIAN_WINDOW_SIZE] = {0};
  int median_idx                          = 0;
  bool filter_initialized                 = false;

  // 1D Kalman filter state
  float kalman_x = 0.0f; // Estimated state
  float kalman_p = 1.0f; // Estimate covariance

  float final_filtered_temp = 0.0f;

  PX4_INFO("Depth translation engine started. Target instance: %d", target_baro_instance);

  /* =======================================
   * Event-driven main loop
   * ======================================= */
  while (!thread_should_exit)
  {
    int poll_ret = px4_poll(fds, 1, 500);

    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      struct sensor_baro_s baro_data;
      orb_copy(ORB_ID(sensor_baro), baro_sub_fd, &baro_data);

      /* Raw sensor data */
      float raw_pressure_pa = baro_data.pressure;
      float raw_temp_c      = baro_data.temperature;

      /* Active data path (raw or filtered) */
      float active_pressure = raw_pressure_pa;
      float active_temp     = raw_temp_c;

      /* =======================================================
       * Composite filtering pipeline (Median + Kalman)
       * ======================================================= */
      if (enable_filter)
      {
        // Initialize filter to avoid startup spikes
        if (!filter_initialized)
        {
          for (int i = 0; i < MEDIAN_WINDOW_SIZE; i++)
          {
            median_buffer[i] = raw_pressure_pa;
          }
          kalman_x            = raw_pressure_pa;
          kalman_p            = 1.0f;
          final_filtered_temp = raw_temp_c;
          filter_initialized  = true;
        }

        /* --- Stage 1: Median filter (outlier rejection) --- */
        median_buffer[median_idx] = raw_pressure_pa;
        median_idx                = (median_idx + 1) % MEDIAN_WINDOW_SIZE;

        float sort_buffer[MEDIAN_WINDOW_SIZE];
        memcpy(sort_buffer, median_buffer, sizeof(median_buffer));

        // Bubble sort (efficient for N=5)
        for (int i = 0; i < MEDIAN_WINDOW_SIZE - 1; i++)
        {
          for (int j = 0; j < MEDIAN_WINDOW_SIZE - i - 1; j++)
          {
            if (sort_buffer[j] > sort_buffer[j + 1])
            {
              float temp         = sort_buffer[j];
              sort_buffer[j]     = sort_buffer[j + 1];
              sort_buffer[j + 1] = temp;
            }
          }
        }

        float measurement_z = sort_buffer[MEDIAN_WINDOW_SIZE / 2];

        /* --- Stage 2: 1D Kalman filter (noise smoothing) --- */
        float p_predict = kalman_p + KALMAN_Q;

        float K = p_predict / (p_predict + KALMAN_R);

        kalman_x = kalman_x + K * (measurement_z - kalman_x);
        kalman_p = (1.0f - K) * p_predict;

        /* --- Temperature low-pass filter --- */
        final_filtered_temp = (TEMP_LPF_ALPHA * raw_temp_c) + ((1.0f - TEMP_LPF_ALPHA) * final_filtered_temp);

        active_pressure = kalman_x;
        active_temp     = final_filtered_temp;
      }
      else
      {
        // Reset filter so it reinitializes correctly when re-enabled
        filter_initialized = false;
      }

      /* =======================================================
       * Tare (zeroing) logic
       * ======================================================= */
      if (request_tare)
      {
        surface_pressure_pa = active_pressure;
        request_tare        = false;
        PX4_INFO("Zeroed! New surface pressure baseline: %.2f Pa", (double) surface_pressure_pa);
      }

      /* =======================================================
       * Hydrostatic depth calculation
       * ======================================================= */
      float final_temp = active_temp + offset_temp_c;

      // Formula: h = (P_sensor - P_surface) / (rho * g)
      float depth_calculated = (active_pressure - surface_pressure_pa) / (fluid_density_kg_m3 * 9.80665f);

      float final_depth = depth_calculated + offset_depth_m;

      /* =======================================================
       * Publish result
       * ======================================================= */
      report.timestamp     = hrt_absolute_time();
      report.depth_m       = final_depth;
      report.temperature_c = final_temp;

      orb_publish(ORB_ID(depth_sensor), depth_pub, &report);
    }
    else if (poll_ret == 0)
    {
      // Optional watchdog timeout handling
    }
  }

  /* =======================================================
   * Cleanup and exit
   * ======================================================= */
  PX4_INFO("Depth translation engine stopping...");
  orb_unsubscribe(baro_sub_fd);
  if (depth_pub != nullptr)
  {
    orb_unadvertise(depth_pub);
  }

  thread_running = false;
  return 0;
}
