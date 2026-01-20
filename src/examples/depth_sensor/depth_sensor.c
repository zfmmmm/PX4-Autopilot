#include <drivers/drv_hrt.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <uORB/topics/depth_sensor.h>
#include <uORB/uORB.h>

__EXPORT int depth_sensor_main(int argc, char *argv[]);

/*
 * Use volatile to ensure visibility across threads.
 * Increase buffer sizes to prevent overflow.
 */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static volatile bool reset_flag = false;
static volatile bool offset_d_set = false;
static volatile bool offset_t_set = false;
static volatile bool density_set = false;

/*
 * Command buffers:
 * !D+xx.xx\r\n
 * !T+xx.xx\r\n
 * !Fxxxx\r\n
 */
static char cmd_buffer_d[20];
static char cmd_buffer_t[20];
static char cmd_buffer_density[20];

static int daemon_task;
static float last_depth = -1.0f;
static float last_temp = -1.0f;

// static const float DEPTH_THRESHOLD = 0.001f;
// static const float TEMP_THRESHOLD = 0.01f;

int depth_sensor_thread_main(int argc, char *argv[]);

/*
 * Print usage information
 */
static void usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s", reason);
  }
  fprintf(stderr, "usage: depth_sensor {start|stop|status|reset|offsetD "
                  "[+/-]xx.xx|offsetT [+/-]xx.xx|density xxxx}\n");
}

/*
 * Command-line entry point
 */
int depth_sensor_main(int argc, char *argv[]) {
  if (argc < 2) {
    usage("missing command");
    return 1;
  }

  /* Start background thread */
  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      PX4_INFO("already running");
      return 0;
    }
    thread_should_exit = false;
    daemon_task = px4_task_spawn_cmd(
        "depth_sensor", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2000,
        depth_sensor_thread_main, (argv) ? (char *const *)&argv[2] : NULL);
    return 0;
  }

  /* Stop background thread */
  if (!strcmp(argv[1], "stop")) {
    thread_should_exit = true;
    return 0;
  }

  /* Query running status */
  if (!strcmp(argv[1], "status")) {
    PX4_INFO("%s", thread_running ? "running" : "stopped");
    return 0;
  }

  /* Send reset command */
  if (!strcmp(argv[1], "reset")) {
    reset_flag = true;
    PX4_INFO("Reset flagged");
    return 0;
  }

  /* Parameter configuration commands */
  if (argc >= 3) {

    /* Set depth offset */
    if (!strcmp(argv[1], "offsetD")) {
      if (strlen(argv[2]) != 6) {
        usage("offset format must be [+/-]xx.xx (6 chars)");
        return 1;
      }
      snprintf(cmd_buffer_d, sizeof(cmd_buffer_d), "!D%s\r\n", argv[2]);
      offset_d_set = true;
      PX4_INFO("Depth Offset Sent: %s", cmd_buffer_d);
      return 0;

      /* Set temperature offset */
    } else if (!strcmp(argv[1], "offsetT")) {
      if (strlen(argv[2]) != 6) {
        usage("offset format must be [+/-]xx.xx (6 chars)");
        return 1;
      }
      snprintf(cmd_buffer_t, sizeof(cmd_buffer_t), "!T%s\r\n", argv[2]);
      offset_t_set = true;
      PX4_INFO("Temp Offset Set: %s", cmd_buffer_t);
      return 0;

      /* Set fluid density */
    } else if (!strcmp(argv[1], "density")) {
      if (strlen(argv[2]) != 4) {
        usage("density format must be xxxx (4 chars)");
        return 1;
      }
      snprintf(cmd_buffer_density, sizeof(cmd_buffer_density), "!F%s\r\n",
               argv[2]);
      density_set = true;
      PX4_INFO("Density Set: %s", cmd_buffer_density);
      return 0;
    }
  }

  usage("unrecognized command");
  return 1;
}

/*
 * Background thread: handles UART communication and uORB publishing
 */
int depth_sensor_thread_main(int argc, char *argv[]) {
  const char *uart_name = "/dev/ttyS3";
  int serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd < 0) {
    PX4_ERR("failed to open port");
    return -1;
  }

  /* UART configuration: 115200-8-N-1 */
  struct termios uart_config;
  tcgetattr(serial_fd, &uart_config);
  cfsetispeed(&uart_config, B115200);
  cfsetospeed(&uart_config, B115200);
  uart_config.c_cflag |= (CS8 | CLOCAL | CREAD);
  uart_config.c_iflag = IGNPAR;
  uart_config.c_oflag = 0;
  uart_config.c_lflag = 0;
  tcsetattr(serial_fd, TCSANOW, &uart_config);

  /* uORB publication setup */
  struct depth_sensor_s report;
  memset(&report, 0, sizeof(report));
  orb_advert_t depth_pub = orb_advertise(ORB_ID(depth_sensor), &report);

  thread_running = true;

  char rx_buf[128];   // Raw UART read buffer
  char frame_buf[64]; // Frame assembly buffer
  int frame_pos = 0;
  bool sync_found = false;

  px4_pollfd_struct_t fds[] = {{.fd = serial_fd, .events = POLLIN}};

  while (!thread_should_exit) {

    /* ---------- Handle outgoing control commands ---------- */

    if (reset_flag) {
      reset_flag = false;
      write(serial_fd, "!r\r\n", 4);
      PX4_INFO("Sensor Reset Sent");
    }

    if (offset_d_set) {
      write(serial_fd, cmd_buffer_d, strlen(cmd_buffer_d));
      offset_d_set = false;
    }

    if (offset_t_set) {
      write(serial_fd, cmd_buffer_t, strlen(cmd_buffer_t));
      offset_t_set = false;
      PX4_INFO("Temp Offset Sent: %s", cmd_buffer_t);
    }

    if (density_set) {
      write(serial_fd, cmd_buffer_density, strlen(cmd_buffer_density));
      density_set = false;
      PX4_INFO("Density Set: %s", cmd_buffer_density);
    }

    /* ---------- Read and parse incoming data ---------- */

    int ret = px4_poll(fds, 1, 10);

    if (ret > 0 && (fds[0].revents & POLLIN)) {
      ssize_t bytes_read = read(serial_fd, rx_buf, sizeof(rx_buf));

      for (ssize_t i = 0; i < bytes_read; i++) {
        char c = rx_buf[i];

        /* Frame synchronization: start at 'D' */
        if (c == 'D') {
          frame_pos = 0;
          sync_found = true;
        }

        if (sync_found) {
          frame_buf[frame_pos++] = c;

          /* End of frame detected at 'C' or buffer limit */
          if (c == 'C' || frame_pos >= (int)sizeof(frame_buf) - 1) {
            frame_buf[frame_pos] = '\0';

            char *end_ptr;

            /*
             * Example frame:
             * D=Depth:0.17m,T=Temp:23.5C
             * Parse depth starting from fixed offset
             */
            float current_depth = strtof(&frame_buf[6], &end_ptr);

            /* Find temperature field */
            char *t_start = strchr(end_ptr, 'T');

            if (t_start) {
              float current_temp = strtof(t_start + 5, NULL);

              /* Publish depth & temperature */
              report.timestamp = hrt_absolute_time();
              report.depth_m = current_depth;
              report.temperature_c = current_temp;
              orb_publish(ORB_ID(depth_sensor), depth_pub, &report);

              last_depth = current_depth;
              last_temp = current_temp;
            }

            sync_found = false;
            frame_pos = 0;
          }
        }
      }
    }
  }

  /* Cleanup */
  if (depth_pub != NULL) {
    orb_unadvertise(depth_pub);
  }

  close(serial_fd);
  thread_running = false;
  return 0;
}
