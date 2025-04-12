#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define ESCAPE_CHAR 0x1d
#define EXIT_CHAR '.'
#define CTRL_C_CHAR 3
#define RESET_CHAR 'x'
#define DTR_TOGGLE_CHAR 'd'
#define RTS_TOGGLE_CHAR 'r'
#define BREAK_CHAR 'b'
#define XON_CHAR 0x11
#define XOFF_CHAR 0x13

#define EXIT_OK 0
#define EXIT_TIMEOUT 1
#define EXIT_ERROR 2

#define RESET_SEQUENCE "\033c"

#define DTR_INIT_NONE 0
#define DTR_INIT_HIGH 1
#define DTR_INIT_LOW 2
#define DTR_PULSE_WIDTH 100

#define RTS_INIT_NONE 0
#define RTS_INIT_HIGH 1
#define RTS_INIT_LOW 2
#define RTS_PULSE_WIDTH 100

#define SELECT_TIMEOUT_US 500000

static int timeout;
static int in;
static int out;
static int serial;
static struct termios old_input_options;
static int escape_state;
static int xoff = 0;
static int speed;
static char *terminal_device;
static int opt_ascii = 0;
static int opt_reset = 0;
static int opt_exit_ctrl_c = 0;
static int opt_translate = 0;
static int opt_hard_flow_control = 0;
static int opt_soft_flow_control = 0;
static int opt_dtr = DTR_INIT_HIGH;
static int opt_rts = RTS_INIT_HIGH;
static int opt_pulse_rts = 0;
static int opt_pulse_dts = 0;
static int opt_timeout = 0;
static char *options = "acxtdDrRfsT:p:";

void set_modem_lines(int on, int lines) {
  unsigned int status;

  ioctl(serial, TIOCMGET, &status);
  if (on)
    status |= lines;
  else
    status &= ~lines;
  ioctl(serial, TIOCMSET, &status);
}

int clear_to_send(void) {
  unsigned int status;

  ioctl(serial, TIOCMGET, &status);
  return !(status & TIOCM_CTS);
}

void toggle(int line, int active_low, int delay_ms) {
  set_modem_lines(!active_low, line);
  usleep(delay_ms * 1000);
  set_modem_lines(active_low, line);
  usleep(delay_ms * 1000);
  set_modem_lines(!active_low, line);
}

void send_break(void) { tcsendbreak(serial, 0); }

void send_data(char *data, unsigned int count) {
  if (opt_hard_flow_control) {
    while (!clear_to_send())
      ;
  }
  write(serial, data, count);
}

bool transfer_to_serial(void) {
  char c;
  int byte_count;

  byte_count = read(in, &c, sizeof(c));
  if (byte_count <= 0) {
    return false;
  }

  if (!escape_state) {
    if (opt_exit_ctrl_c && c == CTRL_C_CHAR) {
      return false;
    }
    if (c == ESCAPE_CHAR) {
      escape_state = 1;
      return true;
    }

    if (!opt_translate || c != '\n') {
      send_data(&c, byte_count);
    } else {
      send_data("\r\n", 2);
    }
    return true;
  }

  switch (c) {
  case EXIT_CHAR:
    return false;
  case RESET_CHAR:
    write(out, RESET_SEQUENCE, sizeof(RESET_SEQUENCE));
    break;
  case DTR_TOGGLE_CHAR:
    toggle(TIOCM_DTR, opt_dtr == DTR_INIT_HIGH, DTR_PULSE_WIDTH);
    break;
  case RTS_TOGGLE_CHAR:
    toggle(TIOCM_RTS, opt_rts == RTS_INIT_HIGH, RTS_PULSE_WIDTH);
    break;
  case BREAK_CHAR:
    send_break();
    break;
  default:
    send_data(&c, byte_count);
  }
  escape_state = 0;
  return true;
}

bool transfer_from_serial(void) {
  unsigned char buffer[512];
  int byte_count;

  byte_count = read(serial, buffer, sizeof(buffer));
  if (byte_count <= 0) {
    return false;
  }

  for (size_t i = 0; i < byte_count; i++) {
    if (opt_soft_flow_control) {
      if (buffer[i] == XOFF_CHAR) {
        xoff = 1;
        continue;
      }
      if (buffer[i] == XON_CHAR) {
        xoff = 0;
        continue;
      }
      if (opt_ascii &&
          (buffer[i] > 128 || (buffer[i] < ' ' && buffer[i] != '\t' &&
                               buffer[i] != '\r' && buffer[i] != '\n')))
        buffer[i] = '.';
    }
    write(out, &buffer[i], 1);
  }
  return true;
}

void configure_input(void) {
  struct termios options;

  tcgetattr(in, &options);

  old_input_options = options;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10;
  options.c_cflag |= CLOCAL | CREAD | HUPCL;
  options.c_cflag &= ~(IXON | IXOFF);
  options.c_lflag = 0;

  tcflush(in, TCIFLUSH);
  tcsetattr(in, TCSANOW, &options);
}

void configure_serial(int baud) {
  struct termios options;
  unsigned int status;

  tcgetattr(serial, &options);

  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~INLCR & ~ICRNL & ~INPCK;
  options.c_cflag |= IXON | IXOFF;
  options.c_iflag |= IGNPAR;
  options.c_cflag &= ~PARENB & ~CSTOPB & ~CSIZE & ~CRTSCTS & ~IXANY;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10;
  options.c_cflag |= CLOCAL | CREAD | HUPCL | CS8;
  options.c_lflag = 0;

  cfsetspeed(&options, baud);
  tcflush(serial, TCIFLUSH);
  tcsetattr(serial, TCSANOW, &options);

  ioctl(serial, TIOCMGET, &status);
  status &= ~(TIOCM_DTR | TIOCM_RTS);
  if (opt_rts == RTS_INIT_LOW) {
    usleep(250 * 1000);
    status |= TIOCM_RTS;
    ioctl(serial, TIOCMSET, &status);
  }
  if (opt_dtr == DTR_INIT_LOW) {
    usleep(250 * 1000);
    status |= TIOCM_DTR;
    ioctl(serial, TIOCMSET, &status);
  }

  if (opt_pulse_rts) {
    usleep(500 * 1000);
    toggle(TIOCM_RTS, opt_rts == RTS_INIT_HIGH, RTS_PULSE_WIDTH);
  }

  if (opt_pulse_dts) {
    usleep(500 * 1000);
    toggle(TIOCM_DTR, opt_rts == DTR_INIT_HIGH, RTS_PULSE_WIDTH);
  }
}

void unconfigure_input(void) { tcsetattr(in, TCSANOW, &old_input_options); }

void signal_handler(int signum) {
  switch (signum) {
  case SIGQUIT:
  case SIGINT:
  case SIGTERM:
    unconfigure_input();
    break;
  default:
    break;
  }
}

void setup_signal_handlers(void) {
  signal(SIGQUIT, signal_handler);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
}

void handle_cmd_line(int argc, char **argv) {
  int c;
  struct stat st;

  while ((c = getopt(argc, argv, options)) != -1) {
    switch (c) {
    case 'a':
      opt_ascii = 1;
      break;
    case 'x':
      opt_reset = 1;
      break;
    case 'c':
      opt_exit_ctrl_c = 1;
      break;
    case 'f':
      opt_hard_flow_control = 1;
      break;
    case 's':
      opt_soft_flow_control = 1;
      break;
    case 't':
      opt_translate = 1;
      break;
    case 'd':
      opt_dtr = DTR_INIT_HIGH;
      break;
    case 'D':
      opt_dtr = DTR_INIT_LOW;
      break;
    case 'r':
      opt_rts = RTS_INIT_HIGH;
      break;
    case 'R':
      opt_rts = RTS_INIT_LOW;
      break;
    case 'T':
      opt_timeout = atoi(optarg);
      break;
    case 'p':
      switch (optarg[0]) {
      case 'd':
        opt_pulse_dts = 1;
        break;
      case 'r':
        opt_pulse_rts = 1;
        break;
      default:
        break;
      }
      break;
    case '?':
      exit(1);
      break;
    default:
      break;
    }
  }
  if (optind < argc) {
    terminal_device = argv[optind];
    if (stat(terminal_device, &st) != 0) {
      perror(terminal_device);
      exit(2);
    }
    optind++;
  } else {
    printf("Please specify a device.\n");
    exit(1);
  }
  if (optind < argc)
    speed = atoi(argv[optind]);
  else
    speed = 115200;
}

int main(int argc, char **argv) {
  struct timeval select_timeout;
  fd_set readfds;
  int r = EXIT_OK;

  handle_cmd_line(argc, argv);

  in = dup(0);
  out = dup(1);
  serial = open(terminal_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial == -1) {
    perror("Failed to open device");
    exit(3);
  }

  configure_serial(speed);
  configure_input();

  setup_signal_handlers();
  escape_state = 0;
  timeout = time(NULL) + opt_timeout;
  for (;;) {
    FD_ZERO(&readfds);
    FD_SET(serial, &readfds);
    FD_SET(in, &readfds);

    select_timeout.tv_sec = 0;
    select_timeout.tv_usec = SELECT_TIMEOUT_US;

    if (select(FD_SETSIZE, &readfds, NULL, NULL, &select_timeout) == -1) {
      r = EXIT_ERROR;
      break;
    }

    if (FD_ISSET(in, &readfds) && !(opt_soft_flow_control && xoff) &&
        transfer_to_serial() == false) {
      break;
    }

    if (FD_ISSET(serial, &readfds)) {
      if (transfer_from_serial() == false) {
        break;
      }
      timeout = time(NULL) + opt_timeout;
    }

    if (opt_timeout && time(NULL) > timeout) {
      r = EXIT_TIMEOUT;
      break;
    }
  }

  unconfigure_input();
  if (opt_reset) {
    write(out, RESET_SEQUENCE, sizeof(RESET_SEQUENCE));
  }

  close(serial);
  close(in);
  close(out);

  return r;
}
