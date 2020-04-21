#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <getopt.h>

#define ESCAPE_CHAR 0x1d
#define EXIT_CHAR '.'
#define CTRL_C_CHAR 3
#define RESET_CHAR 'x'
#define DTR_TOGGLE_CHAR 'd'
#define RTS_TOGGLE_CHAR 'r'
#define BREAK_CHAR 'b'

#define RESET_SEQUENCE "\033c"

#define DTR_INIT_NONE 0
#define DTR_INIT_HIGH 1
#define DTR_INIT_LOW 2
#define DTR_PULSE_WIDTH 100

#define RTS_INIT_NONE 0
#define RTS_INIT_HIGH 1
#define RTS_INIT_LOW 2
#define RTS_PULSE_WIDTH 100

static int in;
static int out;
static int terminal;
static struct termios old_input_options;
static int escape_state;
static int speed;
static char *terminal_device;
static int opt_ascii = 0;
static int opt_reset = 0;
static int opt_exit_ctrl_c = 0;
static int opt_translate = 0;
static int opt_flow_control = 0;
static int opt_dtr = DTR_INIT_NONE;
static int opt_rts = RTS_INIT_NONE;
static char *options = "acxtdDrRf";

void set_modem_lines(int on, int lines)
{
	unsigned int status;

	ioctl(terminal, TIOCMGET, &status);
	if (on)
		status |= lines;
	else
		status &= ~lines;
	ioctl(terminal, TIOCMSET, &status);
}

int clear_to_send(void)
{
	unsigned int status;

	ioctl(terminal, TIOCMGET, &status);
	return !(status & TIOCM_CTS);
}

void toggle(int line, int active_low, int delay_ms)
{
	set_modem_lines(!active_low, line);
	usleep(delay_ms * 1000);
	set_modem_lines(active_low, line);
	usleep(delay_ms * 1000);
	set_modem_lines(!active_low, line);
}

void send_break(void)
{
	tcsendbreak(terminal, 0);
}

void send_data(char *data, unsigned int count)
{
	while (opt_flow_control && !clear_to_send());
	write(terminal, data, count);
}

bool transfer_to_terminal(void)
{
	char c;
	bool r = true;
	int byte_count;

	byte_count = read(in, &c, sizeof(c));
	if (byte_count > 0) {
		if (!escape_state) {
			if (opt_exit_ctrl_c && c == CTRL_C_CHAR) {
				r = false;
			} else if (c != ESCAPE_CHAR) {
				if (!opt_translate || c != '\n') {
					send_data(&c, byte_count);
				} else {
					send_data("\r\n", 2);
				}
			} else {
				escape_state = 1;
			}
		} else {
			switch (c) {
			case EXIT_CHAR:
				r = false;
				break;
			case RESET_CHAR:
				write(out, RESET_SEQUENCE,
				      sizeof(RESET_SEQUENCE));
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
		}
	} else {
		r = false;
	}
	return r;
}

bool transfer_from_terminal(void)
{
	unsigned char buffer[512];
	int i;
	int byte_count;
	bool r = true;

	byte_count = read(terminal, buffer, sizeof(buffer));
	if (byte_count > 0) {
		if (opt_ascii) {
			for (i = 0; i < byte_count; i++) {
				if (buffer[i] > 128 ||
				    (buffer[i] < ' ' &&
				     buffer[i] != '\t' && buffer[i] != '\r' && buffer[i] != '\n'))
					buffer[i] = '.';
			}
		}
		write(out, buffer, byte_count);
	} else {
		r = false;
	}
	return r;
}

void configure_input(void)
{
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

void configure_terminal(int baud)
{
	struct termios options;

	tcgetattr(terminal, &options);

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
	tcflush(terminal, TCIFLUSH);
	tcsetattr(terminal, TCSANOW, &options);

	set_modem_lines(1,
			(opt_dtr == DTR_INIT_LOW ? TIOCM_DTR : 0) |
			(opt_rts == RTS_INIT_LOW ? TIOCM_RTS : 0));
	set_modem_lines(0,
			(opt_dtr == DTR_INIT_HIGH ? TIOCM_DTR : 0) |
			(opt_rts == RTS_INIT_HIGH ? TIOCM_RTS : 0));
}

void unconfigure_input(void)
{
	tcsetattr(in, TCSANOW, &old_input_options);
}

void signal_handler(int signum)
{
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

void setup_signal_handlers(void)
{
	signal(SIGQUIT, signal_handler);
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
}

void handle_cmd_line(int argc, char **argv)
{
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
			opt_flow_control = 1;
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
		printf("Please specify a terminal.\n");
		exit(1);
	}
	if (optind < argc)
		speed = atoi(argv[optind]);
	else
		speed = 115200;

}

int main(int argc, char **argv)
{
	struct timeval timeout;
	fd_set readfds;

	handle_cmd_line(argc, argv);

	in = dup(0);
	out = dup(1);
	terminal = open(terminal_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (terminal == -1) {
		perror("Failed to open terminal");
		exit(3);
	}

	configure_terminal(speed);
	configure_input();

	setup_signal_handlers();
	escape_state = 0;
	for (;;) {
		FD_ZERO(&readfds);
		FD_SET(terminal, &readfds);
		FD_SET(in, &readfds);

		timeout.tv_sec  = 0;
		timeout.tv_usec = 500000;

		if (select(FD_SETSIZE, &readfds, NULL, NULL, &timeout) == -1) {
			break;
		} else {
			if (FD_ISSET(terminal, &readfds) &&
			    transfer_from_terminal() == false)
				break;

			if (FD_ISSET(in, &readfds) &&
			    transfer_to_terminal() == false)
				break;
		}
	}

	unconfigure_input();
	if (opt_reset)
		write(out, RESET_SEQUENCE, sizeof(RESET_SEQUENCE));

	close(terminal);
	close(in);
	close(out);

	return 0;
}

