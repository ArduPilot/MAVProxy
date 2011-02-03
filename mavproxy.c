/* 
   MAVlink proxy for FlightGear
   Andrew Tridgell

   partly based on FGShim.c by Michael Smith
 */ 

#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <sys/select.h>
#include <stdarg.h>
#include <termios.h>
#include <ctype.h>
#include <sys/time.h>
#include <time.h>
#include "util.h"

#pragma pack(1)
#include "mavlink_types.h"

mavlink_system_t mavlink_system;
static void comm_send_ch(mavlink_channel_t chan, uint8_t c);
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "ardupilotmega/mavlink.h"

#define TARGET_SYSTEM           7
#define TARGET_COMPONENT        1

/* local port addresses for communication with FlightGear - these are the FG defaults */
#define IMU_LISTEN_PORT         5501
#define CTRL_SEND_PORT          5500

/* local port addresses for communication with QGroundControl */
#define QGCS_LISTEN_PORT        14551
#define QGCS_SEND_PORT          14550

/* frequency of IMU send to FlightGear */
#define FG_FREQUENCY		50

#define DEFAULT_SERIAL_SPEED 115200

/*
 * Binary packet as exchanged with FG.
 *
 * Described in MAVLink.xml
 */
struct fgIMUData {
        // GPS
        double          latitude;
        double          longitude;
        double          altitude;
        double          heading;
        double          velocityN;
        double          velocityE;

        // IMU
        double          accelX;
        double          accelY;
        double          accelZ;
        double          rateRoll;
        double          ratePitch;
        double          rateYaw;

        double          Roll;
        double          Pitch;
        double          Yaw;

        // trailer
#define MSG_MAGIC       0x4c56414d
        uint32_t        magic;
} __attribute__((packed));

struct fgControlData {
        double          aileron;
        double          elevator;
        double          rudder;
        double          throttle;
} __attribute__((packed));


static struct fgIMUData ins;
static struct fgControlData fgcontrol, fg_swapped;
static int fd_serial;
static int fg_in, fg_out;
static int gc_in, gc_out;
static const char *serial_port;
static unsigned serial_speed = DEFAULT_SERIAL_SPEED;

static struct wpoint {
	int action;
	float param1, param2;
	float lat, lon, alt;
} *wpoints;
static unsigned wpoint_count;

static struct status {
	uint32_t ins_counter, serial_counter, mav_counter;
	uint32_t mav_rc, mav_heartbeat, mav_attitude, mav_global_position;
	uint32_t mav_airspeed, mav_param_value;
	char apm_buf[1024];
} status;


/*
  called by mavlink to send a byte
 */
static void comm_send_ch(mavlink_channel_t chan, uint8_t c)
{
        if (write(fd_serial, &c, 1) != 1) {
		printf("Failed to write mavlink char\n");
	}
}


static void write_status(void)
{
	char buf[1024];
	int fd = open("status.txt", O_WRONLY|O_CREAT, 0644);
	dprintf(fd, "Counters:  INS=%u MAV=%u SER=%u\n",
		status.ins_counter, status.mav_counter, status.serial_counter);
	dprintf(fd, "MAV: HBEAT=%u RC=%u ATT=%u GPOS=%u ASP=%u PVAL=%u\n",
		status.mav_heartbeat, status.mav_rc, status.mav_attitude,
		status.mav_global_position, status.mav_airspeed, status.mav_param_value);
	dprintf(fd, "Ins: lat=%7.2f lon=%7.2f alt=%6f hdg=%5.1f\n",
		ins.latitude, ins.longitude, ins.altitude, ins.heading);
	dprintf(fd, "Att: Roll=%5.2f Pitch=%5.2f Yaw=%5.2f dRoll=%5.2f dPitch=%5.2f dYaw=%5.2f\n",
		ins.Roll, ins.Pitch, ins.Yaw, ins.rateRoll, ins.ratePitch, ins.rateYaw);
	dprintf(fd, "APM: %s", status.apm_buf);
	dprintf(fd, "Ctrl: aileron=%6.2f elevator=%6.2f rudder=%6.2f throttle=%6.2f\n",
		fgcontrol.aileron, fgcontrol.elevator, fgcontrol.rudder, fgcontrol.throttle);
	memset(buf, 0, sizeof(buf));
	write(fd, buf, sizeof(buf));
	close(fd);
}


/*
  handle a mavlink message from APM
 */
static void handle_mavlink_msg(mavlink_message_t *msg)
{
        static time_t lastRCMessage;
	time_t now;
        uint8_t buf[1024];
        uint16_t len;

	status.mav_counter++;

        switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
		status.mav_heartbeat++;
		time(&now);
                if ((now - lastRCMessage) > 2) {
			/*
			  its been more than 2 seconds since the last RC message
			 */
#if 0
                        mavlink_msg_request_data_stream_send(0,
                                                             TARGET_SYSTEM,
                                                             TARGET_COMPONENT, 
                                                             MAV_DATA_STREAM_RAW_CONTROLLER,
                                                             10, /* 10Hz enough? */
                                                             1); /* start */
#endif
                        lastRCMessage = now;
                }
                break;
                
        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED: {
		struct fgControlData fg;

		status.mav_rc++;
                /* XXX ArduPilotMega channel ordering */
                fgcontrol.aileron  = (double)mavlink_msg_rc_channels_scaled_get_chan1_scaled(msg) / 10000.0;
                fgcontrol.elevator = (double)mavlink_msg_rc_channels_scaled_get_chan2_scaled(msg) / 10000.0;
                fgcontrol.throttle = (double)mavlink_msg_rc_channels_scaled_get_chan3_scaled(msg) / 10000.0;
                fgcontrol.rudder   = (double)mavlink_msg_rc_channels_scaled_get_chan4_scaled(msg) / 10000.0;
		fg = fgcontrol;
		fg.throttle = constrain(fg.throttle, 0, 0.8);
		fg.aileron  = fg.aileron + (fg.throttle*0.144);
		fg.elevator = -fg.elevator;
		fg.rudder   = fg.rudder;

//		fg.aileron = 0;
//		fg.elevator = 0;
//		fg.throttle = 0.75;
//		fg.rudder = 0;
//		printf("aileron=%f elevator=%f throttle=%f rudder=%f\n",
//		       fg.aileron, fg.elevator, fg.throttle, fg.rudder);
		swap64(&fg, 4);
		fg_swapped = fg;
                lastRCMessage = now;
		break;
	}

        case MAVLINK_MSG_ID_ATTITUDE:
		status.mav_attitude++;
		break;

        case MAVLINK_MSG_ID_AIRSPEED:
		status.mav_airspeed++;
		break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION:
		status.mav_global_position++;
		break;

        case MAVLINK_MSG_ID_PARAM_VALUE:
		status.mav_param_value++;
		break;

        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_GPS_STATUS:
        case MAVLINK_MSG_ID_LOCAL_POSITION:
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        case MAVLINK_MSG_ID_WAYPOINT_CURRENT:
        case MAVLINK_MSG_ID_GPS_RAW:
		break;

        case MAVLINK_MSG_ID_WAYPOINT_REQUEST: {
		unsigned seq = mavlink_msg_waypoint_request_get_seq(msg);
		if (seq >= wpoint_count) {
			printf("Request for bad wpoint %u\n", seq);
			break;
		}
		mavlink_msg_waypoint_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					  seq, MAV_FRAME_GLOBAL, MAV_ACTION_NAVIGATE,
					  0, 0, 0, 0, 0,
					  wpoints[seq].lon, 
					  wpoints[seq].lat, 	
					  wpoints[seq].alt, 
					  0, 1);
		printf("Sent waypoint %u\n", seq);
		break;
	}

        case MAVLINK_MSG_ID_STATUSTEXT: {
		int8_t buf[100];
		int len = mavlink_msg_statustext_get_text(msg, buf);
		int severity = mavlink_msg_statustext_get_severity(msg);
		if (severity > 1) {
			printf("MSG(%d): '%.*s'\n", severity, len, (char *)buf);
		}
		break;
	}

	default:
		printf("mav msg %d\n", (int)msg->msgid);
		break;
        }

        /* pass all messages on to QGCS */
#if 1
        len = mavlink_msg_to_send_buffer(buf, msg);
        write(gc_out, buf, len);
#endif
}


static void send_imu(void)
{
#if 0
        mavlink_msg_raw_imu_send(0,
                                 get_usec(),
                                 fpss2mg(ins.accelX),
                                 fpss2mg(ins.accelY),
                                 fpss2mg(ins.accelZ),
                                 dps2mrps(ins.rateRoll),
                                 dps2mrps(ins.ratePitch),
                                 dps2mrps(ins.rateYaw),
                                 0,     /* xmag */
                                 0,     /* ymag */
                                 0);    /* zmag */
#else
        mavlink_msg_attitude_send(0,
				  get_usec(),
				  deg2rad(ins.Roll),
				  deg2rad(ins.Pitch),
				  deg2rad(ins.Yaw),
				  deg2rad(ins.rateRoll),
				  deg2rad(ins.ratePitch),
				  deg2rad(ins.rateYaw));

#endif
}

static void send_gps(void)
{
        mavlink_msg_gps_raw_send(0,
                                 get_usec(),
                                 3,                     /* 3D fix */
                                 ins.latitude,
                                 ins.longitude,
                                 ins.altitude,
                                 0,                     /* no uncertainty */
                                 0,                     /* no uncertainty */
                                 ft2m(sqrt((ins.velocityN * ins.velocityN) +
                                           (ins.velocityE * ins.velocityE))),
				 ins.heading);
}


static void load_waypoints(const char *filename)
{
	FILE *f = fopen(filename, "r");
	char line[200];
	static struct {
		const char *command_string;
		int action;
	} commands[] = {
		{ "TAKEOFF", MAV_ACTION_TAKEOFF },
		{ "NAVIGATE", MAV_ACTION_NAVIGATE },
		{ NULL, 0 }
	};

	if (f == NULL) {
		printf("Failed to open '%s' : %s\n", filename, strerror(errno));
		return;
	}

	if (wpoints) free(wpoints);
	wpoints = NULL;
	wpoint_count = 0;

	while (fgets(line, sizeof(line), f)) {
		struct wpoint w;
		char cmd[200];
		const char *p = &line[0];
		int i;

		while (isspace(*p)) p++;
		if (!isupper(p[0])) continue;

		if (sscanf(p, "%s %f %f %f %f %f", 
			   cmd,
			   &w.param1, &w.param2,
			   &w.lat, &w.lon, &w.alt) != 6) {
			printf("Bad waypoint line '%s'\n", p);
			fclose(f);
			if (wpoints) free(wpoints);
			wpoint_count = 0;
			return;
		}

		for (i=0; commands[i].command_string; i++) {
			if (strcmp(cmd, commands[i].command_string) == 0) {
				w.action = commands[i].action;
				break;
			}
		}
		if (commands[i].command_string == NULL) {
			printf("Unknown command '%s'\n", cmd);
			fclose(f);
			if (wpoints) free(wpoints);
			wpoint_count = 0;
			return;
		}

		if (wpoint_count == 0) {
			wpoints = malloc(sizeof(struct wpoint));
		} else {
			wpoints = realloc(wpoints, sizeof(struct wpoint)*(wpoint_count+1));
		}
		wpoints[wpoint_count] = w;
		wpoint_count++;
	}
	fclose(f);

	mavlink_msg_waypoint_clear_all_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
	if (wpoint_count == 0) return;

	printf("Loaded %u waypoints\n", wpoint_count);

	mavlink_msg_waypoint_count_send(0, TARGET_SYSTEM, TARGET_COMPONENT, wpoint_count);
}


static void process_stdin(void)
{
	char line[500]="";
	const int max_toks = 20;
	char *tok, *toks[max_toks];
	int num_toks;

	fgets(line, sizeof(line)-2, stdin);

	num_toks = 0;
	for (tok = strtok(line, "\r\n "); tok; tok=strtok(NULL, "\r\n ")) {
		toks[num_toks++] = tok;
		if (num_toks == max_toks) {
			printf("too many tokens\n");
			continue;
		}
	}
	if (num_toks == 0) return;
	
	if (strcmp(toks[0], "loiter") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_LOITER);
	} else if (strcmp(toks[0], "auto") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_SET_AUTO);
	} else if (strcmp(toks[0], "manual") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_SET_MANUAL);
	} else if (strcmp(toks[0], "rtl") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_RETURN);
	} else if (strcmp(toks[0], "takeoff") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_TAKEOFF);
	} else if (strcmp(toks[0], "land") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_LAND);
	} else if (strcmp(toks[0], "next") == 0) {
		mavlink_msg_action_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					MAV_ACTION_CONTINUE);
	} else if (strcmp(toks[0], "command") == 0) {
		if (num_toks != 2) {
			printf("usage: command <commandnum>\n");
			return;
		}
		mavlink_msg_waypoint_set_current_send(0, TARGET_SYSTEM, 
						      TARGET_COMPONENT, atoi(toks[1]));
	} else if (strcmp(toks[0], "param") == 0) {
		if (num_toks < 2) {
			printf("usage: param list\n");
			return;
		}
		if (strcmp(toks[1], "list") == 0) {
			mavlink_msg_param_request_list_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
		}
	} else if (strcmp(toks[0], "load") == 0) {
		if (num_toks != 2) {
			printf("usage: load <filename>\n");
			return;
		}				
		load_waypoints(toks[1]);
	} else {
		printf("unknown command '%s'\n", tok);
	}
}


static void process_serial(void)
{
	static char serial_buf[1024];
	static int serial_len;
	static bool is_mavlink;
	static mavlink_message_t msg;
	char c;
	mavlink_status_t mstatus;
	
	if (read(fd_serial, &c, 1) != 1) {
		close(fd_serial);
		printf("reopening serial port '%s'\n", serial_port);
		while ((fd_serial = open_serial(serial_port, serial_speed)) == -1) {
			sleep(1);
		}
		return;
	}

	/*
	  attempt to cope with debug text on the same
	  port - just hope a message doesn't start
	  with 'U' !
	*/
	if (serial_len == 0 && !is_mavlink) {
		is_mavlink = (c == MAVLINK_STX);
	}
	
	if (is_mavlink) {
		if (mavlink_parse_char(0, c, &msg, &mstatus)) {
			handle_mavlink_msg(&msg);
			is_mavlink = false;
		}
		return;
	}
	
	serial_buf[serial_len++] = c;
	serial_buf[serial_len] = 0;
	
	if (serial_len == sizeof(serial_buf)) {
		printf("serial buffer overflow\n");
		serial_len = 0;
		return;
	}
	if (c != '\n') return;
	status.serial_counter++;
	printf("APM: %s", serial_buf);
	strcpy(status.apm_buf, serial_buf);
	serial_len = 0;
}

static void process_fg(void)
{
	struct fgIMUData buf;
	ssize_t len;

	len = read(fg_in, &buf, sizeof(buf));
	if (len != sizeof(buf)) {
		printf("Bad packet length %d from FG - expected %d\n", 
		       (int)len, (int)sizeof(buf));
		return;
	}
	if (ntohl(buf.magic) != 0x4c56414d) {
		printf("Bad FG magic 0x%08x\n", buf.magic);
		return;
	}
	swap64(&buf, (sizeof(buf)-4)/8);
	ins = buf;
	ins.altitude = ft2m(ins.altitude);
	if (ins.altitude > 0.0) {
		status.ins_counter++;
		send_imu();
		send_gps();
	}
}

static void process_gc(void)
{
	char buf[2048];
	ssize_t len;

	/* pass mavlink packets from gcs to APM */
	len = read(gc_in, buf, sizeof(buf));
	if (len > 0) {
		write(fd_serial, buf, len);
	}
}

static void send_to_fg(void)
{
	struct timeval tv;
	static uint64_t lastt;
	uint64_t t;

	gettimeofday(&tv, NULL);
	t = (((uint64_t)tv.tv_sec) * 1000000) + tv.tv_usec;
	if ((t-lastt) > (1000*1000)/FG_FREQUENCY) {
		write(fg_out, &fg_swapped, sizeof(fg_swapped));
		lastt = t;
	}
}

	
int main(int argc, char* argv[])
{
	if (argc < 2) {
		printf("Usage: mavproxy <serialport> [speed]\n");
		exit(1);
	}
	serial_port = argv[1];
	if (argc > 2) {
		serial_speed = atoi(argv[2]);
	}

	fd_serial     = open_serial(serial_port, serial_speed);
	fg_in         = open_socket_in("127.0.0.1", IMU_LISTEN_PORT);
	fg_out        = open_socket_out("127.0.0.1", CTRL_SEND_PORT);
	gc_in         = open_socket_in("127.0.0.1", QGCS_LISTEN_PORT);
	gc_out        = open_socket_out("127.0.0.1", QGCS_SEND_PORT);

	printf("mavproxy started\n");

	while (1) {
		fd_set fds;
		int ret;
		struct timeval tv;

		FD_ZERO(&fds);
		FD_SET(fd_serial, &fds);
		FD_SET(fg_in, &fds);
		FD_SET(gc_in, &fds);
		FD_SET(0, &fds);

		/* send data to flight gear every 20ms */
		send_to_fg();

		tv.tv_sec = 0;
		tv.tv_usec = 100000;

		write_status();
		fflush(stdout);
		
		ret = select(16, &fds, NULL, NULL, &tv);
		if (ret == 0) {
			continue;
		}
		
		if (FD_ISSET(fg_in, &fds)) {
			process_fg();
			continue;
		}

		if (FD_ISSET(gc_in, &fds)) {
			process_gc();
			continue;
		}

		if (FD_ISSET(0, &fds)) {
			process_stdin();
			continue;
		}

		if (FD_ISSET(fd_serial, &fds)) {
			process_serial();
			continue;
		}
	}
	return 0;
}
