/* 
   MAVlink proxy for FlightGear
   Andrew Tridgell

   Released under the GNU GPL version 3 or later

   inspired by FGShim.c by Michael Smith
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
#include <fnmatch.h>
#include <readline/readline.h>
#include <readline/history.h>
#include "util.h"
#include "editfile.h"

#pragma pack(1)
#include "mavlink_types.h"

mavlink_system_t mavlink_system;
static void comm_send_ch(mavlink_channel_t chan, uint8_t c);
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "ardupilotmega/mavlink.h"

#define TARGET_SYSTEM           7
#define TARGET_COMPONENT        1

/* local port addresses for communication with FlightGear */
#define IMU_LISTEN_PORT         5501
#define CTRL_SEND_PORT          5500

/* local port address for communication with QGroundControl */
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

        double          airspeed_kt;

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


static bool setup_mode;

static struct fgIMUData ins;
static struct fgControlData fgcontrol;
static int fd_serial;
static int fg_in, fg_out;
static int gc_sock;
static const char *serial_port;
static unsigned serial_speed = DEFAULT_SERIAL_SPEED;

static unsigned num_mav_param;
static mavlink_param_value_t *mav_param;
static enum param_op { PARAM_NONE, PARAM_FETCH, PARAM_EDIT, PARAM_SAVE } param_op;
static char *param_save_filename;
static char *param_wildcard;

static unsigned wp_count;
static mavlink_waypoint_t *wp_list;
static enum wp_op { WP_NONE, WP_LIST, WP_SAVE } wp_op;
static char *wp_save_filename;

#define NUM_MSG_LINES 8

static struct status {
	uint32_t ins_counter, serial_counter, mav_counter, gc_counter;
	uint32_t mav_rc, mav_heartbeat, mav_attitude, mav_global_position;
	uint32_t mav_airspeed, mav_param_value;
	char apm_buf[1024];
	char msg_buf[NUM_MSG_LINES][200];
} status;

static bool loading_waypoints;
static time_t loading_waypoint_lasttime;



/*
  called by mavlink to send a byte
 */
static void comm_send_ch(mavlink_channel_t chan, uint8_t c)
{
        if (write(fd_serial, &c, 1) != 1) {
		printf("Failed to write mavlink char\n");
	}
}


/*
  show current status in status.txt. This is a very crude ground
  control station! Use "watch -n1 cat status.txt" to view the status
  while running
 */
static void write_status(void)
{
	char buf[1024];
	int fd;
	int i;
	static time_t last_time;
	time_t t;

	t = time(NULL);
	if (t == last_time) return;
	last_time = t;
	
	fd = open("status.txt", O_WRONLY|O_CREAT, 0644);

	dprintf(fd, "Counters:  INS=%u MAV=%u SER=%u GC=%u\n",
		status.ins_counter, status.mav_counter, status.serial_counter, status.gc_counter);
	dprintf(fd, "MAV: HBEAT=%u RC=%u ATT=%u GPOS=%u ASP=%u PVAL=%u\n",
		status.mav_heartbeat, status.mav_rc, status.mav_attitude,
		status.mav_global_position, status.mav_airspeed, status.mav_param_value);
	dprintf(fd, "Ins: lat=%7.2f lon=%7.2f alt=%6f hdg=%5.1f\n",
		ins.latitude, ins.longitude, ins.altitude, ins.heading);
	dprintf(fd, "Att: Roll=%5.2f Pitch=%5.2f Yaw=%5.2f dRoll=%5.2f dPitch=%5.2f dYaw=%5.2f\n",
		ins.Roll, ins.Pitch, ins.Yaw, ins.rateRoll, ins.ratePitch, ins.rateYaw);
	dprintf(fd, "Ctrl: aileron=%6.2f elevator=%6.2f rudder=%6.2f throttle=%6.2f\n",
		fgcontrol.aileron, fgcontrol.elevator, fgcontrol.rudder, fgcontrol.throttle);
	for (i=0; i<NUM_MSG_LINES; i++) {
		if (status.msg_buf[i][0] != 0) {
			dprintf(fd, "%s", status.msg_buf[i]);			
		}
	}
	dprintf(fd, "APM: %s", status.apm_buf);
	memset(buf, 0, sizeof(buf));
	write(fd, buf, sizeof(buf));
	close(fd);
}

static int param_find(const char *id)
{
	int i;
	for (i=0; i<num_mav_param; i++) {
		if (strcmp(id, (char *)mav_param[i].param_id) == 0) {
			return i;
		}
	}
	return -1;
}


/*
  called when the user has finished editing a parameter list
 */
static void param_load_file(const char *fname)
{
	FILE *f;
	char id[16];
	float value;
	char line[100];

	f = fopen(fname, "r");
	if (f == NULL) {
		printf("Unable to open edit file '%s'\n", fname);
		return;
	}

	while (fgets(line, sizeof(line), f)) {
		int i;
		char *p = line;

		while (isspace(*p)) p++;
		if (*p == '#') continue;

		if (sscanf(line, "%15s %f", id, &value) != 2) {
			continue;
		}

		i = param_find(id);
		if (i == -1) {
			printf("Unable to find parameter '%s'\n", id);
			break;
		}

		/* see if its changed */
		if (value == mav_param[i].param_value) continue;

		mavlink_msg_param_set_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
					   mav_param[i].param_id, value);
		printf("Setting %s to %f\n", id, value);
	}
	fclose(f);
}

/*
  called when the user has finished editing a parameter list
 */
static void edit_param_callback(const char *fname, int status)
{
	printf("Finished edit status=%d\n", status);
	if (status != 0) {
		unlink(fname);
		return;
	}

	param_load_file(fname);
	
	unlink(fname);
}

/*
  handle an incoming parameter value from APM via MAVLink
 */
static void process_param_value(mavlink_message_t *msg)
{
	unsigned count = mavlink_msg_param_value_get_param_count(msg);
	unsigned idx   = mavlink_msg_param_value_get_param_index(msg);
	int i;

	if (count != num_mav_param) {
		if (mav_param) free(mav_param);
		mav_param = calloc(sizeof(mav_param[0]), count);
		if (mav_param == NULL) {
			printf("Out of memory allocating mav_param\n");
			num_mav_param = 0;
			return;
		}
		num_mav_param = count;
	}

	mavlink_msg_param_value_decode(msg, &mav_param[idx]);
	if (idx < count - 1) {
		return;
	}

	printf("Received %u parameter values\n", count);

	switch (param_op) {
	case PARAM_NONE:
		break;

	case PARAM_FETCH:
		param_op = PARAM_NONE;
		break;

	case PARAM_EDIT: {
		char fname[] = "/tmp/mavlink_param.XXXXXX";
		int fd = mkstemp(fname);
		if (fd == -1) {
			printf("Unable to create temporary file for editing : %s\n",
			       strerror(errno));
			break;
		}
		for (i=0; i<num_mav_param; i++) {
			if (param_wildcard && 
			    fnmatch(param_wildcard, (char *)mav_param[i].param_id, 0) != 0) {
				continue;
			}
			dprintf(fd, "%-15.15s %f\n", mav_param[i].param_id, mav_param[i].param_value);
		}
		close(fd);
		edit_file_background(fname, edit_param_callback);
		param_op = PARAM_NONE;
		break;
	}

	case PARAM_SAVE: {
		int fd = open(param_save_filename, O_WRONLY|O_CREAT|O_TRUNC, 0666);
		unsigned count = 0;
		if (fd == -1) {
			printf("Unable to create parameter save file : %s\n",
			       strerror(errno));
			break;
		}
		for (i=0; i<num_mav_param; i++) {
			if (param_wildcard && 
			    fnmatch(param_wildcard, (char *)mav_param[i].param_id, 0) != 0) {
				continue;
			}
			count++;
			dprintf(fd, "%-15.15s %f\n", mav_param[i].param_id, mav_param[i].param_value);
		}
		close(fd);
		printf("Saved %u parameters to %s\n", count, param_save_filename);
		param_op = PARAM_NONE;
		break;
	}
	}
}


/*
  handle an incoming waypoint request via MAVLink
 */
static void process_waypoint_request(mavlink_message_t *msg)
{
	mavlink_waypoint_t *wp;
	if (!loading_waypoints || 
	    time(NULL) > loading_waypoint_lasttime + 10) {
		return;
	}
	unsigned seq = mavlink_msg_waypoint_request_get_seq(msg);
	if (seq >= wp_count) {
		printf("Request for bad wpoint %u (max %u)\n", seq, wp_count);
		return;
	}
	wp = &wp_list[seq];

	mavlink_msg_waypoint_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 
				  wp->seq, wp->frame, wp->action,
				  wp->orbit, wp->orbit_direction, wp->param1, 
				  wp->param2, wp->current, wp->x, wp->y, wp->z, wp->yaw, wp->autocontinue);
	loading_waypoint_lasttime = time(NULL);
	if (seq == wp_count -1 ) {
		loading_waypoints = false;
	}
	printf("Sent waypoint %u\n", seq);
}

/*
  handle an incoming waypoint count via MAVLink
 */
static void process_waypoint_count(mavlink_message_t *msg)
{
	if (wp_op == WP_NONE) return;

	if (wp_list) free(wp_list);
	wp_count = mavlink_msg_waypoint_count_get_count(msg);
	wp_list = calloc(sizeof(mavlink_waypoint_t), wp_count);
	if (wp_count > 0) {
		printf("Requesting %u waypoints\n", wp_count);
		mavlink_msg_waypoint_request_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 0);
	}
}


/*
  handle an incoming waypoint via MAVLink
 */
static void process_waypoint(mavlink_message_t *msg)
{
	uint16_t i;
	if (wp_op == WP_NONE) return;

	i = mavlink_msg_waypoint_get_seq(msg);
	if (i >= wp_count) {
		printf("Invalid waypoint index %u\n", i);
		return;
	}
	mavlink_msg_waypoint_decode(msg, &wp_list[i]);
	if (i < wp_count-1) {
		mavlink_msg_waypoint_request_send(0, TARGET_SYSTEM, TARGET_COMPONENT, i+1);
		return;
	}

	switch (wp_op) {
	case WP_NONE:
		break;
	case WP_LIST:
		for (i=0; i<wp_count; i++) {
			printf("%.10f %.10f %f\n", wp_list[i].x, wp_list[i].y, wp_list[i].z);
		}
		break;
	case WP_SAVE: {
		FILE *f = fopen(wp_save_filename, "w");
		if (f == NULL) {
			printf("Unable to open %s : %s\n", wp_save_filename, strerror(errno));
			break;
		}
		/* use qgroundcontrol format - see src/Waypoint.cc */
		fprintf(f, "QGC WPL 100\n");
		for (i=0; i<wp_count; i++) {
			mavlink_waypoint_t *wp = &wp_list[i];
			fprintf(f, "%u\t%u\t%u\t%.0f\t%u\t%.0f\t%.0f\t%u\t%f\t%f\t%.2f\t%.0f\t%u\n", 
				wp->seq, wp->frame, wp->action, wp->orbit, wp->orbit_direction, wp->param1, 
				wp->param2, wp->current, wp->x, wp->y, wp->z, wp->yaw, wp->autocontinue);
		}
		fclose(f);
		printf("Saved %u waypoints to %s\n", wp_count, wp_save_filename);
		free(wp_save_filename);
		wp_save_filename = NULL;
		break;
	}
	}

	wp_op = WP_NONE;
}

/*
  scale servo inputs into fg controls
 */
static double scale_rc(uint16_t servo, double minv, double maxv)
{
	/* assume a servo range of 1000 to 2000 */
	double v = (servo - 1000.0) / 1000.0;
	return minv + v*(maxv-minv);
}

/*
  handle a mavlink message from APM
 */
static void handle_mavlink_msg(mavlink_message_t *msg)
{
        uint8_t buf[1024];
        uint16_t len;

	status.mav_counter++;

        switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
		status.mav_heartbeat++;
                break;
                
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
		status.mav_rc++;
                fgcontrol.aileron  = scale_rc(mavlink_msg_rc_channels_raw_get_chan1_raw(msg), -1, 1);
                fgcontrol.elevator = - scale_rc(mavlink_msg_rc_channels_raw_get_chan2_raw(msg), -1, 1);
                fgcontrol.throttle = scale_rc(mavlink_msg_rc_channels_raw_get_chan3_raw(msg), 0, 1);
                fgcontrol.rudder   = scale_rc(mavlink_msg_rc_channels_raw_get_chan4_raw(msg), -1, 1);
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
		process_param_value(msg);
		break;

        case MAVLINK_MSG_ID_SYS_STATUS:
        case MAVLINK_MSG_ID_GPS_STATUS:
        case MAVLINK_MSG_ID_LOCAL_POSITION:
        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
        case MAVLINK_MSG_ID_WAYPOINT_CURRENT:
        case MAVLINK_MSG_ID_GPS_RAW:
        case MAVLINK_MSG_ID_WAYPOINT_ACK:
        case MAVLINK_MSG_ID_RAW_IMU:
        case MAVLINK_MSG_ID_RAW_PRESSURE:
		break;

        case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
		process_waypoint_request(msg);
		break;

        case MAVLINK_MSG_ID_WAYPOINT:
		process_waypoint(msg);
		break;

        case MAVLINK_MSG_ID_WAYPOINT_COUNT:
		process_waypoint_count(msg);
		break;

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
        len = mavlink_msg_to_send_buffer(buf, msg);
        write(gc_sock, buf, len);
}

/*
  send instrument data to APM
 */
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
	mavlink_msg_airspeed_send(0,
				  kt2mps(ins.airspeed_kt));
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

/*
  load waypoints from a file
 */
static void load_waypoints(const char *filename)
{
	FILE *f = fopen(filename, "r");
	char line[200];
	if (f == NULL) {
		printf("Failed to open '%s' : %s\n", filename, strerror(errno));
		return;
	}

	if (wp_list) free(wp_list);
	wp_list = NULL;
	wp_count = 0;


	while (fgets(line, sizeof(line), f)) {
		mavlink_waypoint_t *wp;
		const char *p = &line[0];
		unsigned seq, frame, action, orbit_direction, current, autocontinue;
		float orbit, param1, param2, x, y, z, yaw;

		while (isspace(*p)) p++;
		if (!isdigit(p[0])) continue;

		if (sscanf(p, "%u\t%u\t%u\t%f\t%u\t%f\t%f\t%u\t%f\t%f\t%f\t%f\t%u\n", 
			   &seq, &frame, &action, &orbit, &orbit_direction, &param1, 
			   &param2, &current, &x, &y, &z, &yaw, &autocontinue) != 13) {
			printf("Bad waypoint line '%s'\n", p);
			fclose(f);
			return;
		}

		if (wp_count == 0) {
			wp_list = malloc(sizeof(mavlink_waypoint_t));
		} else {
			wp_list = realloc(wp_list, sizeof(mavlink_waypoint_t)*(wp_count+1));
		}
		wp = &wp_list[wp_count];

		wp->seq = wp_count;
		wp->frame = frame;
		wp->action = action;
		wp->orbit = orbit;
		wp->orbit_direction = orbit_direction;
		wp->param1 = param1;
		wp->param2 = param2;
		wp->current = current;
		wp->x = x;
		wp->y = y;
		wp->z = z;
		wp->yaw = yaw;
		wp->autocontinue = autocontinue;
		wp_count++;
	}
	fclose(f);

	mavlink_msg_waypoint_clear_all_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
	if (wp_count == 0) return;

	printf("Loaded %u waypoints\n", wp_count);

	loading_waypoints = true;
	loading_waypoint_lasttime = time(NULL);

	mavlink_msg_waypoint_count_send(0, TARGET_SYSTEM, TARGET_COMPONENT, wp_count);
}

/*
  handle parameter commands
 */
static void cmd_param(int num_args, char **args)
{
	if (num_args < 1) {
		printf("usage: param <fetch|edit|set|show>\n");
		return;
	}
	if (strcmp(args[0], "fetch") == 0) {
		mavlink_msg_param_request_list_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
		printf("Requested parameter list\n");
		param_op = PARAM_FETCH;
	} else if (strcmp(args[0], "edit") == 0) {
		if (param_wildcard) free(param_wildcard);
		if (num_args > 1) {
			param_wildcard = strdup(args[1]);
		} else {
			param_wildcard = NULL;
		}
		mavlink_msg_param_request_list_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
		printf("Requested parameter list for editing\n");
		param_op = PARAM_EDIT;
	} else if (strcmp(args[0], "save") == 0) {
		if (num_args < 2) {
			printf("usage: param save <filename> [wildcard]\n");
			return;
		}
		if (param_save_filename) free(param_save_filename);
		param_save_filename = strdup(args[1]);
		if (param_wildcard) free(param_wildcard);
		if (num_args > 2) {
			param_wildcard = strdup(args[2]);
		} else {
			param_wildcard = NULL;
		}
		mavlink_msg_param_request_list_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
		printf("Requested parameter list for saving\n");
		param_op = PARAM_SAVE;
	} else if (strcmp(args[0], "set") == 0) {
		if (num_args != 3) {
			printf("Usage: param set PARMNAME VALUE\n");
			return;
		}
		if (param_find(args[1]) == -1) {
			printf("Unable to find parameter '%s'\n", args[1]);
			return;
		}
		mavlink_msg_param_set_send(0, TARGET_SYSTEM, TARGET_COMPONENT, (const int8_t *)args[1], atof(args[2]));
		param_op = PARAM_NONE;
	} else if (strcmp(args[0], "load") == 0) {
		if (num_args != 2) {
			printf("Usage: param load <filename>\n");
			return;
		}
		param_load_file(args[1]);
		param_op = PARAM_NONE;
	} else if (strcmp(args[0], "show") == 0) {
		const char *pattern = "*";
		int i;
		if (num_args > 1) {
			pattern = args[1];
		}
		for (i=0; i<num_mav_param; i++) {
			if (fnmatch(pattern, (char *)mav_param[i].param_id, 0) == 0) {
				printf("%-15.15s %f\n", mav_param[i].param_id, mav_param[i].param_value);
			}
		}
		param_op = PARAM_NONE;
	} else {
		printf("Unknown subcommand '%s' (try 'edit', 'save', 'set', 'show' or 'load')\n", args[0]);
	}
}

/*
  handle waypoint commands
 */
static void cmd_wp(int num_args, char **args)
{
	if (num_args < 1) {
		printf("usage: wp <list|load|save|set>\n");
		return;
	}
	if (strcmp(args[0], "load") == 0) {
		if (num_args != 2) {
			printf("usage: wp load <filename>\n");
			return;
		}
		load_waypoints(args[1]);
		return;
	}
	if (strcmp(args[0], "list") == 0) {
		wp_op = WP_LIST;
		mavlink_msg_waypoint_request_list_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
		return;
	}
	if (strcmp(args[0], "save") == 0) {
		if (num_args != 2) {
			printf("usage: wp save <filename>\n");
			return;
		}
		if (wp_save_filename) free(wp_save_filename);
		wp_save_filename = strdup(args[1]);
		wp_op = WP_SAVE;
		mavlink_msg_waypoint_request_list_send(0, TARGET_SYSTEM, TARGET_COMPONENT);
		return;
	}

	if (strcmp(args[0], "set") == 0) {
		if (num_args != 2) {
			printf("usage: wp set <wpindex>\n");
			return;
		}
		mavlink_msg_waypoint_set_current_send(0, TARGET_SYSTEM, 
						      TARGET_COMPONENT, atoi(args[1]));
		return;
	}
	
	printf("Usage: wp <list|load|save|set>\n");
}


/*
  force a PWM value
 */
static void control_set(int num_args, char **args, const char *name, const char *parm_name)
{
	uint16_t value;
	if (num_args != 1) {
		printf("Usage: %s <value>\n", name);
		return;
	}
	value = atoi(args[0]);
	mavlink_msg_param_set_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 	
				   (int8_t *)parm_name, value);
	if (value == 0) {
		printf("Disabled %s override\n", name);
	} else {
		printf("Set %s override to %u\n", name, value);
	}
}

/*
  force throttle
 */
static void cmd_throttle(int num_args, char **args)
{
	control_set(num_args, args, "throttle", "PWM_THR_FIX");
}

/*
  force roll
 */
static void cmd_roll(int num_args, char **args)
{
	control_set(num_args, args, "roll", "PWM_ROLL_FIX");
}

/*
  force pitch
 */
static void cmd_pitch(int num_args, char **args)
{
	control_set(num_args, args, "pitch", "PWM_PITCH_FIX");
}

/*
  force rudder
 */
static void cmd_rudder(int num_args, char **args)
{
	control_set(num_args, args, "rudder", "PWM_YAW_FIX");
}


/*
  set a override RC switch value
 */
static void cmd_switch(int num_args, char **args)
{
	int value;
	uint16_t mapping[] = { 0, 1165, 1295, 1425, 1555, 1685, 1815 };
	int flite_mode_ch_parm;
	char *parm_name;

	if (num_args != 1) {
		printf("Usage: switch <value>\n");
		return;
	}
	value = atoi(args[0]);
	if (value < 0 || value > 6) {
		printf("Invalid switch value. Use 1-6 for flight modes, '0' to disable\n");
		return;
	}
	flite_mode_ch_parm = param_find("FLITE_MODE_CH");
	if (flite_mode_ch_parm == -1) {
		printf("Unable to find FLITE_MODE_CH parameter\n");
		return;
	}
	asprintf(&parm_name, "PWM_CH%u_FIX", (unsigned)mav_param[flite_mode_ch_parm].param_value);
	mavlink_msg_param_set_send(0, TARGET_SYSTEM, TARGET_COMPONENT, 	
				   (int8_t *)parm_name, mapping[value]);
	if (value == 0) {
		printf("Disabled RC switch override\n");
	} else {
		printf("Set RC switch override to %u (PWM=%u %s)\n", value, mapping[value], parm_name);
	}
	free(parm_name);
}


/*
  go into setup mode
 */
static void cmd_setup(int num_args, char **args)
{
	setup_mode = true;
	printf("Entering setup mode - use '.' on a line by itself to exit\n");
	rl_set_prompt("setup> ");
}

/*
  reopen the APM serial link
 */
static void cmd_reset(int num_args, char **args)
{
	close(fd_serial);
	while ((fd_serial = open_serial(serial_port, serial_speed)) == -1) {
		sleep(1);
	}
}

static struct {
	char *command;
	int mav_action;
	void (*f)(int num_args, char **args);
	const char *description;
} commands[] = {
	{ "loiter", MAV_ACTION_LOITER,     NULL, "loiter in current position" },
	{ "auto",   MAV_ACTION_SET_AUTO,   NULL, "set APM to auto mode" },
	{ "manual", MAV_ACTION_SET_MANUAL, NULL, "set APM to manual mode" },
	{ "rtl",    MAV_ACTION_RETURN,     NULL, "return to launch point and loiter" },
	{ "takeoff",MAV_ACTION_TAKEOFF,    NULL, "start takeoff" },
	{ "land",   MAV_ACTION_LAND,       NULL, "start landing" },
	{ "roll",     0,		   cmd_roll,    "set fixed roll PWM" },
	{ "pitch",    0,		   cmd_pitch,   "set fixed pitch PWM" },
	{ "rudder",   0,		   cmd_rudder,  "set fixed rudder PWM" },
	{ "throttle", 0,		   cmd_throttle,"set fixed throttle PWM" },
	{ "switch", 0,       		   cmd_switch,  "set RC switch value (1-5), 0 disables" },
	{ "wp",	    0,			   cmd_wp,      "waypoint management (<load|save|list>)" },
	{ "param",  0,                     cmd_param,   "manage APM parameters (<fetch|edit|save|load>)" },
	{ "setup",  0,                     cmd_setup,   "go into setup mode (direct serial control)" },
	{ "reset",  0,                     cmd_reset,   "reopen the connection to the APM" },
	{ NULL, 0, NULL, NULL }
};

static void show_help(void)
{
	int i;
	for (i=0; commands[i].command; i++) {
		printf("\t%-10s   : %s\n", commands[i].command, commands[i].description);
	}
}

/*
  process input from the keyboard
 */
static void process_stdin(char *line)
{
	const int max_toks = 20;
	char *tok, *toks[max_toks];
	int num_toks;
	int i;

	if (line == NULL) {
		exit(0);
	}

	if (line[0] && !isspace(line[0])) {
		add_history(line);
	}

	if (setup_mode) {
		if (strcmp(line, ".") == 0) {
			setup_mode = false;
			rl_set_prompt("MAV> ");
			return;
		}
		write(fd_serial, line, strlen(line));
		write(fd_serial, "\r\n", 2);
		return;
	}

	num_toks = 0;
	for (tok = strtok(line, "\r\n "); tok; tok=strtok(NULL, "\r\n ")) {
		toks[num_toks++] = tok;
		if (num_toks == max_toks-1) {
			printf("too many tokens\n");
			continue;
		}
	}
	if (num_toks == 0) return;

	toks[num_toks] = NULL;
	
	if (strcmp(toks[0], "help") == 0) {
		show_help();
		return;
	}

	for (i=0; commands[i].command; i++) {
		if (strcmp(toks[0], commands[i].command) == 0) {
			if (commands[i].f == NULL) {
				/* its a mavlink action */
				mavlink_msg_action_send(0, TARGET_SYSTEM, 
							TARGET_COMPONENT, 
							commands[i].mav_action);
				return;
			}
			commands[i].f(num_toks-1, &toks[1]);
			return;
		}
	}

	printf("unknown command '%s' (try 'help')\n", toks[0]);
}


/*
  process input from the serial port
 */
static void process_serial(void)
{
	static char serial_buf[1024];
	static int serial_len;
	static bool is_mavlink;
	static mavlink_message_t msg;
	char c;
	mavlink_status_t mstatus;
	ssize_t len;

	len = read(fd_serial, &c, 1);
	if (len == -1 && (errno == EINTR || errno == EAGAIN)) {
		return;
	}
	if (len != 1) {
		close(fd_serial);
		printf("reopening serial port '%s'\n", serial_port);
		while ((fd_serial = open_serial(serial_port, serial_speed)) == -1) {
			sleep(1);
		}
		return;
	}

	if (setup_mode) {
		write(1, &c, 1);
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

	if (serial_buf[0] == '@') {
		int i, linenum = atoi(&serial_buf[1]);
		if (linenum >= 0 && linenum < NUM_MSG_LINES) {
			strncpy(status.msg_buf[linenum], &serial_buf[3], sizeof(status.msg_buf[0]));
			serial_len = 0;
			for (i=linenum+1; i<NUM_MSG_LINES; i++) {
				status.msg_buf[i][0] = 0;
			}
			return;
		}
	}

	status.serial_counter++;
	printf("APM: %s", serial_buf);
	strcpy(status.apm_buf, serial_buf);
	serial_len = 0;
}


/*
  process a packet from flightgear
 */
static void process_fg(void)
{
	struct fgIMUData buf;
	ssize_t len;

	len = read(fg_in, &buf, sizeof(buf));
	if (len == -1 && (errno == EINTR || errno == EAGAIN)) {
		return;
	}
	if (len != sizeof(buf)) {
		printf("Bad packet length %d from FG - expected %d : %s\n", 
		       (int)len, (int)sizeof(buf), strerror(errno));
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

/*
  process a packet from the ground control station
 */
static void process_gc(void)
{
	char buf[2048];
	ssize_t len;

	/* pass mavlink packets from gcs to APM */
	len = read(gc_sock, buf, sizeof(buf));
	if (len > 0) {
		write(fd_serial, buf, len);
		status.gc_counter++;
	}
}


static void limit_servo_speed(double *v, double last_v)
{
	const double change_limit = 0.02;
	if ((*v) - last_v > change_limit) {
		(*v) += change_limit;
	} else if (last_v - (*v) > change_limit) {
		(*v) -= change_limit;
	}
}

/*
  send current APM controls to flightgear
 */
static void send_to_fg(void)
{
	struct timeval tv;
	static uint64_t lastt;
	uint64_t t;
	static struct fgControlData last_fg;

	gettimeofday(&tv, NULL);
	t = (((uint64_t)tv.tv_sec) * 1000*1000) + tv.tv_usec;
	if ((t-lastt) > (1000*1000)/FG_FREQUENCY) {
		struct fgControlData fg = fgcontrol;		

		/* ensure we don't change values too rapidly - a real
		 * servo has movement limits */
		limit_servo_speed(&fg.aileron, last_fg.aileron);
		limit_servo_speed(&fg.elevator, last_fg.elevator);
		limit_servo_speed(&fg.throttle, last_fg.throttle);
		limit_servo_speed(&fg.rudder, last_fg.rudder);
		last_fg = fg;

		swap64(&fg, sizeof(fg)/8);
		lastt = t;
		write(fg_out, &fg, sizeof(fg));
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
	if (fd_serial == -1) {
		printf("Unable to open serial port: %s\n", strerror(errno));
		exit(1);
	}
	fg_in         = open_socket_in("127.0.0.1", IMU_LISTEN_PORT);
	fg_out        = open_socket_out("127.0.0.1", CTRL_SEND_PORT);
	gc_sock       = open_socket_out("127.0.0.1", QGCS_SEND_PORT);

	/* setup for readline handling */
	rl_callback_handler_install("MAV> ", process_stdin);

	printf("mavproxy started\n");

	while (1) {
		fd_set fds;
		int ret;
		struct timeval tv;

		FD_ZERO(&fds);
		FD_SET(fd_serial, &fds);
		FD_SET(fg_in, &fds);
		FD_SET(gc_sock, &fds);
		FD_SET(0, &fds);

		/* send data to flight gear every 20ms */
		send_to_fg();

		tv.tv_sec = 0;
		tv.tv_usec = 1000;

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

		if (FD_ISSET(gc_sock, &fds)) {
			process_gc();
			continue;
		}

		if (FD_ISSET(0, &fds)) {
			rl_callback_read_char();
			continue;
		}

		if (FD_ISSET(fd_serial, &fds)) {
			process_serial();
			continue;
		}
	}
	return 0;
}
