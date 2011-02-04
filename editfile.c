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
#include <sys/wait.h>
#include <stdarg.h>
#include <termios.h>
#include <ctype.h>
#include <sys/time.h>
#include <time.h>
#include "util.h"
#include "editfile.h"

static pid_t editor_pid;
static char *edit_filename;
static edit_file_callback_t edit_callback;

static void child_handler(int signum)
{
	pid_t pid;
	int status;

	pid = waitpid(-1, &status, WNOHANG);
	if (pid != editor_pid) {
		return;
	}

	edit_callback(edit_filename, status);
	editor_pid = 0;
	free(edit_filename);
}

/*
  start an editor on a file. Call callback() when complete
 */
void edit_file_background(const char *fname, edit_file_callback_t callback)
{
	const char *editor;
	
	/* work out what editor to use */
	editor = getenv("EDITOR");
	if (editor == NULL) {
		editor = getenv("VISUAL");
	}
	if (editor == NULL) {
		editor = "vi";
	}

	if (editor_pid != 0) {
		fprintf(stderr, "Edit already started\n");
		return;
	}

	edit_filename = strdup(fname);
	edit_callback = callback;

	signal(SIGCHLD, child_handler);

	editor_pid = fork();
	if (editor_pid == -1) {
		/* the system is very sick! best to exit */
		exit(1);
	}

	if (editor_pid != 0) {
		return;
	}

	if (execlp(editor, edit_filename, edit_filename, NULL) != 0) {
		fprintf(stderr, "Failed to started editor '%s' : %s\n",
			editor, strerror(errno));
		exit(1);
	}
}
