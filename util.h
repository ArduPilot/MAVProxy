void set_nonblocking(int fd);
int open_serial(const char *device, unsigned speed);
int open_socket_in(const char *listen_addr, int port);
int open_socket_out(const char *server, int port);
float constrain(float v, float minv, float maxv);
uint64_t get_usec(void);
void swap32(void *p, int n);
void swap64(void *p, int n);

#define ft2m(_x)        ((_x) * 0.3408)                         /* feet to metres */
#define dps2mrps(_x)    ((_x) * 17.453293)                      /* degrees per second to milliradians per second */
#define fpss2mg(_x)     ((_x) * 1000/ 32.2)                     /* feet per second per second to milligees */
#define rad2deg(_x)     fmod((((_x) * 57.29578) + 360), 360)    /* radians to degrees */
#define deg2rad(_x)     (((_x) / 360.0) * 2.0*M_PI)             /* degrees to radians */
#define kt2mps(_x)      ((_x)*1.94384449)                       /* knots to m/s */
