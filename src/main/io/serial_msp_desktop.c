#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

typedef struct serialConfig_s {
    void* unused;
} serialConfig_t;

int fc_serial_fd;
void mspSerialInit(serialConfig_t *serialConfig) {
  fc_serial_fd = open("/dev/fc_serial", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fc_serial_fd == -1) {
    perror("open_port: Unable to open /dev/ttyf1 - ");
  } else {
    fcntl(fc_serial_fd, F_SETFL, 0);
  }
}

