#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "pl_lib.h"

int debug(int fd)
{
    if (ioctl(fd, DEBUG) == -1) {
        perror(__func__);
        return -1;
    }
    return 0;
}

int pl_open(int *fd) {
    if ( (*fd = open(DEVNAME, O_RDWR)) <= 0 ) {
        perror(__func__);
        return -1;
    }

    return 0;
}

int pl_close(int fd) {
    return close(fd);
}

u_int32_t pl_register_read(int fd, u_int32_t addr) {
    pldrv_io_t p;
    p.address = addr;// / sizeof(uint32_t);
    p.data = 0xbeef;
    
    if (ioctl(fd, READ_REG, &p) == -1) {
        perror(__func__);
        return -1;
    }
    
    return p.data;
}

int pl_register_write(int fd, u_int32_t addr, u_int32_t data) {
    pldrv_io_t p;
    p.address = addr;// / sizeof(uint32_t);
    p.data = data;
    
    if (ioctl(fd, WRITE_REG, &p) == -1) {
        perror(__func__);
        return -1;
    }
    
    return 0;
}

int pl_debug(int fd) {
    debug(fd);
    return 0;
}

    
