#ifndef PL_LIB_H
#define PL_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pl_ioctl.h"

#define DEVNAME "/dev/vipic"
#define PAGE_SIZE 4096

int pl_open(int *fd);
int pl_close(int fd);
int debug(int fd);
u_int32_t pl_register_read(int fd, u_int32_t addr);
int pl_register_write(int fd, u_int32_t addr, u_int32_t data);
int fee_register_write(int fd, u_int32_t id, u_int32_t addr, u_int32_t data);
u_int16_t fee_register_read(int fd, u_int32_t id, u_int32_t addr);
int pl_trigger_dma(int fd);
u_int32_t pl_get_dma_status(int fd);
int pl_set_rate(int fd, u_int32_t data);
int pl_set_buff_len(int fd, u_int32_t len);
int pl_set_brust_len(int fd, u_int32_t len);
int pl_set_debug_level(int fd, u_int32_t level);
int pl_debug(int fd);
int dam_wait_for_dma(int fd, int *data);
int dam_enable_irq(int fd);
int dam_disable_irq(int fd);
int dam_reset_dma_engine(int fd);

#ifdef __cplusplus
}
#endif

#endif // PL_LIB_H
