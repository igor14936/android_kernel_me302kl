#ifndef TTY_AT_H
#define TTY_AT_H

#include <mach/usb_bridge.h>

/* Use it to enable/disable the tty_at feature */
//#define DISABLE_TTY_AT

/* AT_ID is the same as dun id */
#define AT_ID 0

bool is_tty_at_opened(void);
void set_usb_open(bool flag);

void set_ctrl_dev(struct ctrl_bridge *dev);
void set_data_dev(struct data_bridge *dev);

int tty_at_setup(void);
void tty_at_release(void);
void tty_at_probe(void);
void tty_at_disconnect(void);

void tty_data_bridge_process_rx(struct work_struct *work, unsigned int timestamp);

/* Be defined in mdm_data_bridge.c */
int tty_submit_rx_urb(struct data_bridge *dev, struct urb *rx_urb, gfp_t flags);

#endif
