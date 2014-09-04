/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __LINUX_USB_BRIDGE_H__
#define __LINUX_USB_BRIDGE_H__

#include <linux/netdevice.h>
#include <linux/usb.h>

/* bridge device 0: DUN
 * bridge device 1 : Tethered RMNET
 */
#define MAX_BRIDGE_DEVICES 2

struct bridge_ops {
	int (*send_pkt)(void *, void *, size_t actual);
	void (*send_cbits)(void *, unsigned int);

	/* flow control */
	void (*unthrottle_tx)(void *);
};

#define TX_THROTTLED BIT(0)
#define RX_THROTTLED BIT(1)

struct bridge {
	/* context of the gadget port using bridge driver */
	void *ctx;

	/* bridge device array index mapped to the gadget port array index.
	 * data bridge[ch_id] <-- bridge --> gadget port[ch_id]
	 */
	unsigned int ch_id;

	/* flow control bits */
	unsigned long flags;

	/* data/ctrl bridge callbacks */
	struct bridge_ops ops;
};

//Move ctrl_bridge and data_bridge here for other files to use.
enum ctrl_bridge_rx_state {
	RX_IDLE, /* inturb is not queued */
	RX_WAIT, /* inturb is queued and waiting for data */
	RX_BUSY, /* inturb is completed. processing RX */
};

struct ctrl_bridge {
	struct usb_device	*udev;
	struct usb_interface	*intf;

	unsigned int		int_pipe;
	struct urb		*inturb;
	void			*intbuf;

	struct urb		*readurb;
	void			*readbuf;

	struct usb_anchor	tx_submitted;
	struct usb_anchor	tx_deferred;
	struct usb_ctrlrequest	*in_ctlreq;

	struct bridge		*brdg;
	struct platform_device	*pdev;

	unsigned long		flags;

	/* input control lines (DSR, CTS, CD, RI) */
	unsigned int		cbits_tohost;

	/* output control lines (DTR, RTS) */
	unsigned int		cbits_tomdm;

	spinlock_t lock;
	enum ctrl_bridge_rx_state rx_state;

	/* counters */
	unsigned int		snd_encap_cmd;
	unsigned int		get_encap_res;
	unsigned int		resp_avail;
	unsigned int		set_ctrl_line_sts;
	unsigned int		notify_ser_state;
};

struct data_bridge {
	struct usb_interface		*intf;
	struct usb_device		*udev;
	int				id;

	unsigned int			bulk_in;
	unsigned int			bulk_out;
	int				err;

	/* keep track of in-flight URBs */
	struct usb_anchor		tx_active;
	struct usb_anchor		rx_active;

	struct list_head		rx_idle;
	struct sk_buff_head		rx_done;

	struct workqueue_struct		*wq;
	struct work_struct		process_rx_w;

	struct bridge			*brdg;

	/* work queue function for handling halt conditions */
	struct work_struct		kevent;

	unsigned long			flags;

	struct platform_device		*pdev;

	/* counters */
	atomic_t			pending_txurbs;
	unsigned int			txurb_drp_cnt;
	unsigned long			to_host;
	unsigned long			to_modem;
	unsigned int			tx_throttled_cnt;
	unsigned int			tx_unthrottled_cnt;
	unsigned int			rx_throttled_cnt;
	unsigned int			rx_unthrottled_cnt;
};

/**
 * timestamp_info: stores timestamp info for skb life cycle during data
 * transfer for tethered rmnet/DUN.
 * @created: stores timestamp at the time of creation of SKB.
 * @rx_queued: stores timestamp when SKB queued to HW to receive
 * data.
 * @rx_done: stores timestamp when skb queued to h/w is completed.
 * @rx_done_sent: stores timestamp when SKB is sent from gadget rmnet/DUN
 * driver to bridge rmnet/DUN driver or vice versa.
 * @tx_queued: stores timestamp when SKB is queued to send data.
 *
 * note that size of this struct shouldnt exceed 48bytes that's the max skb->cb
 * holds.
 */
struct timestamp_info {
	struct data_bridge	*dev;

	unsigned int		created;
	unsigned int		rx_queued;
	unsigned int		rx_done;
	unsigned int		rx_done_sent;
	unsigned int		tx_queued;
};

/* Maximum timestamp message length */
#define DBG_DATA_MSG	128UL

/* Maximum timestamp messages */
#define DBG_DATA_MAX	32UL

/* timestamp buffer descriptor */
struct timestamp_buf {
	char		(buf[DBG_DATA_MAX])[DBG_DATA_MSG];   /* buffer */
	unsigned	idx;   /* index */
	rwlock_t	lck;   /* lock */
};

#if defined(CONFIG_USB_QCOM_MDM_BRIDGE) ||	\
	defined(CONFIG_USB_QCOM_MDM_BRIDGE_MODULE)

/* Bridge APIs called by gadget driver */
int ctrl_bridge_open(struct bridge *);
void ctrl_bridge_close(unsigned int);
int ctrl_bridge_write(unsigned int, char *, size_t);
int ctrl_bridge_set_cbits(unsigned int, unsigned int);
unsigned int ctrl_bridge_get_cbits_tohost(unsigned int);
int data_bridge_open(struct bridge *brdg);
void data_bridge_close(unsigned int);
int data_bridge_write(unsigned int , struct sk_buff *);
int data_bridge_unthrottle_rx(unsigned int);

/* defined in control bridge */
int ctrl_bridge_init(void);
void ctrl_bridge_exit(void);
int ctrl_bridge_probe(struct usb_interface *, struct usb_host_endpoint *, int);
void ctrl_bridge_disconnect(unsigned int);
int ctrl_bridge_resume(unsigned int);
int ctrl_bridge_suspend(unsigned int);

#else

static inline int __maybe_unused ctrl_bridge_open(struct bridge *brdg)
{
	return -ENODEV;
}

static inline void __maybe_unused ctrl_bridge_close(unsigned int id) { }

static inline int __maybe_unused ctrl_bridge_write(unsigned int id,
						char *data, size_t size)
{
	return -ENODEV;
}

static inline int __maybe_unused ctrl_bridge_set_cbits(unsigned int id,
					unsigned int cbits)
{
	return -ENODEV;
}

static inline unsigned int __maybe_unused
ctrl_bridge_get_cbits_tohost(unsigned int id)
{
	return -ENODEV;
}

static inline int __maybe_unused data_bridge_open(struct bridge *brdg)
{
	return -ENODEV;
}

static inline void __maybe_unused data_bridge_close(unsigned int id) { }

static inline int __maybe_unused data_bridge_write(unsigned int id,
					    struct sk_buff *skb)
{
	return -ENODEV;
}

static inline int __maybe_unused data_bridge_unthrottle_rx(unsigned int id)
{
	return -ENODEV;
}

#endif

#endif
