#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include "tty_at.h"

#define TTY_NAME "ttyAT"

#define ACM_CTRL_DTR (1 << 0)
#define ACM_CTRL_RTS (1 << 1)

#define PADING_TTY_SIZE (16)
#define RX_HALT   BIT(1)

//variable declaration
static const unsigned int stop_submit_urb_limit = 500;
static bool s_is_bridge_init = false;
static bool s_is_tty_at_opened = false;
static bool s_is_usb_opened = false;
static struct tty_struct *tty_at = NULL;
static struct tty_driver *tty_at_driver = NULL;
static struct device *tty_at_dev = NULL;
static struct ctrl_bridge *__ctrl_dev = NULL;
static struct data_bridge *__data_dev = NULL;

//set and get functions
bool is_tty_at_opened(void)
{
	return s_is_tty_at_opened;
}

void set_usb_open(bool flag)
{
	s_is_usb_opened = flag;
}

void set_ctrl_dev(struct ctrl_bridge *dev)
{
	__ctrl_dev = dev;
}

void set_data_dev(struct data_bridge *dev)
{
	__data_dev = dev;
}

//ctrl bridge functions
static int tty_ctrl_bridge_set_cbits(unsigned int cbits)
{
	struct ctrl_bridge *dev;
	int retval;

	dev = __ctrl_dev;
	if (!dev)
		return -ENODEV;

	pr_debug("%s: cbits : %u\n", __func__, cbits);

	dev->cbits_tomdm = cbits;

	retval = ctrl_bridge_write(AT_ID, NULL, 0);

	return retval;
}

static int tty_ctrl_bridge_open(void)
{
	struct ctrl_bridge *dev;

	dev = __ctrl_dev;
	if (!dev) {
		err("dev is null\n");
		return -ENODEV;
	}

	if (s_is_usb_opened)
		goto done;

	dev->brdg = NULL;
	dev->snd_encap_cmd = 0;
	dev->get_encap_res = 0;
	dev->resp_avail = 0;
	dev->set_ctrl_line_sts = 0;
	dev->notify_ser_state = 0;

done:
	return 0;
}

static void tty_ctrl_bridge_close(void)
{
	struct ctrl_bridge *dev;

	dev = __ctrl_dev;
	if (!dev)
		return;

	dev_dbg(&dev->intf->dev, "%s:\n", __func__);

	if (!s_is_usb_opened) {
		usb_unlink_anchored_urbs(&dev->tx_submitted);
	}
}

//data bridge functions
static void tty_rx_string(struct sk_buff *skb)
{
	if (tty_at != NULL) {
		unsigned char *ptr;
		int avail = 0;

		avail = tty_prepare_flip_string(tty_at, &ptr, skb->len);

		if (avail <= 0) {
			pr_err("tty_prepare_flip_string err\n");
		} else {
			memcpy(ptr, skb->data, avail);
			tty_flip_buffer_push(tty_at);
		}
		dev_kfree_skb_any(skb);
	}
}

void tty_data_bridge_process_rx(struct work_struct *work,unsigned int timestamp)
{
	int                   retval;
	unsigned long         flags;
	struct urb            *rx_idle;
	struct sk_buff        *skb;
	struct timestamp_info *info;
	struct data_bridge    *dev =
		container_of(work, struct data_bridge, process_rx_w);

	if (test_bit(RX_HALT, &dev->flags))
		return;

	while ((skb = skb_dequeue(&dev->rx_done))) {
		dev->to_host++;
		info = (struct timestamp_info *)skb->cb;
		info->rx_done_sent = timestamp;
		//push buffer for tty core.
		tty_rx_string(skb);
	}

	spin_lock_irqsave(&dev->rx_done.lock, flags);
	while (!list_empty(&dev->rx_idle)) {
		if (dev->rx_done.qlen > stop_submit_urb_limit)
			break;

		rx_idle = list_first_entry(&dev->rx_idle, struct urb, urb_list);
		list_del(&rx_idle->urb_list);
		spin_unlock_irqrestore(&dev->rx_done.lock, flags);
		retval = tty_submit_rx_urb(dev, rx_idle, GFP_KERNEL);
		spin_lock_irqsave(&dev->rx_done.lock, flags);
		if (retval) {
			list_add_tail(&rx_idle->urb_list, &dev->rx_idle);
			break;
		}
	}
	spin_unlock_irqrestore(&dev->rx_done.lock, flags);
}

static void tty_data_bridge_close(void)
{
	struct data_bridge *dev;
	struct sk_buff     *skb;
	unsigned long      flags;

	dev  = __data_dev;
	if (!dev)
		return;

	dev_dbg(&dev->intf->dev, "%s:\n", __func__);

	usb_kill_anchored_urbs(&dev->tx_active);
	usb_kill_anchored_urbs(&dev->rx_active);

	spin_lock_irqsave(&dev->rx_done.lock, flags);
	while ((skb = __skb_dequeue(&dev->rx_done)))
		dev_kfree_skb_any(skb);
	spin_unlock_irqrestore(&dev->rx_done.lock, flags);
}

static int tty_data_bridge_open(void)
{
	struct data_bridge *dev;

	pr_debug("%s\n", __func__);

	dev = __data_dev;
	if (!dev) {
		err("dev is null\n");
		return -ENODEV;
	}

	dev_dbg(&dev->udev->dev, "%s: dev:%p\n", __func__, dev);
	if (s_is_usb_opened)
		goto done;

	dev->brdg = NULL;
	dev->err = 0;
	atomic_set(&dev->pending_txurbs, 0);
	dev->to_host = 0;
	dev->to_modem = 0;
	dev->txurb_drp_cnt = 0;
	dev->tx_throttled_cnt = 0;
	dev->tx_unthrottled_cnt = 0;
	dev->rx_throttled_cnt = 0;
	dev->rx_unthrottled_cnt = 0;

	queue_work(dev->wq, &dev->process_rx_w);

done:
	return 0;
}

//tty driver
static int tty_at_open(struct tty_struct *tty, struct file *f)
{
	int rc = 0;

	pr_debug("%s: usb open %d\n", __func__, s_is_usb_opened);

	if (tty_at != NULL)
	{
		pr_err("%s: already opened\n", __func__);
		return -EBUSY;
	}

	if (!s_is_bridge_init) {
		pr_err("%s: bridge isn't initalize\n", __func__);
		return -ENODEV;
	}
	s_is_tty_at_opened = true;

	rc = tty_ctrl_bridge_open();
	if (rc < 0) {
		goto failed;
	}

	rc = tty_data_bridge_open();
	if (rc < 0) {
		tty_ctrl_bridge_close();
		goto failed;
	}

	tty_at = tty;
	return 0;

failed:
	s_is_tty_at_opened = false;
	return rc;
}

static void tty_at_close(struct tty_struct *tty, struct file *f)
{
	pr_debug("%s: usb open %d\n", __func__, s_is_usb_opened);

	if (!s_is_bridge_init) {
		pr_err("%s: bridge isn't initalize\n", __func__);
		return;
	}

	tty_at = NULL;
	s_is_tty_at_opened = false;

	if (!s_is_usb_opened) {
		tty_data_bridge_close();
		tty_ctrl_bridge_close();
	}

	return;
}

static int tty_at_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
	int rc;
	struct sk_buff *skb;

	if (!s_is_bridge_init) {
		pr_err("%s: bridge isn't initalize\n", __func__);
		return -ENODEV;
	}
	skb = alloc_skb(len + PADING_TTY_SIZE, GFP_ATOMIC);
	if (!skb) {
		pr_err("%s: alloc_skb error\n", __func__);
		return -ENOMEM;
	}

	memcpy(skb_put(skb, len), buf, len);

	rc = data_bridge_write(AT_ID, skb);
	if (rc < 0) {
		return 0;
	}
	return len;
}

static int tty_at_write_room(struct tty_struct *tty)
{
	return 8192;
}

static int tty_at_chars_in_buffer(struct tty_struct *tty)
{
	return 8192;
}

static void tty_at_unthrottle(struct tty_struct *tty)
{
	pr_debug("%s: not support\n", __func__);
	return;
}

static int tty_at_tiocmget(struct tty_struct *tty)
{
	pr_debug("%s: not support\n", __func__);
	return 0;
}

static int tty_at_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
	pr_debug("%s: not support\n", __func__);
	return 0;
}

static const struct tty_operations tty_at_ops = {
	.open = tty_at_open,
	.close = tty_at_close,
	.write = tty_at_write,
	.write_room = tty_at_write_room,
	.chars_in_buffer = tty_at_chars_in_buffer,
	.unthrottle = tty_at_unthrottle,
	.tiocmget = tty_at_tiocmget,
	.tiocmset = tty_at_tiocmset,
};

int tty_at_setup(void)
{
	int ret;

	if (tty_at_driver == NULL) {
		tty_at_driver = alloc_tty_driver(1);
		if (tty_at_driver == NULL)
			return -ENOMEM;

		tty_at_driver->owner = THIS_MODULE;
		tty_at_driver->driver_name = TTY_NAME;
		tty_at_driver->name = TTY_NAME;
		tty_at_driver->major = 0;
		tty_at_driver->minor_start = 0;
		tty_at_driver->type = TTY_DRIVER_TYPE_SERIAL;
		tty_at_driver->subtype = SERIAL_TYPE_NORMAL;
		tty_at_driver->init_termios = tty_std_termios;
		tty_at_driver->init_termios.c_iflag = 0;
		tty_at_driver->init_termios.c_oflag = 0;
		tty_at_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
		tty_at_driver->init_termios.c_lflag = 0;
		tty_at_driver->flags = TTY_DRIVER_RESET_TERMIOS |
			TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
		tty_set_operations(tty_at_driver, &tty_at_ops);

		ret = tty_register_driver(tty_at_driver);
		if (ret) goto failed1;
	} else {
		pr_info("use old driver %p\n", tty_at_driver);
	}

	if (tty_at_dev == NULL) {
		/* this should be dynamic */
		tty_at_dev = tty_register_device(tty_at_driver, 0, 0);
		if (IS_ERR(tty_at_dev)) {
			ret = (int)tty_at_dev;
			goto failed2;
		}
	} else {
		pr_info("use old dev %p\n", tty_at_dev);
	}
	pr_info("%s: %s\n", __func__, TTY_NAME);
	return 0;

failed2:
	tty_unregister_driver(tty_at_driver);
failed1:
	put_tty_driver(tty_at_driver);

	tty_at_dev = NULL;
	tty_at_driver = NULL;
	return ret;
}

void tty_at_release(void)
{
	if (tty_at_dev != NULL) {
		tty_unregister_device(tty_at_driver, 0);
	}
	if (tty_at_driver != NULL) {
		tty_unregister_driver(tty_at_driver);
		put_tty_driver(tty_at_driver);
	}
	tty_at_dev = NULL;
	tty_at_driver = NULL;
	tty_ctrl_bridge_set_cbits(0);
	pr_info("%s: %s\n", __func__, TTY_NAME);
}

void tty_at_probe(void)
{
	pr_info("%s\n", __func__);
	tty_ctrl_bridge_set_cbits(ACM_CTRL_RTS | ACM_CTRL_DTR);
	s_is_bridge_init = true;
}

void tty_at_disconnect(void)
{
	pr_info("%s\n", __func__);
	s_is_bridge_init = false;

	s_is_tty_at_opened = false;
	s_is_usb_opened = false;
	tty_at = NULL;
}

