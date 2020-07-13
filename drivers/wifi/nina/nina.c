/*
 * Copyright (c) 2019 Tobias Svehagen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_nina

#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wifi_nina);

#include <kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <init.h>
#include <stdlib.h>

#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/uart.h>

#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/socket_offload.h>
#include <net/wifi_mgmt.h>

#include "nina_private.h"

K_THREAD_STACK_DEFINE(nina_workq_stack, 1024);

#define NINA_MAX_SOCKETS 10
#define NINA_INIT_TIMEOUT K_MSEC(2000)

enum nina_socket_flags {
	NINA_SOCK_IN_USE = BIT(1),
	NINA_SOCK_CONNECTING = BIT(2),
	NINA_SOCK_CONNECTED = BIT(3)
};

#define NINA_NUM_SOCKETS 4

struct nina_socket {
        struct nina_data *data;
        uint8_t link_id;
        bool connected;

	/* socket info */
	sa_family_t family;
	enum net_sock_type type;
	enum net_ip_protocol ip_proto;
	struct sockaddr src;
	struct sockaddr dst;

	net_context_connect_cb_t connect_cb;
	net_context_send_cb_t send_cb;
	net_context_recv_cb_t recv_cb;

	/* callback data */
	void *conn_user_data;
	void *send_user_data;
	void *recv_user_data;

	struct k_sem sem_data_ready;
	struct net_pkt *tx_pkt;

	struct k_work connect_work;
	struct k_work send_work;
	struct k_work recv_work;
};

K_MEM_SLAB_DEFINE(nina_sockets, sizeof(struct nina_socket), NINA_NUM_SOCKETS, 2);

struct nina_data {
	struct spi_config spi_config;
	struct spi_cs_control cs_ctrl;

	struct net_if *net_iface;

	struct device *spi_dev;

	struct device *ready_dev;
	struct device *reset_dev;
	struct device *irq_dev;

	uint8_t ready_pin;
	uint8_t reset_pin;
	uint8_t irq_pin;

	struct k_work_q workq;
	struct k_work init_work;
	struct k_work scan_work;
	struct k_delayed_work status_work;
	scan_result_cb_t scan_cb;
	struct k_sem sem_if_up;
	uint8_t status;

        struct nina_socket *slots[NINA_NUM_SOCKETS];
};

static struct nina_data nina_data_0;

#define DEV_DATA(dev) ((struct nina_data *)(dev)->driver_data)

static struct net_offload nina_offload;

struct nina_config {
};

struct nina_args {
	uint8_t buf[1];
};

struct nina_result {
	uint8_t buf[1];
};

void nina_args_init(struct nina_args *args)
{
}

void nina_args_addw(struct nina_args *args, uint32_t v)
{
}

void nina_args_addh(struct nina_args *args, uint16_t v)
{
}

void nina_args_addb(struct nina_args *args, uint8_t v)
{
}

static int nina_wait(struct nina_data *n)
{
	uint32_t start;
	int ret;

	for (start = k_uptime_get_32();
	     (int32_t)(k_uptime_get_32() - start) < 10000;) {
		ret = gpio_pin_get(n->ready_dev, n->ready_pin);
		if (ret < 0) {
			return ret;
		}
		if (ret == 0) {
			return 0;
		}
	}
	LOG_INF("timed out");
	return -ETIMEDOUT;
}

static int nina_wait_stable(struct nina_data *n)
{
	LOG_INF("enter");

	uint32_t now = k_uptime_get_32();
	uint32_t stop = k_uptime_get_32() + 2000;
	uint32_t high = now;
	int err;

	for (; (int32_t)(now - stop) < 0; now = k_uptime_get_32()) {
		err = gpio_pin_get(n->ready_dev, n->ready_pin);
		if (err < 0) {
			return err;
		}
		if (err == 0) {
			if ((int32_t)(now - high) > 500) {
				return 0;
			}
		} else {
			high = now;
		}
		k_msleep(1);
	}
	return -1;
}

static int nina_rx(struct nina_data *n, const uint8_t *buf, uintptr_t len,
		   bool final)
{
	struct spi_buf rx_buf = {
		.buf = (uint8_t *)buf,
		.len = len,
	};
	struct spi_buf_set rx_bufs = {
		.buffers = &rx_buf,
		.count = 1,
	};
	int err;

	if (final) {
		n->spi_config.operation &= ~SPI_HOLD_ON_CS;
	} else {
		n->spi_config.operation |= SPI_HOLD_ON_CS;
	}
	err = spi_read(n->spi_dev, &n->spi_config, &rx_bufs);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int nina_sendrecv(struct nina_data *n, const uint8_t *req, uintptr_t len)
{
	LOG_INF("nina_sendrecv len=%d", (int)len);
	int err;

	err = nina_wait(n);
	if (err != 0) {
		return err;
	}
	struct spi_buf tx_buf = {
		.buf = (void *)req,
		.len = len,
	};
	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1,
	};
	err = spi_write(n->spi_dev, &n->spi_config, &tx_bufs);
	if (err != 0) {
		return err;
	}

	/* Receive the header */
	err = nina_wait(n);
	if (err != 0) {
		return err;
	}

	uint8_t rx[3];
	err = nina_rx(n, rx, sizeof(rx), false);
	if (err != 0) {
		return err;
	}
	LOG_INF("rx[..] = %x %x %x", rx[0], rx[1], rx[2]);
	if (rx[0] != NINA_CMD_START) {
		LOG_INF("got %x, want NINA_CMD_START", rx[0]);
		return -EINVAL;
	}
	if (rx[1] != (req[1] | NINA_FLAG_REPLY)) {
		LOG_INF("got %x, want %x", rx[1], req[1] | NINA_FLAG_REPLY);
		return -EINVAL;
	}

	return rx[2];
}

static int nina_param(struct nina_data *n, uint8_t *buf, uintptr_t cap,
		      bool final)
{
	uint8_t len;
	int res;

	res = nina_rx(n, &len, sizeof(len), false);
	if (res < 0) {
		return res;
	}
	if (len >= cap) {
		return -ENOMEM;
	}
	res = nina_rx(n, buf, len, final);
	if (res < 0) {
		return res;
	}
	return len;
}

static int nina_sendrecv_1(struct nina_data *n, const uint8_t *req,
			   uintptr_t len)
{
	uint8_t buf[sizeof(int)];
	int res;

	res = nina_sendrecv(n, req, len);
	if (res < 0) {
		return res;
	}

	res = nina_param(n, buf, sizeof(buf), true);
	if (res < 0) {
		return res;
	}

	/* TODO */
	return buf[0];
}

static int nina_fw_version(struct nina_data *n, char *ver, uintptr_t len)
{
	const uint8_t req[] = {
		NINA_CMD_START,
		NINA_CMD_GET_FW_VERSION,
		0,
		NINA_CMD_END,
	};
	int res;

	res = nina_sendrecv(n, req, sizeof(req));
	if (res < 0) {
		return res;
	}
	if (res < 1) {
		return -EINVAL;
	}

	res = nina_param(n, ver, len - 1, true);
	if (res < 0) {
		return res;
	}
	ver[res] = '\0';

	return 0;
}

static void nina_connect_work(struct k_work *work)
{
        LOG_WRN("nina_connect_work");
}

static void nina_send_work(struct k_work *work)
{
        LOG_WRN("nina_send_work");
}

static void nina_recv_work(struct k_work *work)
{
        LOG_WRN("nina_recv_work");
}

static void nina_init_work(struct k_work *work)
{
	struct nina_data *data =
		CONTAINER_OF(work, struct nina_data, init_work);
	int err;

	LOG_INF("nina_init_work");

	if (net_if_is_up(data->net_iface)) {
		net_if_down(data->net_iface);
	}

	if (data->reset_dev != NULL) {
		LOG_INF("toggling reset");
		err = gpio_pin_configure(data->reset_dev, data->reset_pin,
					 GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
		if (err != 0) {
			goto error;
		}
		k_msleep(1);
		err = gpio_pin_set(data->reset_dev, data->reset_pin, 0);
		if (err != 0) {
			goto error;
		}
		k_msleep(1);
		err = gpio_pin_set(data->reset_dev, data->reset_pin, 1);
		if (err != 0) {
			goto error;
		}
	}

	/* Wait for ready to go stable */
	err = nina_wait_stable(data);
	if (err != 0) {
		goto error;
	}

	uint8_t ver[10];
	err = nina_fw_version(data, ver, sizeof(ver));
	if (err != 0) {
		goto error;
	}

	net_if_up(data->net_iface);

        LOG_INF("nina inited");
	k_sem_give(&data->sem_if_up);

	return;

error:
	LOG_INF("nina init error");
}

static void nina_scan_work(struct k_work *work)
{
	struct nina_data *data =
		CONTAINER_OF(work, struct nina_data, scan_work);

	LOG_INF("nina_scan_work");

	const uint8_t req[] = {
		NINA_CMD_START,
		NINA_CMD_SCAN_NETWORKS,
		0,
		NINA_CMD_END,
	};
	int res;
	int num;

	res = nina_sendrecv(data, req, sizeof(req));
	if (res < 0) {
		goto error;
	}

	num = res;
	for (int i = 0; i < num; i++) {
		struct wifi_scan_result entry = {
			.channel = 0,
		};
		res = nina_param(data, entry.ssid, sizeof(entry.ssid),
				 i == (num - 1));
		if (res < 0) {
			goto error;
		}
		entry.ssid_length = res;

		data->scan_cb(data->net_iface, 0, &entry);
	}

	data->scan_cb(data->net_iface, 0, NULL);
	data->scan_cb = NULL;

	return;
error:
	LOG_ERR("nina_scan_work err=%d", res);

	data->scan_cb(data->net_iface, res, NULL);
	data->scan_cb = NULL;
}

static void nina_status_work(struct k_work *work)
{
	struct nina_data *data =
		CONTAINER_OF(work, struct nina_data, status_work);
	int res;

	LOG_INF("nina_status_work");

	uint8_t req[] = {
		NINA_CMD_START,
		NINA_CMD_GET_CONN_STATUS,
		0,
		NINA_CMD_END,
	};
	res = nina_sendrecv_1(data, req, sizeof(req));
	if (res < 0) {
	}
	if (res != data->status) {
		data->status = res;

		if (res == NINA_STATUS_CONNECTED) {
			wifi_mgmt_raise_connect_result_event(data->net_iface,
							     0);
		}
	}

        if (res != NINA_STATUS_CONNECTED) {
                k_delayed_work_submit_to_queue(&data->workq, &data->status_work,
                                               K_SECONDS(1));
        }
}

static void nina_iface_init(struct net_if *iface)
{
	LOG_INF("nina_iface_init");

	struct device *dev = net_if_get_device(iface);
	struct nina_data *data = dev->driver_data;
        int res;

	net_if_flag_set(iface, NET_IF_NO_AUTO_START);
	data->net_iface = iface;
	iface->if_dev->offload = &nina_offload;

	k_work_submit_to_queue(&data->workq, &data->init_work);

	LOG_INF("Waiting for interface to come up");

	res = k_sem_take(&data->sem_if_up, NINA_INIT_TIMEOUT);
	if (res == -EAGAIN) {
		LOG_ERR("Timeout waiting for interface");
	}
}

static int nina_iface_scan(struct device *dev, scan_result_cb_t cb)
{
	LOG_INF("nina_iface_scan");

	struct nina_data *data = dev->driver_data;

	if (data->scan_cb != NULL) {
		return -EINPROGRESS;
	}

	if (!net_if_is_up(data->net_iface)) {
		return -EIO;
	}

	data->scan_cb = cb;

	k_work_submit_to_queue(&data->workq, &data->scan_work);

	return 0;
}

static int nina_iface_connect(struct device *dev,
			      struct wifi_connect_req_params *params)
{
	LOG_INF("nina_iface_connect sec=%d", params->security);

	struct nina_data *data = dev->driver_data;
	int res;

	if (!net_if_is_up(data->net_iface)) {
		return -EIO;
	}

	if (params->security == WIFI_SECURITY_TYPE_PSK) {
		uint8_t req[WIFI_SSID_MAX_LEN + 64 + 10] = {
			NINA_CMD_START,
			NINA_CMD_SET_PASSPHRASE,
			2,
		};
		uint8_t *p = req + 3;
		*p++ = params->ssid_length;
		memcpy(p, params->ssid, params->ssid_length);
		p += params->ssid_length;
		*p++ = params->psk_length;
		memcpy(p, params->psk, params->psk_length);
		p += params->psk_length;
		*p++ = NINA_CMD_END;

		res = nina_sendrecv_1(data, req, p - req);
	} else {
		uint8_t req[WIFI_SSID_MAX_LEN + 10] = {
			NINA_CMD_START,
			NINA_CMD_SET_NET,
			1,
		};
		uint8_t *p = req + 3;
		*p++ = params->ssid_length;
		memcpy(p, params->ssid, params->ssid_length);
		p += params->ssid_length;
		*p++ = NINA_CMD_END;

		res = nina_sendrecv_1(data, req, p - req);
	}

	LOG_INF("res=%d", res);
	if (res < 0) {
		return res;
	}
	if (res != 1) {
		return -EINVAL;
	}

	k_delayed_work_submit_to_queue(&data->workq, &data->status_work,
				       K_SECONDS(1));
	return 0;
}

static int nina_iface_disconnect(struct device *dev)
{
	LOG_INF("nina_iface_disconnect");
	return -EINVAL;
}

static int nina_iface_ap_enable(struct device *dev,
				struct wifi_connect_req_params *params)
{
	LOG_INF("nina_iface_ap_enable");
	return -EINVAL;
}

static int nina_iface_ap_disable(struct device *dev)
{
	LOG_INF("nina_iface_ap_disable");
	return -EINVAL;
}

struct net_wifi_mgmt_offload nina_mgmt_api = {
	.iface_api.init = nina_iface_init,
	.scan = nina_iface_scan,
	.connect = nina_iface_connect,
	.disconnect = nina_iface_disconnect,
	.ap_enable = nina_iface_ap_enable,
	.ap_disable = nina_iface_ap_disable,
};

static int nina_get(sa_family_t family, enum net_sock_type type,
		    enum net_ip_protocol ip_proto, struct net_context **context)
{
	struct nina_socket *sock;
        struct nina_data *data = &nina_data_0;
        int res;

	LOG_INF("nina_get");

	if (family != AF_INET) {
		return -EAFNOSUPPORT;
	}

        res = k_mem_slab_alloc(&nina_sockets, (void**)&sock, K_NO_WAIT);
        if (res != 0) {
                return res;
        }

        uint8_t cmd[] = {
                NINA_CMD_START,
                NINA_CMD_GET_SOCKET,
                0,
                NINA_CMD_END,
        };

        int link_id = nina_sendrecv_1(data, cmd, sizeof(cmd));
        LOG_INF("link_id=%d", link_id);

        if (link_id < 0) {
                return -ENOMEM;
        }

        *sock = (struct nina_socket){
                .data = data,
                .link_id = UINT8_MAX,
        };

	k_work_init(&sock->connect_work, nina_connect_work);
	k_work_init(&sock->send_work, nina_send_work);
	k_work_init(&sock->recv_work, nina_recv_work);

	(*context)->offload_context = sock;

        LOG_INF("nina_get done");
	return 0;
}

static int nina_bind(struct net_context *context, const struct sockaddr *addr,
		     socklen_t addrlen)
{
	LOG_INF("nina_bind");

        struct nina_socket *sock = 	(struct nina_socket *)context->offload_context;

	sock->src.sa_family = addr->sa_family;

	if (IS_ENABLED(CONFIG_NET_IPV4) && addr->sa_family == AF_INET) {
		net_ipaddr_copy(&net_sin(&sock->src)->sin_addr,
				&net_sin(addr)->sin_addr);
		net_sin(&sock->src)->sin_port = net_sin(addr)->sin_port;
	} else {
		return -EAFNOSUPPORT;
	}

	return 0;
}

static int nina_listen(struct net_context *context, int backlog)
{
	LOG_INF("nina_listen");
	return -EINVAL;
}

static int nina_connect(struct net_context *context,
			const struct sockaddr *addr, socklen_t addrlen,
			net_context_connect_cb_t cb, int32_t timeout,
			void *user_data)
{
        uint8_t cmd[] = {
		NINA_CMD_START,
                NINA_CMD_START_CLIENT_TCP,
                4,
                //      (addr),
                2,
//                (port),
//                1, socket,
//                1, type
        };
	LOG_INF("nina_connect");
	return 0;
}

static int nina_accept(struct net_context *context, net_tcp_accept_cb_t cb,
		       int32_t timeout, void *user_data)
{
	LOG_INF("nina_accept");
	return -EINVAL;
}

static int nina_sendto(struct net_pkt *pkt, const struct sockaddr *dst_addr,
		       socklen_t addrlen, net_context_send_cb_t cb,
		       int32_t timeout, void *user_data)
{
	LOG_WRN("nina_sendto");

	struct net_context *context;
	struct nina_socket *sock;
	int ret = 0;

	context = pkt->context;
	sock = (struct nina_socket *)context->offload_context;

	LOG_DBG("link %d, timeout %d", sock->link_id, timeout);

	if (sock->tx_pkt) {
		return -EBUSY;
	}

	if (sock->type == SOCK_STREAM) {
                return -ENOTCONN;
	} else {
		if (!sock->connected) {
			if (!dst_addr) {
				return -ENOTCONN;
			}

			/* Use a timeout of 5000 ms here even though the
			 * timeout parameter might be different. We want to
			 * have a valid link id before proceeding.
			 */
			ret = nina_connect(context, dst_addr, addrlen, NULL,
					  (5 * MSEC_PER_SEC), NULL);
			if (ret < 0) {
				return ret;
			}
		} else if (dst_addr && memcmp(dst_addr, &sock->dst, addrlen)) {
			/* This might be unexpected behaviour but the ESP
			 * doesn't support changing endpoint.
			 */
			return -EISCONN;
		}
	}

	sock->tx_pkt = pkt;
	sock->send_cb = cb;
	sock->send_user_data = user_data;

	if (timeout == 0) {
		k_work_submit_to_queue(&sock->data->workq, &sock->send_work);
		return 0;
	}

        LOG_INF("sendto!!");
	/* if (ret == 0) { */
	/* 	net_pkt_unref(sock->tx_pkt); */
	/* } else { */
	/* 	LOG_ERR("Failed to send data: link %d, ret %d", sock->link_id, */
	/* 		ret); */
	/* } */

	sock->tx_pkt = NULL;

	if (cb) {
		cb(context, ret, user_data);
	}

	return ret;
}

static int nina_send(struct net_pkt *pkt, net_context_send_cb_t cb,
		     int32_t timeout, void *user_data)
{
	LOG_WRN("nina_send");
	return nina_sendto(pkt, NULL, 0, cb, timeout, user_data);
}

static int nina_recv(struct net_context *context, net_context_recv_cb_t cb,
		     int32_t timeout, void *user_data)
{
	LOG_WRN("nina_recv");

	struct nina_socket *sock;
	int ret;

	sock = (struct nina_socket *)context->offload_context;

	LOG_DBG("link_id %d, timeout %d, cb 0x%x, data 0x%x", sock->link_id,
		timeout, (int)cb, (int)user_data);

	sock->recv_cb = cb;
	sock->recv_user_data = user_data;
	k_sem_reset(&sock->sem_data_ready);

	if (timeout == 0) {
		return 0;
	}

	ret = k_sem_take(&sock->sem_data_ready, K_MSEC(timeout));

	sock->recv_cb = NULL;

	return ret;
}

static int nina_put(struct net_context *context)
{
	struct nina_socket *sock = (struct nina_socket *)context->offload_context;

	LOG_INF("nina_put");

        k_mem_slab_free(&nina_sockets, (void**)&sock);

        return 0;
}

static struct net_offload nina_offload = {
	.get = nina_get,
	.bind = nina_bind,
	.listen = nina_listen,
	.connect = nina_connect,
	.accept = nina_accept,
	.send = nina_send,
	.sendto = nina_sendto,
	.recv = nina_recv,
	.put = nina_put,
};

static int nina_init(struct device *dev)
{
	LOG_INF("nina_init");

	struct nina_data *data = DEV_DATA(dev);
	int err;

	data->spi_dev = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!data->spi_dev) {
		LOG_DBG("No such SPI device");
		return -EPERM;
	}

	data->spi_config = (struct spi_config){
		.frequency = DT_INST_PROP(0, spi_max_frequency),
		.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
		.slave = DT_INST_REG_ADDR(0),
	};
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->cs_ctrl = (struct spi_cs_control){
		.gpio_dev =
			device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0)),
		.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
		.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
	};
	if (data->cs_ctrl.gpio_dev == NULL) {
		LOG_DBG("No such SPI CS GPIO device");
		return -EPERM;
	}
	data->spi_config.cs = &data->cs_ctrl;
	if (gpio_config(data->cs_ctrl.gpio_dev, data->cs_ctrl.gpio_pin,
			GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH) != 0) {
		return -EPERM;
	}
#endif

	data->ready_dev =
		device_get_binding(DT_INST_GPIO_LABEL(0, ready_gpios));
	if (data->ready_dev == NULL) {
		return -EPERM;
	}
	data->ready_pin = DT_INST_GPIO_PIN(0, ready_gpios);
	err = gpio_pin_configure(data->ready_dev, data->ready_pin,
				 GPIO_INPUT | GPIO_PULL_UP);
	if (err != 0) {
		return err;
	}

	data->irq_dev = device_get_binding(DT_INST_GPIO_LABEL(0, irq_gpios));
	if (data->irq_dev == NULL) {
		return -EPERM;
	}
	data->irq_pin = DT_INST_GPIO_PIN(0, irq_gpios);
	err = gpio_pin_configure(data->irq_dev, data->irq_pin,
				 GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
	if (err != 0) {
		return err;
	}

	/* Reset is optional */
	data->reset_dev =
		device_get_binding(DT_INST_GPIO_LABEL(0, reset_gpios));
	data->reset_pin = DT_INST_GPIO_PIN(0, reset_gpios);
	if (data->reset_dev != NULL) {
		err = gpio_pin_configure(data->reset_dev, data->reset_pin,
					 GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
		if (err != 0) {
			return err;
		}
	}

	k_work_q_start(&data->workq, nina_workq_stack,
		       K_THREAD_STACK_SIZEOF(nina_workq_stack), 7);
	k_sem_init(&data->sem_if_up, 0, 1);
	k_work_init(&data->init_work, nina_init_work);
	k_work_init(&data->scan_work, nina_scan_work);
	k_delayed_work_init(&data->status_work, nina_status_work);

	return 0;
}

#define NINA_MTU 2048

NET_DEVICE_OFFLOAD_INIT(wifi_nina, DT_INST_LABEL(0), nina_init,
			device_pm_control_nop, &nina_data_0, NULL,
			CONFIG_WIFI_INIT_PRIORITY, &nina_mgmt_api, NINA_MTU);
