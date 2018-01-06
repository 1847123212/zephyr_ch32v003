/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_DOMAIN "usb/dc"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_USB_DRIVER_LEVEL
#include <logging/sys_log.h>

#include <drivers/usb/usb_dc.h>
#include <qlog.h>
#include <soc.h>
#include <string.h>

#define NVM_USB_PAD_TRANSN_POS 45
#define NVM_USB_PAD_TRANSN_SIZE 5
#define NVM_USB_PAD_TRANSP_POS 50
#define NVM_USB_PAD_TRANSP_SIZE 5
#define NVM_USB_PAD_TRIM_POS 55
#define NVM_USB_PAD_TRIM_SIZE 3

#define USB_SAM0_IN_EP	0x80

struct usb_sam0_data {
	UsbDeviceDescriptor descriptors[USB_EPT_NUM];

	usb_dc_status_callback cb;
	usb_dc_ep_callback ep_cb[2][USB_EPT_NUM];

	u8_t addr;
	u32_t ep_buf[USB_EPT_NUM * 64 / 4];
	int brk;
	u32_t out_at;
};

static struct usb_sam0_data usb_sam0_data_0;

static const struct soc_gpio_pin usb_sam0_dm_pin = PIN_USB_SAM0_DM;
static const struct soc_gpio_pin usb_sam0_dp_pin = PIN_USB_SAM0_DP;

static struct usb_sam0_data *usb_sam0_get_data(void)
{
	return &usb_sam0_data_0;
}

static void usb_sam0_ep_isr(u8_t ep)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	UsbDevice *regs = &USB->DEVICE;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep];
	u32_t intflag = endpoint->EPINTFLAG.reg;

	QLOG2("ep=%x intflag=0x%x", ep, intflag);

	endpoint->EPINTFLAG.reg = intflag;

	if ((intflag & USB_DEVICE_EPINTFLAG_RXSTP) != 0) {
		data->ep_cb[0][ep](ep, USB_DC_EP_SETUP);
	}

	if ((intflag & USB_DEVICE_EPINTFLAG_TRCPT0) != 0) {
		data->ep_cb[0][ep](ep, USB_DC_EP_DATA_OUT);
	}

	if ((intflag & USB_DEVICE_EPINTFLAG_TRCPT1) != 0) {
		data->ep_cb[1][ep](ep | USB_SAM0_IN_EP, USB_DC_EP_DATA_IN);

		if (data->addr != 0) {
			QLOG("Applying new addr");
			regs->DADD.reg = data->addr;
			data->addr = 0;
		}
	}
}

static void usb_sam0_isr(void)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	UsbDevice *regs = &USB->DEVICE;
	u32_t intflag = regs->INTFLAG.reg;
	u32_t epint = regs->EPINTSMRY.reg;

	regs->INTFLAG.reg = intflag;

	if ((intflag & USB_DEVICE_INTFLAG_EORST) != 0) {
		UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[0];

		endpoint->EPINTENSET.reg =
			USB_DEVICE_EPINTENSET_TRCPT0 |
			USB_DEVICE_EPINTENSET_TRCPT1 |
			USB_DEVICE_EPINTENSET_RXSTP;

		data->cb(USB_DC_RESET, NULL);
	}

	while (epint) {
		int ep = find_lsb_set(epint) - 1;

		epint &= ~(1 << ep);
		usb_sam0_ep_isr((u8_t)ep);
	}
}

static void usb_sam0_wait_syncbusy(void)
{
	UsbDevice *regs = &USB->DEVICE;

	while (regs->SYNCBUSY.reg != 0) {
	}
}

static void usb_sam0_load_padcal(void)
{
	UsbDevice *regs = &USB->DEVICE;
	u32_t pad_transn;
	u32_t pad_transp;
	u32_t pad_trim;

	/* Load Pad Calibration */
	pad_transn = (*((uint32_t *)(NVMCTRL_OTP4) +
			(NVM_USB_PAD_TRANSN_POS / 32)) >>
		      (NVM_USB_PAD_TRANSN_POS % 32)) &
		((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

	if (pad_transn == 0x1F) {
		pad_transn = 5;
	}

	regs->PADCAL.bit.TRANSN = pad_transn;

	pad_transp = (*((uint32_t *)(NVMCTRL_OTP4) +
			(NVM_USB_PAD_TRANSP_POS / 32)) >>
		      (NVM_USB_PAD_TRANSP_POS % 32)) &
		     ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

	if (pad_transp == 0x1F) {
		pad_transp = 29;
	}

	regs->PADCAL.bit.TRANSP = pad_transp;

	pad_trim = (*((uint32_t *)(NVMCTRL_OTP4) +
		      (NVM_USB_PAD_TRIM_POS / 32)) >>
		    (NVM_USB_PAD_TRIM_POS % 32)) &
		   ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

	if (pad_trim == 0x7) {
		pad_trim = 3;
	}

	regs->PADCAL.bit.TRIM = pad_trim;
}

int usb_dc_attach(void)
{
	UsbDevice *regs = &USB->DEVICE;
	struct usb_sam0_data *data = usb_sam0_get_data();

	/* Enable the clock in PM */
	PM->APBBMASK.bit.USB_ = 1;

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_USB | GCLK_CLKCTRL_GEN_GCLK0 |
			    GCLK_CLKCTRL_CLKEN;

	while (GCLK->STATUS.bit.SYNCBUSY) {
	}

	/* Connect pins to the peripheral */
	soc_gpio_configure(&usb_sam0_dm_pin);
	soc_gpio_configure(&usb_sam0_dp_pin);

	/* Configure */
	regs->CTRLA.bit.SWRST = 1;
	usb_sam0_wait_syncbusy();

	/* Change QOS values to have the best performance and correct USB
	 * behaviour
	 */
	regs->QOSCTRL.bit.CQOS = 2;
	regs->QOSCTRL.bit.DQOS = 2;

	usb_sam0_load_padcal();

	regs->CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
	regs->CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_HS;

	memset(data->descriptors, 0, sizeof(data->descriptors));
	regs->DESCADD.reg = (uintptr_t)&data->descriptors[0];

	regs->INTENSET.reg = USB_DEVICE_INTENSET_EORST;

	/* Connect and enable the interrupt */
	IRQ_CONNECT(USB_IRQn, CONFIG_USB_DC_SAM0_IRQ_PRI, usb_sam0_isr, 0, 0);

	irq_enable(USB_IRQn);

	/* Enable */
	regs->CTRLA.bit.ENABLE = 1;
	usb_sam0_wait_syncbusy();

	regs->CTRLB.bit.DETACH = 0;

	return 0;
}

int usb_dc_detach(void)
{
	UsbDevice *regs = &USB->DEVICE;

	regs->CTRLB.bit.DETACH = 1;
	usb_sam0_wait_syncbusy();

	return 0;
}

int usb_dc_reset(void)
{
	UsbDevice *regs = &USB->DEVICE;

	irq_disable(USB_IRQn);

	regs->CTRLA.bit.SWRST = 1;
	usb_sam0_wait_syncbusy();

	return 0;
}

int usb_dc_set_address(const u8_t addr)
{
	struct usb_sam0_data *data = usb_sam0_get_data();

	data->addr = addr | USB_DEVICE_DADD_ADDEN;

	return 0;
}

int usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	struct usb_sam0_data *data = usb_sam0_get_data();

	data->cb = cb;

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const cfg)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	UsbDevice *regs = &USB->DEVICE;
	u8_t for_in = cfg->ep_addr & USB_EP_DIR_MASK;
	u8_t ep = cfg->ep_addr & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep];
	UsbDeviceDescriptor *desc = &data->descriptors[ep];
	int type;
	int size;

	QLOG2("addr=0x%x type=%d", cfg->ep_addr, cfg->ep_type);

	switch (cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		type = 1;
		break;
	case USB_DC_EP_ISOCHRONOUS:
		type = 2;
		break;
	case USB_DC_EP_BULK:
		type = 3;
		break;
	case USB_DC_EP_INTERRUPT:
		type = 4;
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->ep_mps) {
	case 8:
		size = 0;
		break;
	case 16:
		size = 1;
		break;
	case 32:
		size = 2;
		break;
	case 64:
		size = 3;
		break;
	case 128:
		size = 4;
		break;
	case 256:
		size = 5;
		break;
	case 512:
		size = 6;
		break;
	case 1023:
		size = 7;
		break;
	default:
		return -EINVAL;
	}

	if (for_in) {
		endpoint->EPCFG.bit.EPTYPE1 = type;
		desc->DeviceDescBank[1].PCKSIZE.bit.SIZE = size;
		desc->DeviceDescBank[1].ADDR.reg =
			(uintptr_t)&data->ep_buf[data->brk];
		endpoint->EPSTATUSCLR.bit.BK1RDY = 1;
	} else {
		endpoint->EPCFG.bit.EPTYPE0 = type;
		desc->DeviceDescBank[0].PCKSIZE.bit.SIZE = size;
		desc->DeviceDescBank[0].ADDR.reg =
			(uintptr_t)&data->ep_buf[data->brk];
		endpoint->EPSTATUSCLR.bit.BK0RDY = 1;
	}

	data->brk += cfg->ep_mps / sizeof(u32_t);

	return 0;
}

int usb_dc_ep_set_stall(const u8_t ep)
{
	UsbDevice *regs = &USB->DEVICE;
	u8_t for_in = ep & USB_EP_DIR_MASK;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];

	if (for_in) {
		endpoint->EPSTATUSSET.bit.STALLRQ1 = 1;
	} else {
		endpoint->EPSTATUSSET.bit.STALLRQ0 = 1;
	}

	return 0;
}

int usb_dc_ep_clear_stall(const u8_t ep)
{
	UsbDevice *regs = &USB->DEVICE;
	u8_t for_in = ep & USB_EP_DIR_MASK;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];

	if (for_in) {
		endpoint->EPSTATUSCLR.bit.STALLRQ1 = 1;
	} else {
		endpoint->EPSTATUSCLR.bit.STALLRQ0 = 1;
	}

	return 0;
}

int usb_dc_ep_is_stalled(const u8_t ep, u8_t *stalled)
{
	UsbDevice *regs = &USB->DEVICE;
	u8_t for_in = ep & USB_EP_DIR_MASK;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];

	if (for_in) {
		*stalled = endpoint->EPSTATUS.bit.STALLRQ1;
	} else {
		*stalled = endpoint->EPSTATUS.bit.STALLRQ0;
	}

	return 0;
}

int usb_dc_ep_enable(const u8_t ep)
{
	UsbDevice *regs = &USB->DEVICE;
	u8_t for_in = ep & USB_EP_DIR_MASK;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];

	QLOG1("ep=%x", ep);

	if (for_in) {
		endpoint->EPSTATUSCLR.bit.BK1RDY = 1;
	} else {
		endpoint->EPSTATUSCLR.bit.BK0RDY = 1;
	}

	endpoint->EPINTENSET.reg = USB_DEVICE_EPINTENSET_TRCPT0 |
				   USB_DEVICE_EPINTENSET_TRCPT1 |
				   USB_DEVICE_EPINTENSET_RXSTP;

	return 0;
}

int usb_dc_ep_write(u8_t ep, const u8_t *buf, u32_t len, u32_t *ret_bytes)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	UsbDevice *regs = &USB->DEVICE;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];
	UsbDeviceDescriptor *desc = &data->descriptors[ep_num];
	u32_t addr = desc->DeviceDescBank[1].ADDR.reg;

	QLOG2("ep=%x len=%d", ep, len);

	if (endpoint->EPSTATUS.bit.BK1RDY) {
		/* Write in progress, drop */
		QLOG("busy");
		return -EAGAIN;
	}

	memcpy((void *)addr, buf, len);
	desc->DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
	desc->DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = len;
	endpoint->EPINTFLAG.reg =
		USB_DEVICE_EPINTFLAG_TRCPT1 | USB_DEVICE_EPINTFLAG_TRFAIL1;
	endpoint->EPSTATUSSET.bit.BK1RDY = 1;

	*ret_bytes = len;

	return 0;
}

int usb_dc_ep_read_ex(u8_t ep, u8_t *buf, u32_t max_data_len,
		      u32_t *read_bytes, bool wait)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	UsbDevice *regs = &USB->DEVICE;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];
	UsbDeviceDescriptor *desc = &data->descriptors[ep_num];
	u32_t addr = desc->DeviceDescBank[0].ADDR.reg;
	u32_t bytes = desc->DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT;
	u32_t take;
	int remain;

	if (!endpoint->EPSTATUS.bit.BK0RDY) {
		SYS_LOG_WRN("!BK0RDY");
		return -EAGAIN;
	}

	if (buf == NULL) {
		data->out_at = 0;

		QLOG1("in_buf=%d", bytes);
		if (read_bytes != NULL) {
			*read_bytes = bytes;
		}

		return 0;
	}

	remain = bytes - data->out_at;
	take = min(max_data_len, remain);
	memcpy(buf, (void *)addr, take);

	if (read_bytes != NULL) {
		*read_bytes = take;
	}

	if (take == remain) {
		QLOG2("in_buf=%d took=%d (all)", bytes, take);
		if (!wait) {
			endpoint->EPSTATUSCLR.bit.BK0RDY = 1;
			data->out_at = 0;
		}
	} else {
		data->out_at += take;
	}

	return 0;
}

int usb_dc_ep_read(u8_t ep, u8_t *buf, u32_t max_data_len, u32_t *read_bytes)
{
	return usb_dc_ep_read_ex(ep, buf, max_data_len, read_bytes, false);
}

int usb_dc_ep_read_wait(u8_t ep, u8_t *buf, u32_t max_data_len,
			u32_t *read_bytes)
{
	return usb_dc_ep_read_ex(ep, buf, max_data_len, read_bytes, true);
}

int usb_dc_ep_read_continue(u8_t ep)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	UsbDevice *regs = &USB->DEVICE;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;
	UsbDeviceEndpoint *endpoint = &regs->DeviceEndpoint[ep_num];

	endpoint->EPSTATUSCLR.bit.BK0RDY = 1;
	data->out_at = 0;

	return 0;
}

int usb_dc_ep_set_callback(const u8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_sam0_data *data = usb_sam0_get_data();
	u8_t for_in = ep & USB_EP_DIR_MASK;
	u8_t ep_num = ep & ~USB_EP_DIR_MASK;

	data->ep_cb[for_in ? 1 : 0][ep_num] = cb;

	return 0;
}
