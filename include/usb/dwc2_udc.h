/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * drivers/usb/gadget/dwc2_udc.h
 * Designware DWC2 on-chip full/high speed USB device controllers
 * Copyright (C) 2005 for Samsung Electronics
 */

#ifndef __DWC2_USB_GADGET
#define __DWC2_USB_GADGET

#include <dm/ofnode.h>

#define PHY0_SLEEP              (1 << 5)
#define DWC2_MAX_HW_ENDPOINTS	16

struct dwc2_plat_otg_data {
	void		*priv;
	ofnode		phy_of_node;
	int		(*phy_control)(int on);
	uintptr_t	regs_phy;
	uintptr_t	regs_otg;
	unsigned int    usb_phy_ctrl;
	unsigned int    usb_flags;
	unsigned int	usb_gusbcfg;
	unsigned int	rx_fifo_sz;
	unsigned int	np_tx_fifo_sz;
	unsigned int	tx_fifo_sz;
	unsigned int	tx_fifo_sz_array[DWC2_MAX_HW_ENDPOINTS];
	unsigned char   tx_fifo_sz_nb;
	bool		force_b_session_valid;
	bool		force_vbus_detection;
	bool		activate_stm_id_vb_detection;
};

int dwc2_udc_probe(struct dwc2_plat_otg_data *pdata);

/*
 * Local diagnostics (ONYX RC2): counters of the non-DM gadget's bulk and
 * interrupt IN endpoints. The board clears and reads them.
 */
struct dwc2_udc_diag {
	u32 in_compl;		/* XferCompl taken */
	u32 in_bytes;		/* bytes given back as sent */
	u32 ahberr;		/* DIEPINT bits seen (cleared, not acted on) */
	u32 timeout;
	u32 intknepmis;
	u32 txfundrn;
	u32 bna;
	u32 gahbcfg;		/* value written at the last gadget start */
};

extern struct dwc2_udc_diag dwc2_udc_diag;

int dwc2_udc_B_session_valid(struct udevice *dev);

#endif	/* __DWC2_USB_GADGET */
