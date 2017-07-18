/*
 * Copyright (C) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/ccic/pdic_notifier.h>
#if defined(CONFIG_CCIC_NOTIFIER)
#include <linux/ccic/ccic_notifier.h>
#endif
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
#include <linux/usb/class-dual-role.h>
#endif
#ifndef __S2MU004_H__
#define __S2MU004_H__

#define USBPD_DEV_NAME	"usbpd-s2mu004"

#define S2MU004_MAX_NUM_MSG_OBJ (7)
#define S2MU004_MAX_NUM_INT_STATUS (7)

#define S2MU004_REG_TYPEC_DIS (1 << 2)
#define S2MU004_REG_LPM_EN (1 << 1)

#define S2MU004_REG_PLUG_CTRL_DRP (3)
#define S2MU004_REG_PLUG_CTRL_DFP (1)
#define S2MU004_REG_PLUG_CTRL_UFP (2)
#define S2MU004_REG_PLUG_CTRL_RP80 (1 << 4)
#define S2MU004_REG_PLUG_CTRL_RP180 (2 << 4)
#define S2MU004_REG_PLUG_CTRL_RP0 (0 << 4)
#define S2MU004_REG_PLUG_CTRL_RESET (~0x33)

#define S2MU004_REG_MSG_DATA_ROLE_MASK (0x1 << 5)

#define S2MU004_REG_INT_STATUS0_MSG_ACCEPT    (1<<0)
#define S2MU004_REG_INT_STATUS0_MSG_GOODCRC   (1<<1)
#define S2MU004_REG_INT_STATUS0_VDM_ATTENTION (1<<2)
#define S2MU004_REG_INT_STATUS0_VDM_EXIT      (1<<3)
#define S2MU004_REG_INT_STATUS0_VDM_ENTER     (1<<4)
#define S2MU004_REG_INT_STATUS0_VDM_DISCOVER_MODE (1<<5)
#define S2MU004_REG_INT_STATUS0_VDM_DISCOVER_SVID (1<<6)
#define S2MU004_REG_INT_STATUS0_VDM_DISCOVER_ID   (1<<7)

#define S2MU004_REG_INT_STATUS1_MSG_PING      (1<<7)
#define S2MU004_REG_INT_STATUS1_MSG_GOTOMIN   (1<<6)
#define S2MU004_REG_INT_STATUS1_MSG_REJECT    (1<<5)
#define S2MU004_REG_INT_STATUS1_MSG_PSRDY     (1<<4)
#define S2MU004_REG_INT_STATUS1_MSG_GETSRCCAP (1<<3)
#define S2MU004_REG_INT_STATUS1_MSG_GETSNKCAP (1<<2)
#define S2MU004_REG_INT_STATUS1_MSG_DR_SWAP   (1<<1)
#define S2MU004_REG_INT_STATUS1_MSG_PR_SWAP   (1<<0)

#define S2MU004_REG_INT_STATUS2_MSG_VCONN_SWAP (1<<7)
#define S2MU004_REG_INT_STATUS2_MSG_WAIT       (1<<6)
#define S2MU004_REG_INT_STATUS2_MSG_SRC_CAP    (1<<5)
#define S2MU004_REG_INT_STATUS2_MSG_SNK_CAP    (1<<4)
#define S2MU004_REG_INT_STATUS2_MSG_REQUEST    (1<<3)
#define S2MU004_REG_INT_STATUS2_MSG_SOFTRESET  (1<<2)
#define S2MU004_REG_INT_STATUS2_WAKEUP         (1<<0)

#define S2MU004_REG_INT_STATUS3_UNS_CMD_DATA   (1<<5)

#define S2MU004_REG_INT_STATUS4_CC12_DET_IRQ  (1<<6)
#define S2MU004_REG_INT_STATUS4_PLUG_IRQ      (1<<5)
#define S2MU004_REG_INT_STATUS4_USB_DETACH    (1<<4)
#define S2MU004_REG_INT_STATUS4_MSG_PASS      (1<<3)
#define S2MU004_REG_INT_STATUS4_MSG_SENT      (1<<2)
#define S2MU004_REG_INT_STATUS4_MSG_ERROR     (1<<1)

#define S2MU004_REG_PLUG_CTRL_CC_HOLD_BIT     (1<<0)

/* interrupt for checking message */
#define ENABLED_INT_0	(S2MU004_REG_INT_STATUS0_MSG_ACCEPT)
#define ENABLED_INT_1	(S2MU004_REG_INT_STATUS1_MSG_DR_SWAP |\
			S2MU004_REG_INT_STATUS1_MSG_PR_SWAP |\
			S2MU004_REG_INT_STATUS1_MSG_GETSRCCAP |\
			S2MU004_REG_INT_STATUS1_MSG_GETSNKCAP |\
			S2MU004_REG_INT_STATUS1_MSG_REJECT |\
			S2MU004_REG_INT_STATUS1_MSG_PSRDY)
#define ENABLED_INT_2	(S2MU004_REG_INT_STATUS2_MSG_SRC_CAP |\
			S2MU004_REG_INT_STATUS2_MSG_SNK_CAP |\
			S2MU004_REG_INT_STATUS2_MSG_REQUEST |\
			S2MU004_REG_INT_STATUS2_MSG_SOFTRESET |\
			S2MU004_REG_INT_STATUS2_MSG_VCONN_SWAP)
#define ENABLED_INT_3	0
#define ENABLED_INT_4	(S2MU004_REG_INT_STATUS4_USB_DETACH |\
				S2MU004_REG_INT_STATUS4_PLUG_IRQ |\
				S2MU004_REG_INT_STATUS4_MSG_PASS)
#define ENABLED_INT_4_PASS	(S2MU004_REG_INT_STATUS4_USB_DETACH |\
				S2MU004_REG_INT_STATUS4_PLUG_IRQ |\
				S2MU004_REG_INT_STATUS4_MSG_PASS)

#define S2MU004_PDIC_PLUG_ATTACH_DONE_SHT	(1)
#define S2MU004_PDIC_SINK_SEL_MONITOR_SHT	(2)
#define S2MU004_PDIC_SOURCE_SEL_MONITOR_SHT	(3)

#define S2MU004_PDIC_SINK (1 << S2MU004_PDIC_SINK_SEL_MONITOR_SHT \
		| 1 << S2MU004_PDIC_PLUG_ATTACH_DONE_SHT)
#define S2MU004_PDIC_SOURCE (1 << S2MU004_PDIC_SOURCE_SEL_MONITOR_SHT \
		| 1 << S2MU004_PDIC_PLUG_ATTACH_DONE_SHT)

#define S2MU004_PR_MASK (S2MU004_PDIC_SINK | S2MU004_PDIC_SOURCE)

/* For S2MU004_REG_MSG_SEND_CON */
#define S2MU004_REG_MSG_SEND_CON_SEND_MSG_EN	0x01
#define S2MU004_REG_MSG_SEND_CON_OP_MODE	0x02
#define S2MU004_REG_MSG_SEND_CON_SOP		0x00
#define S2MU004_REG_MSG_SEND_CON_SOP_Prime	(0x01 << 2)
#define S2MU004_REG_MSG_SEND_CON_SOP_DPrime	(0x02 << 2)
#define S2MU004_REG_MSG_SEND_CON_SOP_PDebug	(0x03 << 2)
#define S2MU004_REG_MSG_SEND_CON_SOP_DPDebug	(0x04 << 2)
#define S2MU004_REG_MSG_SEND_CON_SOP_HardRST	(0x05 << 2)
#define S2MU004_REG_MSG_SEND_CON_SOP_CableRST	(0x06 << 2)
#define S2MU004_REG_PLUG_CTRL_RpRd_Rp_Source_Mode ((0x1 << 7) | (0x1 << 3))
#define S2MU004_REG_PLUG_CTRL_RpRd_Rd_Sink_Mode (0x1 << 7)
#define S2MU004_REG_PLUG_CTRL_RpRd_CC1_VCONN    (0x30)
#define S2MU004_REG_PLUG_CTRL_RpRd_CC2_VCONN    (0x50)
#define S2MU004_REG_PLUG_CTRL_CC1_MANUAL_ON     (0x30)
#define S2MU004_REG_PLUG_CTRL_CC2_MANUAL_ON     (0x50)

#define S2MU004_PDIC_RID_SHIFT			5
#define S2MU004_PDIC_RID_MASK			(0x7 << S2MU004_PDIC_RID_SHIFT)

enum s2mu004_power_role {
	PDIC_SINK,
	PDIC_SOURCE
};

enum s2mu004_pdic_rid {
	REG_RID_UNDF = 0x00,
	REG_RID_255K = 0x03,
	REG_RID_301K = 0x04,
	REG_RID_523K = 0x05,
	REG_RID_619K = 0x06,
	REG_RID_OPEN = 0x07,
	REG_RID_MAX  = 0x08,
};


/* S2MU004 I2C registers */
enum s2mu004_usbpd_reg {
	S2MU004_REG_PD_CTRL		    = 0x01,
	S2MU004_REG_PD_CTRL_2		    = 0x02,
	S2MU004_REG_PLUG_CTRL_PORT          = 0x18,
	S2MU004_REG_PLUG_CTRL_MSG           = 0x19,
	S2MU004_REG_PLUG_CTRL_SET_RD        = 0x1E,
	S2MU004_REG_PLUG_CTRL_SET_RP        = 0x1F,
	S2MU004_REG_PLUG_CTRL_CC_HOLD       = 0x26,
	S2MU004_REG_PLUG_CTRL_RpRd          = 0x27,
	S2MU004_REG_PLUG_CTRL_CC12          = 0x28,
	S2MU004_REG_PLUG_CTRL               = 0x2E,
	S2MU004_REG_CTRL                    = 0x2F,

	S2MU004_REG_INT_MASK0	            = 0x3E,
	S2MU004_REG_INT_MASK1	            = 0x3F,
	S2MU004_REG_INT_MASK2	            = 0x40,
	S2MU004_REG_INT_MASK3	            = 0x41,
	S2MU004_REG_INT_MASK4	            = 0x42,
	S2MU004_REG_INT_STATUS0	            = 0xE0,
	S2MU004_REG_INT_STATUS1	            = 0xE1,
	S2MU004_REG_INT_STATUS2	            = 0xE2,
	S2MU004_REG_INT_STATUS3	            = 0xE3,
	S2MU004_REG_INT_STATUS4	            = 0xE4,
	S2MU004_REG_ADC_STATUS              = 0xB2,
	S2MU004_REG_PLUG_MON1               = 0xB3,
	S2MU004_REG_PLUG_MON2               = 0xB4,

	S2MU004_REG_MSG_SEND_CON            = 0x90,
	S2MU004_REG_MSG_HEADER_L            = 0x91,
	S2MU004_REG_MSG_HEADER_H            = 0x92,
	S2MU004_REG_MSG_OBJECT0_0_L         = 0x93,
	S2MU004_REG_MSG_OBJECT0_0_H         = 0x94,
	S2MU004_REG_MSG_OBJECT0_1_L         = 0x95,
	S2MU004_REG_MSG_OBJECT0_1_H         = 0x96,
	S2MU004_REG_MSG_OBJECT1_0_L         = 0x97,
	S2MU004_REG_MSG_OBJECT1_0_H         = 0x98,
	S2MU004_REG_MSG_OBJECT1_1_L         = 0x99,
	S2MU004_REG_MSG_OBJECT1_1_H         = 0x9A,
	S2MU004_REG_MSG_OBJECT2_0_L         = 0x9B,
	S2MU004_REG_MSG_OBJECT2_0_H         = 0x9C,
	S2MU004_REG_MSG_OBJECT2_1_L         = 0x9D,
	S2MU004_REG_MSG_OBJECT2_1_H         = 0x9E,
	S2MU004_REG_MSG_OBJECT3_0_L         = 0x9F,
	S2MU004_REG_MSG_OBJECT3_0_H         = 0xA0,
	S2MU004_REG_MSG_OBJECT3_1_L         = 0xA1,
	S2MU004_REG_MSG_OBJECT3_1_H         = 0xA2,
	S2MU004_REG_MSG_OBJECT4_0_L         = 0xA3,
	S2MU004_REG_MSG_OBJECT4_0_H         = 0xA4,
	S2MU004_REG_MSG_OBJECT4_1_L         = 0xA5,
	S2MU004_REG_MSG_OBJECT4_1_H         = 0xA6,
	S2MU004_REG_MSG_OBJECT5_0_L         = 0xA7,
	S2MU004_REG_MSG_OBJECT5_0_H         = 0xA8,
	S2MU004_REG_MSG_OBJECT5_1_L         = 0xA9,
	S2MU004_REG_MSG_OBJECT5_1_H         = 0xAA,
	S2MU004_REG_MSG_OBJECT6_0_L         = 0xAB,
	S2MU004_REG_MSG_OBJECT6_0_H         = 0xAC,
	S2MU004_REG_MSG_OBJECT6_1_L         = 0xAD,
	S2MU004_REG_MSG_OBJECT6_1_H         = 0xAE
};
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
typedef enum {
	TYPE_C_DETACH = 0,
	TYPE_C_ATTACH_DFP = 1, /* Host */
	TYPE_C_ATTACH_UFP = 2, /* Device */
	TYPE_C_ATTACH_DRP = 3, /* Dual role */
} CCIC_OTP_MODE;

typedef enum {
	PLUG_CTRL_RP0 = 0,
	PLUG_CTRL_RP80 = 1,
	PLUG_CTRL_RP180 = 2,
	PLUG_CTRL_RP330 = 3
} CCIC_RP_SCR_SEL;

#define DUAL_ROLE_SET_MODE_WAIT_MS 1500
#endif
typedef enum {
	CLIENT_OFF = 0,
	CLIENT_ON = 1,
} CCIC_DEVICE_REASON;

typedef enum {
	HOST_OFF = 0,
	HOST_ON = 1,
} CCIC_HOST_REASON;

#if defined(CONFIG_CCIC_NOTIFIER)
struct ccic_state_work {
	struct work_struct ccic_work;
	int dest;
	int id;
	int attach;
	int event;
};
#endif

struct s2mu004_usbpd_data {
	struct device *dev;
	struct i2c_client *i2c;
#if defined(CONFIG_CCIC_NOTIFIER)
	struct workqueue_struct *ccic_wq;
#endif
	struct mutex _mutex;
	struct mutex poll_mutex;
	struct mutex lpm_mutex;
	int vconn_en;
	int irq_gpio;
	int irq;
	int power_role;
	int data_role;
	int vconn_source;
	msg_header_type header;
	data_obj_type obj[S2MU004_MAX_NUM_MSG_OBJ];
	u64 status_reg;
	bool lpm_mode;
	bool detach_valid;
	bool is_factory_mode;
	bool is_water_detect;
	bool is_otg_vboost;
	bool is_otg_reboost;
	int check_msg_pass;
	int rid;
	int is_host;
	int is_client;
	int data_role_dual; /* data_role for dual role swap */
	int power_role_dual; /* power_role for dual role swap */
	int is_attached;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc *desc;
	struct completion reverse_completion;
	int try_state_change;
	struct delayed_work role_swap_work;
#endif
	struct notifier_block type3_nb;
	struct workqueue_struct *pdic_queue;
	struct delayed_work plug_work;
	struct s2mu004_pdic_notifier_struct pdic_notifier;
};

extern int s2mu004_usbpd_get_adc(void);
extern void s2mu004_usbpd_set_muic_type(int type);
#if defined(CONFIG_CCIC_NOTIFIER)
extern void s2mu004_control_option_command(struct s2mu004_usbpd_data *usbpd_data, int cmd);
void select_pdo(int num);
void ccic_event_work(void *data, int dest, int id, int attach, int event);
#endif
int s2mu004_set_lpm_mode(struct s2mu004_usbpd_data *pdic_data);
int s2mu004_set_normal_mode(struct s2mu004_usbpd_data *pdic_data);

#endif /* __S2MU004_H__ */
