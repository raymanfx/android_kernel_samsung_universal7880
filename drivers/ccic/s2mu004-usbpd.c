/*
 driver/usbpd/s2mu004.c - S2MU004 USB PD(Power Delivery) device driver
 *
 * Copyright (C) 2016 Samsung Electronics
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

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include <linux/ccic/usbpd.h>
#include <linux/ccic/usbpd-s2mu004.h>

#include <linux/mfd/samsung/s2mu004.h>
#include <linux/mfd/samsung/s2mu004-private.h>

#include <linux/muic/muic.h>
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */
#include <linux/sec_batt.h>
#include <linux/battery/sec_charging_common.h>
#ifdef CONFIG_USB_HOST_NOTIFY
#include <linux/usb_notify.h>
#endif
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
#include <linux/battery/battery_notifier.h>
#endif

char *rid_text[] = {
	"UNDEFINED",
	"RID ERROR",
	"RID ERROR",
	"RID 255K",
	"RID 301K",
	"RID 523K",
	"RID 619K"
};
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
static enum dual_role_property fusb_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};
#endif

static usbpd_phy_ops_type s2mu004_ops;
static int s2mu004_receive_message(void *data);
struct i2c_client *test_i2c;
int s2mu004_set_normal_mode(struct s2mu004_usbpd_data *pdic_data);
int s2mu004_check_port_detect(struct s2mu004_usbpd_data *pdic_data);
static int s2mu004_usbpd_reg_init(struct s2mu004_usbpd_data *_data);
static void s2mu004_dfp(struct i2c_client *i2c);
static void s2mu004_ufp(struct i2c_client *i2c);
#if 0 /* disable function that support dp control */
static int s2mu004_usbpd_check_vdm_msg(/*struct usbpd_data */ void *_data, int *val);
#endif
#if 1
static void s2mu004_src(struct i2c_client *i2c);
static void s2mu004_snk(struct i2c_client *i2c);
static void s2mu004_assert_rd(void *_data);
static void s2mu004_assert_rp(void *_data);
static int s2mu004_set_attach(struct s2mu004_usbpd_data *pdic_data, u8 mode);
static int s2mu004_set_detach(struct s2mu004_usbpd_data *pdic_data, u8 mode);
#endif
/* #if defined(CONFIG_CCIC_FACTORY) */
static void s2mu004_usbpd_check_rid(struct s2mu004_usbpd_data *pdic_data);
/* #endif */

int s2mu004_usbpd_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int s2mu004_usbpd_write_reg(struct i2c_client *i2c, u8 reg, u8 value);

#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
struct pdic_notifier_struct pd_noti;
#endif

void vbus_turn_on_ctrl(struct s2mu004_usbpd_data *usbpd_data, bool enable)
{
	struct power_supply *psy_otg;
	union power_supply_propval val;
	int on = !!enable;
	int ret = 0;

	struct otg_notify *o_notify = get_otg_notify();
	if (enable && o_notify)
		o_notify->hw_param[USB_CCIC_OTG_USE_COUNT]++;

	pr_info("%s %d, enable=%d\n", __func__, __LINE__, enable);
	psy_otg = get_power_supply_by_name("otg");

	if (psy_otg) {
		val.intval = enable;
		usbpd_data->is_otg_vboost = enable;
		ret = psy_otg->set_property(psy_otg, POWER_SUPPLY_PROP_ONLINE, &val);
	} else {
		pr_err("%s: Fail to get psy battery\n", __func__);
	}
	if (ret) {
		pr_err("%s: fail to set power_suppy ONLINE property(%d)\n",
			__func__, ret);
	} else {
		pr_info("otg accessory power = %d\n", on);
	}

}

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
void s2mu004_rprd_mode_change(struct s2mu004_usbpd_data *usbpd_data, u8 mode)
{
	u8 data = 0;
	struct i2c_client *i2c = usbpd_data->i2c;
	pr_info("%s, mode=0x%x\n", __func__, mode);

	mutex_lock(&usbpd_data->lpm_mutex);
	if (usbpd_data->lpm_mode)
		goto skip;

	switch (mode) {
	case TYPE_C_ATTACH_DFP: /* SRC */
		s2mu004_set_detach(usbpd_data, mode);
		msleep(1400);
		s2mu004_set_attach(usbpd_data, mode);
		break;
	case TYPE_C_ATTACH_UFP: /* SNK */
		s2mu004_set_detach(usbpd_data, mode);
		msleep(1400);
		s2mu004_set_attach(usbpd_data, mode);
		break;
	case TYPE_C_ATTACH_DRP: /* DRP */
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
		data |= S2MU004_REG_PLUG_CTRL_DRP;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);
		break;
	};
skip:
	mutex_unlock(&usbpd_data->lpm_mutex);
}

void role_swap_check(struct work_struct *wk)
{
	struct delayed_work *delay_work =
		container_of(wk, struct delayed_work, work);
	struct s2mu004_usbpd_data *usbpd_data =
		container_of(delay_work, struct s2mu004_usbpd_data, role_swap_work);
	int mode = 0;

	pr_info("%s: ccic_set_dual_role check again.\n", __func__);
	usbpd_data->try_state_change = 0;

	if (usbpd_data->is_attached == 0) { /* modify here using pd_state */
		pr_err("%s: ccic_set_dual_role reverse failed, set mode to DRP\n", __func__);
		disable_irq(usbpd_data->irq);
		/* exit from Disabled state and set mode to DRP */
		mode =  TYPE_C_ATTACH_DRP;
		s2mu004_rprd_mode_change(usbpd_data, mode);
		enable_irq(usbpd_data->irq);
	}

}

static int ccic_set_dual_role(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct s2mu004_usbpd_data *usbpd_data = dual_role_get_drvdata(dual_role);
	struct i2c_client *i2c;

	USB_STATUS attached_state;
	int mode;
	int timeout = 0;
	int ret = 0;

	if (!usbpd_data) {
		pr_err("%s : usbpd_data is null \n", __func__);
		return -EINVAL;
	}

	i2c = usbpd_data->i2c;

	/* Get Current Role */
	attached_state = usbpd_data->data_role_dual;
	pr_info("%s : request prop = %d , attached_state = %d\n", __func__, prop, attached_state);

	if (attached_state != USB_STATUS_NOTIFY_ATTACH_DFP
	    && attached_state != USB_STATUS_NOTIFY_ATTACH_UFP) {
		pr_err("%s : current mode : %d - just return \n", __func__, attached_state);
		return 0;
	}

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP
	    && *val == DUAL_ROLE_PROP_MODE_DFP) {
		pr_err("%s : current mode : %d - request mode : %d just return \n",
			__func__, attached_state, *val);
		return 0;
	}

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_UFP
	    && *val == DUAL_ROLE_PROP_MODE_UFP) {
		pr_err("%s : current mode : %d - request mode : %d just return \n",
			__func__, attached_state, *val);
		return 0;
	}

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
		/* Current mode DFP and Source  */
		pr_info("%s: try reversing, from Source to Sink\n", __func__);
		/* turns off VBUS first */
		vbus_turn_on_ctrl(usbpd_data, 0);
		muic_disable_otg_detect();
#if defined(CONFIG_CCIC_NOTIFIER)
		/* muic */
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 0/*rprd*/);
#endif
		/* exit from Disabled state and set mode to UFP */
		mode =  TYPE_C_ATTACH_UFP;
		usbpd_data->try_state_change = TYPE_C_ATTACH_UFP;
		s2mu004_rprd_mode_change(usbpd_data, mode);
	} else {
		/* Current mode UFP and Sink  */
		pr_info("%s: try reversing, from Sink to Source\n", __func__);
		/* exit from Disabled state and set mode to UFP */
		mode =  TYPE_C_ATTACH_DFP;
		usbpd_data->try_state_change = TYPE_C_ATTACH_DFP;
		s2mu004_rprd_mode_change(usbpd_data, mode);
	}

	reinit_completion(&usbpd_data->reverse_completion);
	timeout =
	    wait_for_completion_timeout(&usbpd_data->reverse_completion,
					msecs_to_jiffies
					(DUAL_ROLE_SET_MODE_WAIT_MS));

	if (!timeout) {
		usbpd_data->try_state_change = 0;
		pr_err("%s: reverse failed, set mode to DRP\n", __func__);
		disable_irq(usbpd_data->irq);
		/* exit from Disabled state and set mode to DRP */
		mode =  TYPE_C_ATTACH_DRP;
		s2mu004_rprd_mode_change(usbpd_data, mode);
		enable_irq(usbpd_data->irq);
		ret = -EIO;
	} else {
		pr_err("%s: reverse success, one more check\n", __func__);
		schedule_delayed_work(&usbpd_data->role_swap_work, msecs_to_jiffies(DUAL_ROLE_SET_MODE_WAIT_MS));
	}

	dev_info(&i2c->dev, "%s -> data role : %d\n", __func__, *val);
	return ret;
}

/* Decides whether userspace can change a specific property */
int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

/* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
int dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop,
				    unsigned int *val)
{
	struct s2mu004_usbpd_data *usbpd_data = dual_role_get_drvdata(dual_role);

	USB_STATUS attached_state;
	int power_role_dual;

	if (!usbpd_data) {
		pr_err("%s : usbpd_data is null : request prop = %d \n", __func__, prop);
		return -EINVAL;
	}
	attached_state = usbpd_data->data_role_dual;
	power_role_dual = usbpd_data->power_role_dual;

	pr_info("%s : request prop = %d , attached_state = %d, power_role_dual = %d\n",
		__func__, prop, attached_state, power_role_dual);

	if (attached_state == USB_STATUS_NOTIFY_ATTACH_DFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = power_role_dual;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (attached_state == USB_STATUS_NOTIFY_ATTACH_UFP) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = power_role_dual;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

/* Callback for "echo <value> >
 *                      /sys/class/dual_role_usb/<name>/<property>"
 * Block until the entire final state is reached.
 * Blocking is one of the better ways to signal when the operation
 * is done.
 * This function tries to switch to Attached.SRC or Attached.SNK
 * by forcing the mode into SRC or SNK.
 * On failure, we fall back to Try.SNK state machine.
 */
int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop,
			      const unsigned int *val)
{
	pr_info("%s : request prop = %d , *val = %d \n", __func__, prop, *val);
	if (prop == DUAL_ROLE_PROP_MODE)
		return ccic_set_dual_role(dual_role, prop, val);
	else
		return -EINVAL;
}
#endif

#if defined(CONFIG_CCIC_NOTIFIER)
extern struct device *ccic_device;

static void ccic_event_notifier(struct work_struct *data)
{
	struct ccic_state_work *event_work =
		container_of(data, struct ccic_state_work, ccic_work);
	CC_NOTI_TYPEDEF ccic_noti;

	switch (event_work->dest) {
	case CCIC_NOTIFY_DEV_USB:
		pr_info("usb:%s, dest=%s, id=%s, attach=%s, drp=%s, event_work=%p\n", __func__,
				CCIC_NOTI_DEST_Print[event_work->dest],
				CCIC_NOTI_ID_Print[event_work->id],
				event_work->attach ? "Attached" : "Detached",
				CCIC_NOTI_USB_STATUS_Print[event_work->event],
				event_work);
		break;
	default:
		pr_info("usb:%s, dest=%s, id=%s, attach=%d, event=%d, event_work=%p\n", __func__,
			CCIC_NOTI_DEST_Print[event_work->dest],
			CCIC_NOTI_ID_Print[event_work->id],
			event_work->attach,
			event_work->event,
			event_work);
		break;
	}

	ccic_noti.src = CCIC_NOTIFY_DEV_CCIC;
	ccic_noti.dest = event_work->dest;
	ccic_noti.id = event_work->id;
	ccic_noti.sub1 = event_work->attach;
	ccic_noti.sub2 = event_work->event;
	ccic_noti.sub3 = 0;
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
	ccic_noti.pd = &pd_noti;
#endif
	ccic_notifier_notify((CC_NOTI_TYPEDEF *)&ccic_noti, NULL, 0);

	kfree(event_work);
}

void ccic_event_work(void *data, int dest, int id, int attach, int event)
{
	struct s2mu004_usbpd_data *usbpd_data = data;
	struct ccic_state_work *event_work;


	event_work = kmalloc(sizeof(struct ccic_state_work), GFP_ATOMIC);
	pr_info("usb: %s,event_work(%p)\n", __func__, event_work);
	INIT_WORK(&event_work->ccic_work, ccic_event_notifier);

	event_work->dest = dest;
	event_work->id = id;
	event_work->attach = attach;
	event_work->event = event;

#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	if (id == CCIC_NOTIFY_ID_USB) {
		pr_info("usb: %s, dest=%d, event=%d, usbpd_data->data_role_dual=%d, usbpd_data->try_state_change=%d\n",
			__func__, dest, event, usbpd_data->data_role_dual, usbpd_data->try_state_change);

		usbpd_data->data_role_dual = event;

		if (usbpd_data->dual_role != NULL)
			dual_role_instance_changed(usbpd_data->dual_role);

		if (usbpd_data->try_state_change &&
			(usbpd_data->data_role_dual != USB_STATUS_NOTIFY_DETACH)) {
			/* Role change try and new mode detected */
			pr_info("usb: %s, reverse_completion\n", __func__);
			complete(&usbpd_data->reverse_completion);
		}
	}
#endif

	if (queue_work(usbpd_data->ccic_wq, &event_work->ccic_work) == 0) {
		pr_info("usb: %s, event_work(%p) is dropped\n", __func__, event_work);
		kfree(event_work);
	}
}

static void process_dr_swap(struct s2mu004_usbpd_data *usbpd_data)
{
	struct i2c_client *i2c = usbpd_data->i2c;
	dev_info(&i2c->dev, "%s : before - is_host : %d, is_client : %d\n",
		__func__, usbpd_data->is_host, usbpd_data->is_client);
	if (usbpd_data->is_host == HOST_ON) {
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
		msleep(300);
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				1/*attach*/, USB_STATUS_NOTIFY_ATTACH_UFP/*drp*/);
		usbpd_data->is_host = HOST_OFF;
		usbpd_data->is_client = CLIENT_ON;
	} else if (usbpd_data->is_client == CLIENT_ON) {
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
		msleep(300);
		ccic_event_work(usbpd_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB,
				1/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);
		usbpd_data->is_host = HOST_ON;
		usbpd_data->is_client = CLIENT_OFF;
	}
	dev_info(&i2c->dev, "%s : after - is_host : %d, is_client : %d\n",
		__func__, usbpd_data->is_host, usbpd_data->is_client);
}
#endif

int s2mu004_usbpd_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	int ret;
	struct device *dev = &i2c->dev;
	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		if (o_notify)
			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
		return ret;
	}
	ret &= 0xff;
	*dest = ret;
	return 0;
}

int s2mu004_usbpd_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	int ret;
	struct device *dev = &i2c->dev;
	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		if (o_notify)
			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
		return ret;
	}
	return 0;
}
void s2mu004_usbpd_test_read(void)
{
/*
	u8 intr[7];

	s2mu004_usbpd_bulk_read(test_i2c, S2MU004_REG_INT_STATUS0,
			S2MU004_MAX_NUM_INT_STATUS, intr);

	pr_info("%s status[0x%x 0x%x 0x%x 0x%x 0x%x]\n",
			__func__, intr[0], intr[1], intr[2], intr[3], intr[4]);
*/
	u8 data;
	s2mu004_usbpd_read_reg(test_i2c, 0xe2, &data);
	pr_info("%s 0xE2 = %x\n\n\n", __func__, data);
}

EXPORT_SYMBOL_GPL(s2mu004_usbpd_test_read);

int s2mu004_usbpd_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	int ret;
	struct device *dev = &i2c->dev;
	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		if (o_notify)
			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
	}
	return ret;
}

int s2mu004_usbpd_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	int ret;
	struct device *dev = &i2c->dev;
	struct otg_notify *o_notify = get_otg_notify();

	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	if (ret < 0) {
		dev_err(dev, "%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		if (o_notify)
			o_notify->hw_param[USB_CCIC_I2C_ERROR_COUNT]++;
		return ret;
	}
	return 0;
}

int s2mu004_write_msg_header(struct i2c_client *i2c, u8 *buf)
{
	int ret;

	ret = s2mu004_usbpd_bulk_write(i2c, S2MU004_REG_MSG_HEADER_L, 2, buf);

	return ret;
}

int s2mu004_write_msg_obj(struct i2c_client *i2c, int count, data_obj_type *obj)
{
	int ret = 0;
	int i = 0;
	struct device *dev = &i2c->dev;

	if (count > S2MU004_MAX_NUM_MSG_OBJ)
		dev_err(dev, "%s, not invalid obj count number\n", __func__);
	else
		for (i = 0; i < count; i++) {
			ret = s2mu004_usbpd_bulk_write(i2c,
				S2MU004_REG_MSG_OBJECT0_0_L + (4 * i),
							4, obj[i].byte);
		}

	return ret;
}

int s2mu004_send_msg(struct i2c_client *i2c)
{
	int ret;
	u8 reg = S2MU004_REG_MSG_SEND_CON;
	u8 val = S2MU004_REG_MSG_SEND_CON_OP_MODE
			| S2MU004_REG_MSG_SEND_CON_SEND_MSG_EN;

	s2mu004_usbpd_write_reg(i2c, reg, val);

	ret = s2mu004_usbpd_write_reg(i2c, reg, S2MU004_REG_MSG_SEND_CON_OP_MODE);

	return ret;
}

int s2mu004_read_msg_header(struct i2c_client *i2c, msg_header_type *header)
{
	int ret;

	ret = s2mu004_usbpd_bulk_read(i2c, S2MU004_REG_MSG_HEADER_L, 2, header->byte);

	return ret;
}

int s2mu004_read_msg_obj(struct i2c_client *i2c, int count, data_obj_type *obj)
{
	int ret = 0;
	int i = 0;
	struct device *dev = &i2c->dev;

	if (count > S2MU004_MAX_NUM_MSG_OBJ) {
		dev_err(dev, "%s, not invalid obj count number\n", __func__);
		ret = -EINVAL; /*TODO: check fail case */
	} else {
		for (i = 0; i < count; i++) {
			ret = s2mu004_usbpd_bulk_read(i2c,
				S2MU004_REG_MSG_OBJECT0_0_L + (4 * i),
							4, obj[i].byte);
		}
	}

	return ret;
}

static void s2mu004_set_irq_enable(struct s2mu004_usbpd_data *_data,
		u8 int0, u8 int1, u8 int2, u8 int3, u8 int4)
{
	u8 int_mask[S2MU004_MAX_NUM_INT_STATUS]
		= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int ret = 0;
	struct i2c_client *i2c = _data->i2c;
	struct device *dev = &i2c->dev;

	int_mask[0] &= ~int0;
	int_mask[1] &= ~int1;
	int_mask[2] &= ~int2;
	int_mask[3] &= ~int3;
	int_mask[4] &= ~int4;

	ret = i2c_smbus_write_i2c_block_data(i2c, S2MU004_REG_INT_MASK0,
			S2MU004_MAX_NUM_INT_STATUS, int_mask);

	if (ret < 0)
		dev_err(dev, "err write interrupt mask \n");
}

void s2mu004_self_soft_reset(struct i2c_client *i2c)
{
	s2mu004_usbpd_write_reg(i2c, 0xF7, 0x02);/* self soft reset */
	s2mu004_usbpd_write_reg(i2c, 0xF7, 0x00);
}

void s2mu004_driver_reset(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	int i;

	pdic_data->status_reg = 0;
	data->wait_for_msg_arrived = 0;
	pdic_data->header.word = 0;
	for (i = 0; i < S2MU004_MAX_NUM_MSG_OBJ; i++)
		pdic_data->obj[i].object = 0;

	s2mu004_set_irq_enable(pdic_data, ENABLED_INT_0, ENABLED_INT_1,
			ENABLED_INT_2, ENABLED_INT_3, ENABLED_INT_4);
}

static void s2mu004_assert_rd(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 val;
	u8 cc1_val, cc2_val;

#if 1
	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_MON1, &val);

	cc1_val = val & 0x07;
	cc2_val = (val & 0x38) >> 3;

	if (cc1_val == 2) {
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, &val);
		val = (val & 0x8F) | S2MU004_REG_PLUG_CTRL_CC1_MANUAL_ON;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, val);

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &val);
		val = (val & 0x8F) | S2MU004_REG_PLUG_CTRL_RpRd_CC2_VCONN;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, val);
	}

	if (cc2_val == 2) {
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, &val);
		val = (val & 0x8F) | S2MU004_REG_PLUG_CTRL_CC2_MANUAL_ON;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, val);

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &val);
		val = (val & 0x8F) | S2MU004_REG_PLUG_CTRL_RpRd_CC1_VCONN;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, val);
	}
#endif

	/* Ra Setting */
	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &val);
	val = (val & 0x77) | S2MU004_REG_PLUG_CTRL_RpRd_Rd_Sink_Mode;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, val);
}

static void s2mu004_assert_rp(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 val;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &val);

	val = (val & 0x77) | S2MU004_REG_PLUG_CTRL_RpRd_Rp_Source_Mode;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, val);
}

unsigned s2mu004_get_status(/*struct usbpd_data*/ void *_data, unsigned flag)
{
	unsigned ret;
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;

	if (pdic_data->status_reg & flag) {
		ret = pdic_data->status_reg & flag;
		pdic_data->status_reg &= ~flag; /* clear the flag */
		return ret;
	} else {
		return 0;
	}
}

bool s2mu004_poll_status(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct policy_data *policy = &data->policy;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	u8 intr[S2MU004_MAX_NUM_INT_STATUS] = {0};
	int ret = 0;
	u64 status_reg_val = 0;
	u8 reg_val = 0;

	ret = s2mu004_usbpd_bulk_read(i2c, S2MU004_REG_INT_STATUS0,
			S2MU004_MAX_NUM_INT_STATUS, intr);

	dev_info(dev, "%s status[0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\n",
			__func__, intr[0], intr[1], intr[2], intr[3], intr[4], intr[5], intr[6]);

	if ((intr[0] | intr[1] | intr[2] | intr[3] | intr[4]) == 0) {
		status_reg_val |= MSG_NONE;
		goto out;
	}
#if 0
	if (intr[2] & S2MU004_REG_INT_STATUS2_WAKEUP) {
		if (pdic_data->lpm_mode) {
			status_reg_val |= PLUG_WAKEUP;
			goto out;
		} else {
			if ((intr[0] | intr[1] | intr[3] | intr[4]) == 0) {
				status_reg_val |= MSG_NONE;
				goto out;
			}
		}
	}
#endif
	/* when occur detach & attach atomic */
	if (intr[4] & S2MU004_REG_INT_STATUS4_USB_DETACH) {
		status_reg_val |= PLUG_DETACH;
	}

	mutex_lock(&pdic_data->lpm_mutex);
	if ((intr[4] & S2MU004_REG_INT_STATUS4_PLUG_IRQ) &&
			!pdic_data->lpm_mode && !pdic_data->is_water_detect)
		status_reg_val |= PLUG_ATTACH;
	mutex_unlock(&pdic_data->lpm_mutex);

	/* if s2mu004 pdic receive hard reset, it enter default LPM MODE.
		So should be force to NORMAL MODE */
	if (intr[4] & S2MU004_REG_INT_STATUS4_CC12_DET_IRQ) {
		mutex_lock(&pdic_data->lpm_mutex);
		if (!pdic_data->lpm_mode) {
			s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &reg_val);
			if (reg_val & S2MU004_REG_LPM_EN) {
				dev_info(pdic_data->dev, "%s : restore maybe hardreset state \n", __func__);
				usbpd_manager_plug_detach(dev, 0);
				pdic_data->status_reg = 0;
				usbpd_reinit(dev);
#if 0
				s2mu004_dfp(i2c);
				s2mu004_src(i2c);
#endif
				s2mu004_usbpd_reg_init(pdic_data);
#if 0
				if (pdic_data->data_role_dual)
					s2mu004_set_attach(pdic_data, pdic_data->data_role_dual);
				else
					s2mu004_set_normal_mode(pdic_data);
#endif
				msleep(300);

				s2mu004_set_normal_mode(pdic_data);
				mutex_unlock(&pdic_data->lpm_mutex);
				goto out;
			}
		}
		mutex_unlock(&pdic_data->lpm_mutex);
		status_reg_val |= CC_DETECT;
	}
#if 0 /* disable function that support dp control */
	if (pdic_data->check_msg_pass) {
		if (intr[4] & S2MU004_REG_INT_STATUS4_MSG_PASS)
			status_reg_val |= MSG_PASS;
	}
#endif
/* #if defined(CONFIG_CCIC_FACTORY) */
	if ((intr[4] & S2MU004_REG_INT_STATUS4_MSG_PASS) &&
		(intr[3] & S2MU004_REG_INT_STATUS3_UNS_CMD_DATA)) {
		status_reg_val |= MSG_RID;
	}
/* #endif */

	if (intr[0] & S2MU004_REG_INT_STATUS0_MSG_GOODCRC
			|| intr[4] & S2MU004_REG_INT_STATUS4_MSG_SENT)
		status_reg_val |= MSG_GOODCRC;

	if (intr[0] & S2MU004_REG_INT_STATUS0_MSG_ACCEPT)
		status_reg_val |= MSG_ACCEPT;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_PSRDY)
		status_reg_val |= MSG_PSRDY;

	if (intr[2] & S2MU004_REG_INT_STATUS2_MSG_REQUEST)
		status_reg_val |= MSG_REQUEST;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_REJECT)
		status_reg_val |= MSG_REJECT;

	if (intr[2] & S2MU004_REG_INT_STATUS2_MSG_WAIT)
		status_reg_val |= MSG_WAIT;

	if (intr[4] & S2MU004_REG_INT_STATUS4_MSG_ERROR)
		status_reg_val |= MSG_ERROR;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_PING)
		status_reg_val |= MSG_PING;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_GETSNKCAP)
		status_reg_val |= MSG_GET_SNK_CAP;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_GETSRCCAP)
		status_reg_val |= MSG_GET_SRC_CAP;

	if (intr[2] & S2MU004_REG_INT_STATUS2_MSG_SRC_CAP) {
		if (!policy->plug_valid)
			pdic_data->status_reg |= PLUG_ATTACH;
		status_reg_val |= MSG_SRC_CAP;
	}

	if (intr[2] & S2MU004_REG_INT_STATUS2_MSG_SNK_CAP)
		status_reg_val |= MSG_SNK_CAP;

	if (intr[2] & S2MU004_REG_INT_STATUS2_MSG_SOFTRESET)
		status_reg_val |= MSG_SOFTRESET;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_PR_SWAP)
		status_reg_val |= MSG_PR_SWAP;

	if (intr[2] & S2MU004_REG_INT_STATUS2_MSG_VCONN_SWAP)
		status_reg_val |= MSG_VCONN_SWAP;

	if (intr[1] & S2MU004_REG_INT_STATUS1_MSG_DR_SWAP)
		status_reg_val |= MSG_DR_SWAP;

	if (intr[0] & S2MU004_REG_INT_STATUS0_VDM_DISCOVER_ID)
		status_reg_val |= VDM_DISCOVER_IDENTITY;

	if (intr[0] & S2MU004_REG_INT_STATUS0_VDM_DISCOVER_SVID)
		status_reg_val |= VDM_DISCOVER_SVID;

	if (intr[0] & S2MU004_REG_INT_STATUS0_VDM_DISCOVER_MODE)
		status_reg_val |= VDM_DISCOVER_MODE;

	if (intr[0] & S2MU004_REG_INT_STATUS0_VDM_ENTER)
		status_reg_val |= VDM_ENTER_MODE;

	if (intr[0] & S2MU004_REG_INT_STATUS0_VDM_EXIT)
		status_reg_val |= VDM_EXIT_MODE;

	if (intr[0] & S2MU004_REG_INT_STATUS0_VDM_ATTENTION)
		status_reg_val |= VDM_ATTENTION;

#if 0 /* disable function that support dp control */
	/* read message if data object message */
	if (status_reg_val &
			(MSG_REQUEST | MSG_SNK_CAP | MSG_SRC_CAP
			| VDM_DISCOVER_IDENTITY | VDM_DISCOVER_SVID
			| VDM_DISCOVER_MODE | VDM_ENTER_MODE | VDM_EXIT_MODE
			| VDM_ATTENTION | MSG_SOFTRESET | MSG_PASS)) {
		usbpd_protocol_rx(data);
		if (status_reg_val == MSG_PASS && (pdic_data->data_role == USBPD_UFP))
			s2mu004_usbpd_check_vdm_msg(data, &status_reg_val);
	}
#endif
	/* read message if data object message */
	if (status_reg_val &
			(MSG_REQUEST | MSG_SNK_CAP
			| VDM_DISCOVER_IDENTITY | VDM_DISCOVER_SVID
			| VDM_DISCOVER_MODE | VDM_ENTER_MODE | VDM_EXIT_MODE
			| VDM_ATTENTION | MSG_SOFTRESET)) {
		usbpd_protocol_rx(data);
	}
out:
	pdic_data->status_reg |= status_reg_val;

	if (pdic_data->status_reg & data->wait_for_msg_arrived) {
		data->wait_for_msg_arrived = 0;
		complete(&data->msg_arrived);
	}

	return 0;
}

int s2mu004_hard_reset(/*struct usbpd_data */ void *_data)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	int ret;
	u8 reg;

#if defined(CONFIG_SEC_FACTORY)
	if (pdic_data->rid != REG_RID_UNDF && pdic_data->rid != REG_RID_MAX)
		return 0;
#endif
	reg = S2MU004_REG_MSG_SEND_CON;

	ret = s2mu004_usbpd_write_reg(i2c, reg, S2MU004_REG_MSG_SEND_CON_SOP_HardRST
			| S2MU004_REG_MSG_SEND_CON_OP_MODE);
	if (ret < 0)
		goto fail;
	udelay(5);
	ret = s2mu004_usbpd_write_reg(i2c, reg, S2MU004_REG_MSG_SEND_CON_SOP_HardRST
			| S2MU004_REG_MSG_SEND_CON_OP_MODE
			| S2MU004_REG_MSG_SEND_CON_SEND_MSG_EN);
	if (ret < 0)
		goto fail;
	udelay(1);
	ret = s2mu004_usbpd_write_reg(i2c, reg, S2MU004_REG_MSG_SEND_CON_OP_MODE);
	if (ret < 0)
		goto fail;
#if 0
	s2mu004_self_soft_reset(i2c);
#endif
	pdic_data->status_reg = 0;

	return 0;

fail:
	return -EIO;
}

static int s2mu004_receive_message(void *data)
{
	struct s2mu004_usbpd_data *pdic_data = data;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	int obj_num = 0;
	int ret = 0;

	ret = s2mu004_read_msg_header(i2c, &pdic_data->header);
	if (ret < 0)
		dev_err(dev, "%s read msg header error\n", __func__);

	obj_num = pdic_data->header.num_data_objs;

	if (obj_num > 0) {
		ret = s2mu004_read_msg_obj(i2c,
			obj_num, &pdic_data->obj[0]);
	}

	return ret;
}

int s2mu004_tx_msg(/*struct usbpd_data*/ void *_data,
		msg_header_type *header, data_obj_type *obj)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	int ret = 0;
	int count = 0;
	u8 reg_data = 0;
	u8 msg_id = 0;

	mutex_lock(&pdic_data->_mutex);

	/* if there is no attach, skip tx msg */
	if (pdic_data->detach_valid)
		goto done;

	/* using msg id counter at S2MU004 */
	s2mu004_usbpd_read_reg(pdic_data->i2c, 0xf8, &reg_data);
	msg_id = reg_data & 0x7;
	header->msg_id = msg_id;

	ret = s2mu004_write_msg_header(i2c, header->byte);
	if (ret < 0)
		goto done;

	count = header->num_data_objs;

	if (count > 0) {
		ret = s2mu004_write_msg_obj(i2c, count, obj);
		if (ret < 0)
			goto done;
	}

	s2mu004_send_msg(i2c);

	pdic_data->status_reg = 0;
	data->wait_for_msg_arrived = 0;

done:
	mutex_unlock(&pdic_data->_mutex);
	return ret;
}

int s2mu004_rx_msg(/*struct usbpd_data */ void *_data,
		msg_header_type *header, data_obj_type *obj)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	int i;
	int count = 0;

	if (!s2mu004_receive_message(pdic_data)) {
		header->word = pdic_data->header.word;
		count = pdic_data->header.num_data_objs;
		if (count > 0) {
			for (i = 0; i < count; i++)
				obj[i].object = pdic_data->obj[i].object;
		}
		pdic_data->header.word = 0; /* To clear for duplicated call */
		return 0;
	} else {
		return -EINVAL;
	}
}

int s2mu004_set_vconn_source(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 reg_data = 0, reg_val = 0, cc1_val = 0, cc2_val = 0;

	if (!pdic_data->vconn_en) {
		pr_err("%s, not support vconn source\n", __func__);
		return -1;
	}

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_MON1, &reg_val);
	cc1_val = reg_val & 0x07;
	cc2_val = (reg_val & 0x38) >> 3;

	if (val == USBPD_VCONN_ON) {
		if (cc1_val == USBPD_Rd) {
			if (cc2_val == USBPD_Ra) {
				s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &reg_data);
				reg_data &= ~(0x07 << 4);
				reg_data |= S2MU004_REG_PLUG_CTRL_RpRd_CC2_VCONN;
				s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, reg_data);
			}
		}
		if (cc2_val == USBPD_Rd) {
			if (cc1_val == USBPD_Ra) {
				s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &reg_data);
				reg_data &= ~(0x07 << 4);
				reg_data |= S2MU004_REG_PLUG_CTRL_RpRd_CC1_VCONN;
				s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, reg_data);
			}
		}
	} else if (val == USBPD_VCONN_OFF) {
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &reg_data);
		reg_data &= ~(0x07 << 4);
		reg_data |= 0x01 << 4;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, reg_data);
	} else
		return(-1);

	pdic_data->vconn_source = val;
	return 0;
}

int s2mu004_get_vconn_source(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;

	/* TODO
		set s2mu004 pdic register control */

	if (pdic_data->vconn_source != *val) {
		dev_info(pdic_data->dev, "%s, vconn_source(%d) != gpio val(%d)\n",
				__func__, pdic_data->vconn_source, *val);
		pdic_data->vconn_source = *val;
	}

	return 0;
}

/* val : sink(0) or source(1) */
int s2mu004_set_power_role(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;

	if (val == USBPD_SINK) {
#if 1
		s2mu004_assert_rd(data);
#endif
		s2mu004_snk(pdic_data->i2c);
	} else if (val == USBPD_SOURCE) {
		s2mu004_assert_rp(data);
		s2mu004_src(pdic_data->i2c);
	} else
		return(-1);

	pdic_data->power_role = val;
	return 0;
}

int s2mu004_get_power_role(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	*val = pdic_data->power_role;
	return 0;
}

int s2mu004_set_data_role(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 val_port, data_role;

	/* DATA_ROLE (0x18[2])
	 * 0 : UFP
	 * 1 : DFP
	 */
	if (val == USBPD_UFP) {
		data_role = 0;
		s2mu004_ufp(i2c);
	} else {/* (val == USBPD_DFP) */
		data_role = S2MU004_REG_MSG_DATA_ROLE_MASK;
		s2mu004_dfp(i2c);
	}

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, &val_port);
	val_port = (val_port & ~S2MU004_REG_MSG_DATA_ROLE_MASK) | data_role;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, val_port);

	pdic_data->data_role = val;

#if defined(CONFIG_CCIC_NOTIFIER)
	process_dr_swap(pdic_data);
#endif
	return 0;
}

int s2mu004_get_data_role(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;
	*val = pdic_data->data_role;
	return 0;
}

int s2mu004_set_check_msg_pass(/*struct usbpd_data */ void *_data, int val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	struct s2mu004_usbpd_data *pdic_data = data->phy_driver_data;

	dev_info(pdic_data->dev, "%s: check_msg_pass val(%d)\n", __func__, val);

	pdic_data->check_msg_pass = val;

	if (val) {
		s2mu004_set_irq_enable(pdic_data,
			ENABLED_INT_0, ENABLED_INT_1, ENABLED_INT_2,
			ENABLED_INT_3, ENABLED_INT_4_PASS);
	} else {
		s2mu004_set_irq_enable(pdic_data,
			ENABLED_INT_0, ENABLED_INT_1, ENABLED_INT_2,
			ENABLED_INT_3, ENABLED_INT_4);
	}

	return 0;
}

#ifndef CONFIG_SEC_FACTORY
static void s2mu004_usbpd_set_rp_scr_sel(struct s2mu004_usbpd_data *pdic_data, CCIC_RP_SCR_SEL scr_sel)
{
	struct i2c_client *i2c = pdic_data->i2c;
	u8 data = 0;
	pr_info("%s: scr_sel : (%d)\n", __func__, scr_sel);
	switch (scr_sel) {
	case PLUG_CTRL_RP80:
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
		data &= ~(0x3 << 4);
		data |= S2MU004_REG_PLUG_CTRL_RP80;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);
		/* set RdRa threshold 0.2V */
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_SET_RD, 0x04);
		/* set RpRd threshold 1.6V */
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_SET_RP, 0x2A);
		break;
	case PLUG_CTRL_RP180:
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
		data &= ~(0x3 << 4);
		data |= S2MU004_REG_PLUG_CTRL_RP180;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);
		/* set RdRa threshold 0.4V */
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_SET_RD, 0x09);
		/* set RpRd threshold 2.05V */
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_SET_RP, 0x34);
		break;
	default:
		break;
	}
	return;
}
#endif

#if 0 /* disable function that support dp control */
int s2mu004_usbpd_check_vdm_msg(/*struct usbpd_data */ void *_data, int *val)
{
	struct usbpd_data *data = (struct usbpd_data *) _data;
	int vdm_command = 0;

	if (data->protocol_rx.msg_header.num_data_objs == 0)
		return 0;

	if (data->protocol_rx.msg_header.msg_type != USBPD_Vendor_Defined)
		return 0;

	vdm_command = data->protocol_rx.data_obj[0].structured_vdm.command;

	switch (vdm_command) {
	case DisplayPort_Status_Update:
		*val |= VDM_DP_STATUS_UPDATE;
		break;
	case DisplayPort_Configure:
		*val |= VDM_DP_CONFIGURE;
		break;
	default:
		return 0;
	}

	dev_info(data->dev, "%s: check vdm mag val(%d)\n", __func__, vdm_command);

	return 0;
}
#endif

#if 1
static void s2mu004_dfp(struct i2c_client *i2c)
{
	u8 data;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, &data);
	data |= S2MU004_REG_MSG_DATA_ROLE_MASK;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, data);
}
#endif
#if 1
static void s2mu004_ufp(struct i2c_client *i2c)
{
	u8 data;
	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, &data);
	data &= ~S2MU004_REG_MSG_DATA_ROLE_MASK;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, data);
}
#endif
#if 1
static void s2mu004_src(struct i2c_client *i2c)
{
	u8 data;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, &data);
	data = (data & 0xF7) | (0x1 << 6);
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, data);
}
#endif
static void s2mu004_snk(struct i2c_client *i2c)
{
	u8 data;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, &data);
	data &= ~(0x01 << 6);
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_MSG, data);
}

#if defined(CONFIG_CCIC_NOTIFIER)
void s2mu004_control_option_command (struct s2mu004_usbpd_data *pdic_data, int cmd) {
	struct usbpd_data *_data = dev_get_drvdata(pdic_data->dev);
	int pd_cmd = cmd & 0x0f;

/* 0x1 : Vconn control option command ON
 * 0x2 : Vconn control option command OFF
 * 0x3 : Water Detect option command ON
 * 0x4 : Water Detect option command OFF
 */
	switch (pd_cmd) {
	case 1:
		s2mu004_set_vconn_source(_data, USBPD_VCONN_ON);
		break;
	case 2:
		s2mu004_set_vconn_source(_data, USBPD_VCONN_OFF);
		break;
	case 3:
	case 4:
		pr_err("%s : not implement water control\n", __func__);
		break;
	default:
		break;
	}
}
#endif

static void s2mu004_notify_pdic_rid(struct s2mu004_usbpd_data *pdic_data, int rid)
{
#if defined(CONFIG_CCIC_NOTIFIER)
	pdic_data->is_factory_mode = false;
	if (rid == RID_523K)
		pdic_data->is_factory_mode = true;
	/* rid */
	ccic_event_work(pdic_data,
		CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_RID, rid/*rid*/, 0);

	if (rid == REG_RID_523K || rid == REG_RID_619K || rid == REG_RID_OPEN)
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH);
#else
	muic_attached_dev_t new_dev;
	pdic_data->is_factory_mode = false;
	switch (rid) {
	case REG_RID_255K:
		new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		break;
	case REG_RID_301K:
		new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		break;
	case REG_RID_523K:
		new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		pdic_data->is_factory_mode = true;
		break;
	case REG_RID_619K:
		new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		break;
	default:
		new_dev = ATTACHED_DEV_NONE_MUIC;
		return;
	}
	s2mu004_pdic_notifier_attach_attached_jig_dev(new_dev);
#endif
	dev_info(pdic_data->dev, "%s : attached rid state(%d)", __func__, rid);
}

/* #if defined(CONFIG_CCIC_FACTORY) */
static void s2mu004_usbpd_check_rid(struct s2mu004_usbpd_data *pdic_data)
{
	struct i2c_client *i2c = pdic_data->i2c;
	u8 rid;
	int prev_rid = pdic_data->rid;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_ADC_STATUS, &rid);
	rid &= S2MU004_PDIC_RID_MASK;
	rid >>= S2MU004_PDIC_RID_SHIFT;

	dev_info(pdic_data->dev, "%s : attached rid state(%d)", __func__, rid);

	if (rid) {
		if (pdic_data->rid != rid) {
			pdic_data->rid = rid;
			if (prev_rid >= REG_RID_OPEN && rid >= REG_RID_OPEN)
				dev_err(pdic_data->dev, "%s : rid is not changed, skip notify(%d)", __func__, rid);
			else
				s2mu004_notify_pdic_rid(pdic_data, rid);
		}

		if (rid >= REG_RID_MAX) {
			dev_err(pdic_data->dev, "%s : overflow rid value", __func__);
			return;
		}
	}
}

static int s2mu004_set_attach(struct s2mu004_usbpd_data *pdic_data, u8 mode)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
	data &= S2MU004_REG_PLUG_CTRL_RESET;
	data |= mode | S2MU004_REG_PLUG_CTRL_RP180;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &data);
	data &= ~S2MU004_REG_LPM_EN;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL, data);

	dev_info(dev, "%s s2mu004 force to attach\n", __func__);

	return ret;
}

static int s2mu004_set_detach(struct s2mu004_usbpd_data *pdic_data, u8 mode)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	if (mode == TYPE_C_ATTACH_DFP) {
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &data);
		data |= 0x1 << 7 | 0x1 << 3;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, data);
	} else if (mode == TYPE_C_ATTACH_UFP) {
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &data);
		data |= 0x1 << 7;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, data);
	}
	msleep(50);

	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, 0x00);
	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
	data &= S2MU004_REG_PLUG_CTRL_RESET;
	data |= S2MU004_REG_PLUG_CTRL_DFP | S2MU004_REG_PLUG_CTRL_RP0;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &data);
	data |= S2MU004_REG_LPM_EN;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL, data);

	dev_info(dev, "%s s2mu004 force to detach\n", __func__);

	return ret;
}

int s2mu004_set_normal_mode(struct s2mu004_usbpd_data *pdic_data)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
	data &= S2MU004_REG_PLUG_CTRL_RESET;
	data |= S2MU004_REG_PLUG_CTRL_DRP | S2MU004_REG_PLUG_CTRL_RP180;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &data);
	data &= ~S2MU004_REG_LPM_EN;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL, data);

	pdic_data->lpm_mode = false;

	s2mu004_set_irq_enable(pdic_data, 0, 0,
		S2MU004_REG_INT_STATUS2_MSG_SRC_CAP, ENABLED_INT_3,
				ENABLED_INT_4_PASS);

	dev_info(dev, "%s s2mu004 exit lpm mode\n", __func__);

	return ret;
}

int s2mu004_set_lpm_mode(struct s2mu004_usbpd_data *pdic_data)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
#if 0
	u8 intr[7];
#endif
	pdic_data->lpm_mode = true;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &data);
	data &= S2MU004_REG_PLUG_CTRL_RESET;
	data |= S2MU004_REG_PLUG_CTRL_DFP | S2MU004_REG_PLUG_CTRL_RP0;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, data);

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &data);
	data |= S2MU004_REG_LPM_EN;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL, data);
#if 0
	s2mu004_usbpd_bulk_read(i2c, S2MU004_REG_INT_STATUS0,
			S2MU004_MAX_NUM_INT_STATUS, intr);

	dev_info(dev, "%s status[0x%x 0x%x 0x%x 0x%x 0x%x]\n",
			__func__, intr[0], intr[1], intr[2], intr[3], intr[4]);
#endif
	s2mu004_set_irq_enable(pdic_data, 0, 0, 0, 0, 0);

	dev_info(dev, "%s s2mu004 enter lpm mode\n", __func__);

	return ret;
}

static void s2mu004_src_sink_init(struct work_struct *work)
{
	struct s2mu004_usbpd_data *pdic_data =
		container_of(work, struct s2mu004_usbpd_data, plug_work.work);
	struct usbpd_data *pd_data = dev_get_drvdata(pdic_data->dev);
	bool plug_valid = pd_data->policy.plug_valid;
	struct i2c_client *i2c = pdic_data->i2c;
	u8 val = 0;
	u8 data = 0;

	mutex_lock(&pdic_data->_mutex);

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_ADC_STATUS, &data);

	pr_info("%s, plug_valid = %x, rid = %x\n", __func__, plug_valid, data);

	if (!plug_valid) {
		s2mu004_ufp(i2c);
		s2mu004_snk(i2c);

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &val);
		val = val & 0x7F;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, val);

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, &val);
		val = val & 0xEF;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, val);

		if (!pdic_data->lpm_mode)
			s2mu004_set_lpm_mode(pdic_data);
	}

	mutex_unlock(&pdic_data->_mutex);
}

#if defined(CONFIG_MUIC_NOTIFIER)
static int type3_handle_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
#if defined(CONFIG_CCIC_NOTIFIER)
	CC_NOTI_ATTACH_TYPEDEF *p_noti = (CC_NOTI_ATTACH_TYPEDEF *)data;
	muic_attached_dev_t attached_dev = p_noti->cable_type;
#else
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
#endif
	struct s2mu004_usbpd_data *pdic_data =
		container_of(nb, struct s2mu004_usbpd_data,
			     type3_nb);
	struct i2c_client *i2c = pdic_data->i2c;
	u8 reg_data = 0;

	struct otg_notify *o_notify = get_otg_notify();

	mutex_lock(&pdic_data->lpm_mutex);
	pr_info("%s action:%d, attached_dev:%d, lpm:%d, pdic_data->is_otg_vboost:%d, pdic_data->is_otg_reboost:%d\n",
		__func__, (int)action, (int)attached_dev, pdic_data->lpm_mode,
		(int)pdic_data->is_otg_vboost, (int)pdic_data->is_otg_reboost);

	if ((action == MUIC_PDIC_NOTIFY_CMD_ATTACH) &&
		(attached_dev == ATTACHED_DEV_TYPE3_MUIC)) {
		if (pdic_data->lpm_mode) {
			pr_info("%s try to exit lpm mode-->\n", __func__);
			s2mu004_set_normal_mode(pdic_data);
			pr_info("%s after exit lpm mode<--\n", __func__);
		}
#if 0
		cancel_delayed_work_sync(&pdic_data->plug_work);
		queue_delayed_work_on(0, pdic_data->pdic_queue,
			&pdic_data->plug_work, msecs_to_jiffies(3000));
		mutex_unlock(&pdic_data->_mutex);
			&pdic_data->plug_work, msecs_to_jiffies(20000));
#endif
	} else if ((action == MUIC_PDIC_NOTIFY_CMD_ATTACH) &&
		attached_dev == ATTACHED_DEV_WATER_MUIC) {
		pr_info("%s, ATTACH : ATTACHED_DEV_WATER_MUIC(WATER)\n", __func__);
		pdic_data->is_water_detect = true;

		s2mu004_set_lpm_mode(pdic_data);

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &reg_data);
		reg_data &= ~S2MU004_REG_LPM_EN;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL, reg_data);
		if (o_notify)
			o_notify->hw_param[USB_CCIC_WATER_INT_COUNT]++;
	} else if ((action == MUIC_PDIC_NOTIFY_CMD_DETACH) &&
		attached_dev == ATTACHED_DEV_WATER_MUIC) {
		pr_info("%s, DETACH : ATTACHED_DEV_WATER_MUIC(DRY)\n", __func__);
		pdic_data->is_water_detect = false;

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &reg_data);
		reg_data |= S2MU004_REG_LPM_EN;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL, reg_data);
		if (o_notify)
			o_notify->hw_param[USB_CCIC_DRY_INT_COUNT]++;
	} else if (action == MUIC_PDIC_NOTIFY_CMD_DETACH) {
		if (!pdic_data->lpm_mode) {
			pr_info("%s try to exit lpm mode-->\n", __func__);
			s2mu004_set_lpm_mode(pdic_data);
			pr_info("%s after exit lpm mode<--\n", __func__);
		}
	}
#ifndef CONFIG_SEC_FACTORY
	else if ((action == MUIC_PDIC_NOTIFY_CMD_ATTACH)
			&& (attached_dev == ATTACHED_DEV_CHECK_OCP)
			&& pdic_data->is_otg_vboost
			&& pdic_data->data_role_dual == USB_STATUS_NOTIFY_ATTACH_DFP) {
		if (o_notify) {
			if (is_blocked(o_notify, NOTIFY_BLOCK_TYPE_HOST)) {
				pr_info("%s, upsm mode, skip OCP handling\n", __func__);
				goto EOH;
			}
		}
		if (pdic_data->is_otg_reboost) {
			/* todo : over current event to platform */
			pr_info("%s, CHECK_OCP, Can't afford it(OVERCURRENT)\n", __func__);
#if 0
			if (o_notify) {
				send_otg_notify(o_notify, NOTIFY_EVENT_OVERCURRENT, 0);
				o_notify->hw_param[USB_CCIC_OVC_COUNT]++;
			}
#endif
			goto EOH;
		}
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 1/*rprd*/);

		pr_info("%s, CHECK_OCP, start OCP W/A\n", __func__);
		pdic_data->is_otg_reboost = true;
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_CC_HOLD, &reg_data);
		reg_data |= S2MU004_REG_PLUG_CTRL_CC_HOLD_BIT;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_CC_HOLD, reg_data);

		s2mu004_usbpd_set_rp_scr_sel(pdic_data, PLUG_CTRL_RP80);
		vbus_turn_on_ctrl(pdic_data, 0);
		vbus_turn_on_ctrl(pdic_data, 1);

		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_CC_HOLD, &reg_data);
		reg_data &= ~S2MU004_REG_PLUG_CTRL_CC_HOLD_BIT;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_CC_HOLD, reg_data);
	}
EOH:
#endif
	mutex_unlock(&pdic_data->lpm_mutex);

	return 0;
}
#endif

int s2mu004_adc_check(struct s2mu004_usbpd_data *pdic_data)
{
	int adc_val = s2mu004_usbpd_get_adc();
	pr_info("%s, adc_val = %x\n", __func__, adc_val);
#if 0
	if ((adc_val < 0x01) || (adc_val == 0x1f))
		return true;
#else
	if (adc_val < 0x01)
		return true;
#endif
	else
		return false;
}

int s2mu004_check_port_detect(struct s2mu004_usbpd_data *pdic_data)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	struct otg_notify *o_notify = get_otg_notify();

	ret = s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_MON2, &data);
	if (ret < 0)
		dev_err(dev, "%s, i2c read PLUG_MON2 error\n", __func__);

	if ((data & S2MU004_PR_MASK) == S2MU004_PDIC_SINK) {
		pdic_data->power_role = PDIC_SINK;
		pdic_data->data_role = USBPD_UFP;
		s2mu004_snk(i2c);
		s2mu004_ufp(i2c);
		dev_info(dev, "SINK\n");
		if (pdic_data->is_factory_mode == true)
#if defined(CONFIG_CCIC_NOTIFIER)
		{
			/* muic */
			ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 0/*rprd*/);
			return true;
		}
#else
		return true;
#endif
		usbpd_policy_reset(pd_data, PLUG_EVENT);
		cancel_delayed_work_sync(&pdic_data->plug_work);
#if defined(CONFIG_CCIC_NOTIFIER)
		dev_info(&i2c->dev, "%s %d: is_host = %d, is_client = %d\n",
					__func__, __LINE__, pdic_data->is_host, pdic_data->is_client);
		pdic_data->is_attached = 1;

		if (pdic_data->is_host == HOST_ON) {
			dev_info(&i2c->dev, "%s %d: turn off host\n", __func__, __LINE__);
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 1/*rprd*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_NONE;
#endif
			/* add to turn off external 5V */
			vbus_turn_on_ctrl(pdic_data, 0);
			muic_disable_otg_detect();
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			pdic_data->is_host = HOST_OFF;
			msleep(300);
		}

		/* muic */
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 0/*rprd*/);
		if (!(pdic_data->rid == REG_RID_523K || pdic_data->rid == REG_RID_619K)) {
			if (pdic_data->is_client == CLIENT_OFF && pdic_data->is_host == HOST_OFF) {
				/* usb */
				pdic_data->is_client = CLIENT_ON;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
				pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_SNK;
#endif
				ccic_event_work(pdic_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_UFP/*drp*/);
			}
		}
#endif
	} else if ((data & S2MU004_PR_MASK) == S2MU004_PDIC_SOURCE) {
		pdic_data->power_role = PDIC_SOURCE;
		pdic_data->data_role = USBPD_DFP;
		dev_info(dev, "SOURCE\n");
		usbpd_policy_reset(pd_data, PLUG_EVENT);
#if defined(CONFIG_CCIC_NOTIFIER)
		dev_info(&i2c->dev, "%s %d: is_host = %d, is_client = %d\n",
						__func__, __LINE__, pdic_data->is_host, pdic_data->is_client);
		pdic_data->is_attached = 1;

		if (pdic_data->is_client == CLIENT_ON) {
			dev_info(&i2c->dev, "%s %d: turn off client\n", __func__, __LINE__);
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 0/*rprd*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_NONE;
#endif
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
			pdic_data->is_client = CLIENT_OFF;
			/* msleep(300); */
		}

		if (pdic_data->is_host == HOST_OFF) {
			/* muic */
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 1/*attach*/, 1/*rprd*/);
			/* otg */
			pdic_data->is_host = HOST_ON;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_SRC;
#endif
			/* USB */
			ccic_event_work(pdic_data,
					CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 1/*attach*/, USB_STATUS_NOTIFY_ATTACH_DFP/*drp*/);
			/* add to turn on external 5V */
			if (!is_blocked(o_notify, NOTIFY_BLOCK_TYPE_HOST))
				vbus_turn_on_ctrl(pdic_data, 1);
		}
#else
		usbpd_manager_plug_attach(dev, ATTACHED_DEV_TYPE3_ADAPTER_MUIC);
#endif
		cancel_delayed_work_sync(&pdic_data->plug_work);

		s2mu004_set_vconn_source(pd_data, USBPD_VCONN_ON);
#if 1
		s2mu004_dfp(i2c);
		s2mu004_src(i2c);
#endif
		msleep(600); /* dont over 310~620ms(tTypeCSinkWaitCap) */
	} else {
		dev_err(dev, "%s, PLUG Error\n", __func__);
		return -1;
	}

	pdic_data->detach_valid = false;

	/* Detect DFP or UDP , CC1 or CC2 */
	data = 0;
	ret = s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_MON1, &data);
	if (ret < 0) {
		dev_err(dev, "%s, i2c read PLUG_MON1 error\n", __func__);
	} else {
		s2mu004_set_irq_enable(pdic_data,
			ENABLED_INT_0, ENABLED_INT_1, ENABLED_INT_2,
			ENABLED_INT_3, ENABLED_INT_4);
	}

	return ret;
}

int s2mu004_check_init_port(struct s2mu004_usbpd_data *pdic_data)
{
	u8 data;
	int ret = 0;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;

	ret = s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_MON2, &data);
	if (ret < 0)
		dev_err(dev, "%s, i2c read PLUG_MON2 error\n", __func__);

	if ((data & S2MU004_PR_MASK) == S2MU004_PDIC_SOURCE)
		return PDIC_SOURCE;

	return 0;
}

static irqreturn_t s2mu004_irq_thread(int irq, void *data)
{
	struct s2mu004_usbpd_data *pdic_data = data;
	struct i2c_client *i2c = pdic_data->i2c;
	struct device *dev = &i2c->dev;
	struct usbpd_data *pd_data = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg_data;
	unsigned attach_status = 0, rid_status = 0;
#if defined(CONFIG_SEC_FACTORY)
	u8 rid;
#endif /* CONFIG_SEC_FACTORY */

	dev_info(dev, "%s\n", __func__);

	mutex_lock(&pdic_data->_mutex);

	s2mu004_poll_status(pd_data);

	if (s2mu004_get_status(pd_data, MSG_NONE))
		goto out;

	if (s2mu004_get_status(pd_data, MSG_HARDRESET)) {
		usbpd_rx_hard_reset(dev);
		usbpd_kick_policy_work(dev);
		goto out;
	}
#if 0 /* for cc pin water detect */
	if (s2mu004_get_status(pd_data, PLUG_WAKEUP)) {
		if (pdic_data->lpm_mode) {
			/* wait for muic RID update */
			msleep(50);
			if (s2mu004_adc_check(pdic_data)) {
				s2mu004_set_normal_mode(pdic_data);
				cancel_delayed_work_sync(&pdic_data->plug_work);
				queue_delayed_work_on(0, pdic_data->pdic_queue,
					&pdic_data->plug_work, msecs_to_jiffies(20000));
			}
		}
		goto out;
	}
#endif

	if (s2mu004_get_status(pd_data, PLUG_DETACH)) {
#if defined(CONFIG_SEC_FACTORY)
		if (pdic_data->rid == REG_RID_619K) {
			msleep(250);
			s2mu004_usbpd_read_reg(i2c, S2MU004_REG_ADC_STATUS, &rid);
			rid &= S2MU004_PDIC_RID_MASK;
			rid >>= S2MU004_PDIC_RID_SHIFT;
			dev_info(&i2c->dev, "%s %d: Detached, check if still 619K? => 0x%X\n",
					__func__, __LINE__, rid);
			if (rid == REG_RID_619K)
				goto skip_detach;
		}
#endif /* CONFIG_SEC_FACTORY */
#ifndef CONFIG_SEC_FACTORY
		if (pdic_data->is_otg_reboost) {
			dev_info(&i2c->dev, "%s %d: Detached, go back to 180uA\n",
					__func__, __LINE__);
			s2mu004_usbpd_set_rp_scr_sel(pdic_data, PLUG_CTRL_RP180);
			pdic_data->is_otg_reboost = false;
		}
#endif
		attach_status = s2mu004_get_status(pd_data, PLUG_ATTACH);
		rid_status = s2mu004_get_status(pd_data, MSG_RID);
		pdic_data->status_reg = 0;
		usbpd_reinit(dev);
		/* for ccic hw detect */
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_MSG_SEND_CON, 0x00);
		pdic_data->rid = REG_RID_MAX;
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
		pd_noti.sink_status.current_pdo_num = 0;
		pd_noti.sink_status.selected_pdo_num = 0;
#endif
		s2mu004_usbpd_reg_init(pdic_data);
		if (attach_status) {
			ret = s2mu004_check_port_detect(pdic_data);
			if (ret >= 0) {
				if (rid_status) {
					s2mu004_usbpd_check_rid(pdic_data);
				}
				goto hard_reset;
			}
		}

		s2mu004_set_vconn_source(pd_data, USBPD_VCONN_OFF);

		pdic_data->detach_valid = true;
		pdic_data->is_factory_mode = false;
#if 0 /* disable because that controlled by muic */
		cancel_delayed_work_sync(&pdic_data->plug_work);
		queue_delayed_work_on(0, pdic_data->pdic_queue,
			&pdic_data->plug_work, msecs_to_jiffies(500));
#endif
#if defined	(CONFIG_CCIC_NOTIFIER)
		pdic_data->is_attached = 0;

		usbpd_manager_plug_detach(dev, 0);
		/* MUIC */
		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_ATTACH, 0/*attach*/, 0/*rprd*/);

		ccic_event_work(pdic_data,
			CCIC_NOTIFY_DEV_MUIC, CCIC_NOTIFY_ID_RID, REG_RID_OPEN/*rid*/, 0);
		if (pdic_data->is_host > HOST_OFF || pdic_data->is_client > CLIENT_OFF) {
			if (pdic_data->is_host > HOST_OFF) {
				vbus_turn_on_ctrl(pdic_data, 0);
				muic_disable_otg_detect();
			}
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pr_info("%s, data_role (%d)\n", __func__, pdic_data->data_role_dual);
			if (pdic_data->data_role_dual == USB_STATUS_NOTIFY_ATTACH_DFP &&
				!pdic_data->try_state_change) {
				s2mu004_usbpd_read_reg(i2c, 0x27, &reg_data);
				reg_data |= 0x1 << 7;
				s2mu004_usbpd_write_reg(i2c, 0x27, reg_data);

				msleep(200);

				s2mu004_usbpd_read_reg(i2c, 0x27, &reg_data);
				reg_data &= ~(0x1 << 7);
				s2mu004_usbpd_write_reg(i2c, 0x27, reg_data);
			}
#endif
			/* usb or otg */
			dev_info(&i2c->dev, "%s %d: is_host = %d, is_client = %d\n",
					__func__, __LINE__, pdic_data->is_host, pdic_data->is_client);
			pdic_data->is_host = HOST_OFF;
			pdic_data->is_client = CLIENT_OFF;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			pdic_data->power_role_dual = DUAL_ROLE_PROP_PR_NONE;
#endif
			/* USB */
			ccic_event_work(pdic_data,
				CCIC_NOTIFY_DEV_USB, CCIC_NOTIFY_ID_USB, 0/*attach*/, USB_STATUS_NOTIFY_DETACH/*drp*/);
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
			if (!pdic_data->try_state_change)
				s2mu004_rprd_mode_change(pdic_data, TYPE_C_ATTACH_DRP);
#endif
		}
#else
		usbpd_manager_plug_detach(dev, 1);
#endif
		s2mu004_set_irq_enable(data, 0, 0,
			S2MU004_REG_INT_STATUS2_MSG_SRC_CAP, 0,
						ENABLED_INT_4_PASS);
		goto out;
	}
#if defined(CONFIG_SEC_FACTORY)
skip_detach:
#endif /* CONFIG_SEC_FACTORY */
	if (s2mu004_get_status(pd_data, PLUG_ATTACH)) {
		if (s2mu004_check_port_detect(data) < 0)
			goto out;
	}

	if (s2mu004_get_status(pd_data, MSG_RID)) {
		s2mu004_usbpd_check_rid(pdic_data);
	}

hard_reset:
	mutex_lock(&pdic_data->lpm_mutex);
	if (!pdic_data->lpm_mode)
		usbpd_kick_policy_work(dev);
	mutex_unlock(&pdic_data->lpm_mutex);
out:
	mutex_unlock(&pdic_data->_mutex);

	return IRQ_HANDLED;
}
static int s2mu004_usbpd_reg_init(struct s2mu004_usbpd_data *_data)
{
	struct i2c_client *i2c = _data->i2c;
	u8 data = 0;

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL_2, &data);
/*
	if ((data >> 4) <= (0x0f - 2))
		data += (2 << 4);
	else
		data |= 0xf0;
*/
	data |= 0xf0;
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PD_CTRL_2, data);

	/* set Rd threshold to 400mV */
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_SET_RD, 0x09);
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_SET_RP, 0x34);

	if (_data->vconn_en) {
		/* Off Manual Rd setup & On Manual Vconn setup */
		s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, &data);
		data &= ~(0x01 << 7);
		data |= 0x1 << 4;
		s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, data);
	}

	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_RpRd, 0x00);
	s2mu004_usbpd_write_reg(i2c, S2MU004_REG_PLUG_CTRL_CC12, 0x00);

	return 0;
}

static irqreturn_t s2mu004_irq_isr(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

static int s2mu004_usbpd_irq_init(struct s2mu004_usbpd_data *_data)
{
	struct i2c_client *i2c = _data->i2c;
	struct device *dev = &i2c->dev;
	int ret = 0;

	if (!_data->irq_gpio) {
		dev_err(dev, "%s No interrupt specified\n", __func__);
		return -ENXIO;
	}

/*	s2mu004_usbpd_bulk_read(i2c, S2MU004_REG_INT_STATUS0,
			S2MU004_MAX_NUM_INT_STATUS, intr);

	pr_info("%s status[0x%x 0x%x 0x%x 0x%x 0x%x]\n",
			__func__, intr[0], intr[1], intr[2], intr[3], intr[4]);
*/
	i2c->irq = gpio_to_irq(_data->irq_gpio);

	if (i2c->irq) {
		ret = request_threaded_irq(i2c->irq, s2mu004_irq_isr,
				s2mu004_irq_thread,
				(IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND),
				"s2mu004-usbpd", _data);
		if (ret < 0) {
			dev_err(dev, "%s failed to request irq(%d)\n",
					__func__, i2c->irq);
			return ret;
		}

		ret = enable_irq_wake(i2c->irq);
		if (ret < 0)
			dev_err(dev, "%s failed to enable wakeup src\n",
					__func__);
	}

	if (_data->lpm_mode)
		s2mu004_set_irq_enable(_data, 0, 0, 0, 0, 0);
	else
		s2mu004_set_irq_enable(_data, 0, 0, 0, 0,
					ENABLED_INT_4_PASS);

	return ret;
}

static int of_s2mu004_dt(struct device *dev,
			struct s2mu004_usbpd_data *_data)
{
	struct device_node *np_usbpd = dev->of_node;
	int ret = 0;

	if (np_usbpd == NULL) {
		dev_err(dev, "%s np NULL\n", __func__);
		return -EINVAL;
	} else {
		_data->irq_gpio = of_get_named_gpio(np_usbpd,
							"usbpd,usbpd_int", 0);
		if (_data->irq_gpio < 0) {
			dev_err(dev, "error reading usbpd irq = %d\n",
						_data->irq_gpio);
			_data->irq_gpio = 0;
		}
		if (of_find_property(np_usbpd, "vconn-en", NULL))
			_data->vconn_en = true;
		else
			_data->vconn_en = false;
	}

	return ret;
}

static int s2mu004_usbpd_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
	struct s2mu004_usbpd_data *pdic_data;
	struct device *dev = &i2c->dev;
	int ret = 0;
	u8 data = 0, init_port = 0;
	u8 rid = 0;
	bool pdic_port = 0;
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
#endif

	dev_info(dev, "%s\n", __func__);
	test_i2c = i2c;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c functionality check error\n", __func__);
		ret = -EIO;
		goto err_return;
	}

	pdic_data = kzalloc(sizeof(struct s2mu004_usbpd_data), GFP_KERNEL);
	if (!pdic_data) {
		dev_err(dev, "%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	/* save platfom data for gpio control functions */
	pdic_data->dev = &i2c->dev;
	pdic_data->i2c = i2c;
	i2c_set_clientdata(i2c, pdic_data);

	ret = of_s2mu004_dt(&i2c->dev, pdic_data);
	if (ret < 0)
		dev_err(dev, "%s: not found dt!\n", __func__);

	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PD_CTRL, &data);
	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_ADC_STATUS, &rid);
	s2mu004_usbpd_read_reg(i2c, S2MU004_REG_PLUG_CTRL_PORT, &init_port);

	rid &= S2MU004_PDIC_RID_MASK;
	rid >>= S2MU004_PDIC_RID_SHIFT;
	init_port &= 0x3;

	pdic_data->rid = rid;

	if (init_port == S2MU004_REG_PLUG_CTRL_DFP) {
		pdic_data->lpm_mode = true;
		pdic_data->is_factory_mode = false;
	} else if (rid) {
		pdic_data->lpm_mode = false;
		pdic_data->is_factory_mode = false;
		if (factory_mode) {
			if (rid != REG_RID_523K) {
				dev_err(dev, "%s : In factory mode, but RID is not 523K\n", __func__);
			} else {
				dev_err(dev, "%s : In factory mode, but RID is 523K OK\n", __func__);
				pdic_data->is_factory_mode = true;
			}
		}
	} else {
		dev_err(dev, "%s : Initial abnormal state to LPM Mode\n", __func__);
		pdic_port = s2mu004_check_init_port(pdic_data);
		s2mu004_set_lpm_mode(pdic_data);
		pdic_data->lpm_mode = true;
		msleep(150);
		pdic_data->is_factory_mode = false;
		if (pdic_port) {
			s2mu004_set_normal_mode(pdic_data);
		}
	}
#if 0
	if (factory_mode) {
		pdic_data->lpm_mode = false;
		pdic_data->is_factory_mode = true;
	} else
		s2mu004_set_lpm_mode(pdic_data);
#endif
	pdic_data->check_msg_pass = false;
	pdic_data->vconn_source = USBPD_VCONN_OFF;
	pdic_data->rid = REG_RID_MAX;
	pdic_data->is_host = 0;
	pdic_data->is_client = 0;
	pdic_data->data_role_dual = 0;
	pdic_data->power_role_dual = 0;
	pdic_data->is_attached = 0;
	pdic_data->is_water_detect = false;
	pdic_data->detach_valid = true;
	pdic_data->is_otg_vboost = false;
	pdic_data->is_otg_reboost = false;

	ret = usbpd_init(dev, pdic_data);
	if (ret < 0) {
		dev_err(dev, "failed on usbpd_init\n");
		goto err_return;
	}

	usbpd_set_ops(dev, &s2mu004_ops);

	mutex_init(&pdic_data->_mutex);
	mutex_init(&pdic_data->lpm_mutex);

	s2mu004_usbpd_reg_init(pdic_data);

	pdic_data->pdic_queue =
	    alloc_workqueue(dev_name(dev), WQ_MEM_RECLAIM, 1);
	if (!pdic_data->pdic_queue) {
		dev_err(dev,
			"%s: Fail to Create Workqueue\n", __func__);
		goto err_return;
	}
	INIT_DELAYED_WORK(&pdic_data->plug_work, s2mu004_src_sink_init);

#if defined(CONFIG_CCIC_NOTIFIER)
	ccic_notifier_init();
	/* Create a work queue for the ccic irq thread */
	pdic_data->ccic_wq
		= create_singlethread_workqueue("ccic_irq_event");
	if (!pdic_data->ccic_wq) {
		pr_err("%s failed to create work queue for ccic notifier\n", __func__);
		goto err_return;
	}
	if (pdic_data->rid == REG_RID_UNDF)
		pdic_data->rid = REG_RID_MAX;
	dev_set_drvdata(ccic_device, pdic_data);
#endif

	ret = s2mu004_usbpd_irq_init(pdic_data);
	if (ret) {
		dev_err(dev, "%s: failed to init irq(%d)\n", __func__, ret);
		goto fail_init_irq;
	}

	s2mu004_irq_thread(-1, pdic_data);

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_ccic_notifier_register(&pdic_data->type3_nb,
			       type3_handle_notification,
			       MUIC_NOTIFY_DEV_PDIC);
#endif
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
		desc =
			devm_kzalloc(&i2c->dev,
					 sizeof(struct dual_role_phy_desc), GFP_KERNEL);
		if (!desc) {
			pr_err("unable to allocate dual role descriptor\n");
			goto fail_init_irq;
		}

		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_local_prop;
		desc->set_property = dual_role_set_prop;
		desc->properties = fusb_drp_properties;
		desc->num_properties = ARRAY_SIZE(fusb_drp_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		dual_role =
			devm_dual_role_instance_register(&i2c->dev, desc);
		dual_role->drv_data = pdic_data;
		pdic_data->dual_role = dual_role;
		pdic_data->desc = desc;
		init_completion(&pdic_data->reverse_completion);
		INIT_DELAYED_WORK(&pdic_data->role_swap_work, role_swap_check);
#endif

	dev_info(dev, "%s s2mu004 usbpd driver uploaded!\n", __func__);

	return 0;

fail_init_irq:
	if (i2c->irq)
		free_irq(i2c->irq, pdic_data);
err_return:
	return ret;
}

#if defined CONFIG_PM
static int s2mu004_usbpd_suspend(struct device *dev)
{
	struct usbpd_data *_data = dev_get_drvdata(dev);
	struct s2mu004_usbpd_data *pdic_data = _data->phy_driver_data;

	if (device_may_wakeup(dev))
		enable_irq_wake(pdic_data->i2c->irq);

	disable_irq(pdic_data->i2c->irq);

	return 0;
}

static int s2mu004_usbpd_resume(struct device *dev)
{
	struct usbpd_data *_data = dev_get_drvdata(dev);
	struct s2mu004_usbpd_data *pdic_data = _data->phy_driver_data;

	if (device_may_wakeup(dev))
		disable_irq_wake(pdic_data->i2c->irq);

	enable_irq(pdic_data->i2c->irq);

	return 0;
}
#else
#define s2mu004_muic_suspend NULL
#define s2mu004_muic_resume NULL
#endif

static int s2mu004_usbpd_remove(struct i2c_client *i2c)
{
	struct s2mu004_usbpd_data *_data = i2c_get_clientdata(i2c);

	if (_data) {
#if defined(CONFIG_DUAL_ROLE_USB_INTF)
		devm_dual_role_instance_unregister(_data->dev, _data->dual_role);
		devm_kfree(_data->dev, _data->desc);
#endif
		disable_irq_wake(_data->i2c->irq);
		free_irq(_data->i2c->irq, _data);
		mutex_destroy(&_data->_mutex);
		i2c_set_clientdata(_data->i2c, NULL);
		kfree(_data);
	}
	return 0;
}

static const struct i2c_device_id s2mu004_usbpd_i2c_id[] = {
	{ USBPD_DEV_NAME, 1 },
	{}
};
MODULE_DEVICE_TABLE(i2c, s2mu004_i2c_id);

static struct of_device_id sec_usbpd_i2c_dt_ids[] = {
	{ .compatible = "sec-usbpd,i2c" },
	{ }
};

static void s2mu004_usbpd_shutdown(struct i2c_client *i2c)
{
	struct s2mu004_usbpd_data *_data = i2c_get_clientdata(i2c);

	if (!_data->i2c)
		return;
}

static usbpd_phy_ops_type s2mu004_ops = {
	.tx_msg			= s2mu004_tx_msg,
	.rx_msg			= s2mu004_rx_msg,
	.hard_reset		= s2mu004_hard_reset,
	.set_power_role		= s2mu004_set_power_role,
	.get_power_role		= s2mu004_get_power_role,
	.set_data_role		= s2mu004_set_data_role,
	.get_data_role		= s2mu004_get_data_role,
	.set_vconn_source	= s2mu004_set_vconn_source,
	.get_vconn_source	= s2mu004_get_vconn_source,
	.set_check_msg_pass	= s2mu004_set_check_msg_pass,
	.get_status		= s2mu004_get_status,
	.poll_status		= s2mu004_poll_status,
	.driver_reset  		= s2mu004_driver_reset,
};

#if defined CONFIG_PM
const struct dev_pm_ops s2mu004_usbpd_pm = {
	.suspend = s2mu004_usbpd_suspend,
	.resume = s2mu004_usbpd_resume,
};
#endif

static struct i2c_driver s2mu004_usbpd_driver = {
	.driver		= {
		.name	= USBPD_DEV_NAME,
		.of_match_table	= sec_usbpd_i2c_dt_ids,
#if defined CONFIG_PM
		.pm	= &s2mu004_usbpd_pm,
#endif /* CONFIG_PM */
	},
	.probe		= s2mu004_usbpd_probe,
	.remove		= s2mu004_usbpd_remove,
	.shutdown	= s2mu004_usbpd_shutdown,
	.id_table	= s2mu004_usbpd_i2c_id,
};

static int __init s2mu004_usbpd_init(void)
{
	return i2c_add_driver(&s2mu004_usbpd_driver);
}
late_initcall(s2mu004_usbpd_init);

static void __exit s2mu004_usbpd_exit(void)
{
	i2c_del_driver(&s2mu004_usbpd_driver);
}
module_exit(s2mu004_usbpd_exit);

MODULE_DESCRIPTION("S2MU004 USB PD driver");
MODULE_LICENSE("GPL");
