#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/ioctl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <linux/mfd/core.h>
#include <linux/mfd/nvtec.h>
#include <linux/nvcommon.h>

/* Daniel Wang */
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <linux/switch_gpio.h>

#define NVTEC_MAGIC_ENTER_FLASH_MODE_DEFAULT    (0xAA55)
#define NVTEC_MAGIC_ENTER_FLASH_MODE_CONFIG     (0xEFBE)
#define NVTEC_MAGIC_EXIT_FLASH_MODE             (0xAA55)

#define NVTEC_MAX_FW_SIZE               65536 /* 64K */
#define NVTEC_MAX_DATAFLASH_SIZE	21	/* 21 Bytes */	
//#define NVTEC_I2C_MAX_PACKET_SIZE       34

#define MAXLEN	32

struct nvtec {
    struct mutex    lock;
    struct device   *dev;
    struct i2c_client *client;

    int request_pin;
    int pwr_state_pin;
    int ap_wake_pin;
    int wake_pin;
    int hdmi_sw_pin;
    int flash_mode;
    int data_flash_mode;
    int state;
    int lock_state; /* 0=lock, 1=unlock */

    struct blocking_notifier_head undock_notifier;
    struct blocking_notifier_head system_notifier;
    struct blocking_notifier_head battery_notifier;
    struct blocking_notifier_head cir_notifier;

    struct notifier_block switch_gpio_notifier;

    wait_queue_head_t wq_ack;
};

enum nvtec_state {
    NVTEC_STATE_S0 = 0,
    NVTEC_STATE_S1 = 1,
    NVTEC_STATE_S2 = 2,
    NVTEC_STATE_S3 = 3,
    NVTEC_STATE_S4 = 4,

    NVTEC_STATE_FACTORY = 0xe0,
    NVTEC_STATE_TEST = 0xf0,
    NVTEC_STATE_TEST1 = 0xf1,
    NVTEC_STATE_TEST2 = 0xf2,
    NVTEC_STATE_TEST3 = 0xf3,
    NVTEC_STATE_TEST4 = 0xf4,
    NVTEC_STATE_TEST5 = 0xf5,
    NVTEC_STATE_TEST6 = 0xf6,
    NVTEC_STATE_TEST7 = 0xf7,
    NVTEC_STATE_TEST8 = 0xf8,
    NVTEC_STATE_TEST9 = 0xf9,
    NVTEC_STATE_TESTA = 0xfa,
    NVTEC_STATE_DISABLE = 0xfb,
    NVTEC_STATE_SHIP = 0xfc,
    NVTEC_STATE_RESET = 0xff,
};

enum nvtec_hdmi_switch_mode {
    NVTEC_HDMI_SW_OFF = 0,
    NVTEC_HDMI_SW_MB = 1,
    NVTEC_HDMI_SW_DOCK = 2,
    NVTEC_HDMI_SW_AUTO = 3,
    NVTEC_HDMI_SW_MAX,
};

/* Daniel Wang */
struct mcu_led_info{
    uint16_t raw;
    char *name;
};

struct mcu_led_info led_mode[5] = {
    {0x0,"mcu"},
    {0x150,"on"},
    {0x154,"off"},
    {0x155,"blink_1"},
    {0x156,"blink_2"}
};

struct mcu_lock_key_info{
    uint8_t raw;
    char *mode;
};

struct mcu_lock_key_info lock_mode[2] = {
    {0x1,"lock"},
    {0x0,"unlock"}
};

unsigned int percentage = 0;
unsigned short data_flash_return = 0xF;
static struct wake_lock upgrade_lock;
static NvBool upgrade_lock_flag = NV_FALSE;
static NvBool default_mcu_mode = NV_TRUE;

static struct nvtec *nvtec = NULL;
static int nvtec_set_state(struct i2c_client *client, uint16_t state);
static int nvtec_read_event(struct i2c_client *client, uint16_t *event);

static int __nvtec_read_byte(struct i2c_client *client, int reg, uint8_t *val)
{
    int ret = 0;
    int retry = 5;
    
    while (retry --) {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    if (ret >= 0)
        *val = (uint8_t) ret;

    return (ret < 0) ? ret : 0;
}

static int __nvtec_read_word(struct i2c_client *client, int reg, uint16_t *val)
{
    int ret = 0;
    int retry = 5;
    
    while (retry --) {
        ret = i2c_smbus_read_word_data(client, reg);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    if (ret >= 0)
        *val = (uint16_t) ret;

    return (ret < 0) ? ret : 0;
}

static int __nvtec_read_block(struct i2c_client *client, int reg, 
                                     int len, uint8_t *val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int __nvtec_write_byte(struct i2c_client *client, int reg, uint8_t val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed writing 0x%02x ot 0x%02x\n",
                    val, reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int __nvtec_write_word(struct i2c_client *client, int reg, uint16_t val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_word_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed writing 0x%04x to 0x%02x\n",
                    val, reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

static int __nvtec_write_block(struct i2c_client *client, int reg,
                                      int len, uint8_t *val)
{
    int ret = 0;
    int retry = 5;

    while (retry --) {
        ret = i2c_smbus_write_block_data(client, reg, len, val);
        if (ret < 0) {
            dev_err(&client->dev, "failed write to 0x%02x\n", reg);
            msleep(35);
            continue;
        } else {
            break;
        }
    }

    return (ret < 0) ? ret : 0;
}

int nvtec_read_byte(struct device *dev, int reg, uint8_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_read_byte);

int nvtec_read_word(struct device *dev, int reg, uint16_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_read_word);

int nvtec_read_block(struct device *dev, int reg, int len, uint8_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_block(to_i2c_client(dev), reg, len, val);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_read_block);

int nvtec_write_byte(struct device *dev, int reg, uint8_t val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_byte(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_write_byte);

int nvtec_write_word(struct device *dev, int reg, uint16_t val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(to_i2c_client(dev), reg, val);
    mutex_unlock(&nvtec->lock);
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_write_word);

int nvtec_write_block(struct device *dev, int reg, int len, uint8_t *val)
{
    int ret;

    if (nvtec->state != NVTEC_STATE_S0 ||
        nvtec->flash_mode == 1)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_block(to_i2c_client(dev), reg, len, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_write_block);

int nvtec_prepare_to_power_off(void)
{
    return nvtec_set_state(nvtec->client, NVTEC_STATE_S4);
}
EXPORT_SYMBOL_GPL(nvtec_prepare_to_power_off);

int nvtec_register_event_notifier(struct device *dev, 
                                  enum nvtec_event_type event,
                                  struct notifier_block *nb)
{
    int ret;

    switch (event) {
    case NVTEC_EVENT_UNDOCK:
        ret = blocking_notifier_chain_register(&nvtec->undock_notifier, nb);
        break;
    case NVTEC_EVENT_SYSTEM:
        ret = blocking_notifier_chain_register(&nvtec->system_notifier, nb);
        break;
    case NVTEC_EVENT_BATTERY:
        ret = blocking_notifier_chain_register(&nvtec->battery_notifier, nb);
        break;
    case NVTEC_EVENT_CIR:
        ret = blocking_notifier_chain_register(&nvtec->cir_notifier, nb);
        break;
    default:
        ret = -EINVAL;
        break;
    }
    
    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_register_event_notifier);

int nvtec_unregister_event_notifier(struct device *dev,
                                    enum nvtec_event_type event,
                                    struct notifier_block *nb)
{
    int ret;

    switch (event) {
    case NVTEC_EVENT_UNDOCK:
        ret = blocking_notifier_chain_unregister(&nvtec->undock_notifier, nb);
        break;
    case NVTEC_EVENT_SYSTEM:
        ret = blocking_notifier_chain_unregister(&nvtec->system_notifier, nb);
        break;
    case NVTEC_EVENT_BATTERY:
        ret = blocking_notifier_chain_unregister(&nvtec->battery_notifier, nb);
        break;
    case NVTEC_EVENT_CIR:
        ret = blocking_notifier_chain_unregister(&nvtec->cir_notifier, nb);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}
EXPORT_SYMBOL_GPL(nvtec_unregister_event_notifier);

static int nvtec_get_firmware_version(struct i2c_client *client, uint8_t *ver_str)
{
    int ret;
    char version[32];
    int len;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_block(client, 0xC0, 32, version);
    mutex_unlock(&nvtec->lock);

    if (ret < 0)
        return ret;

    len = (version[0] >= 31) ? 31 : version[0];
    strncpy(ver_str, &version[1], len);
    ver_str[len] = '\0';

    /* Daniel Wang */
    if ((version[16]=='5') && (version[17] =='A'))
    {
	printk("nvtec: M0516LBN 64KB \n");
    }
    else
    {
	printk("nvtec: Unknow MCU Chip Set to default %d KB\n",NVTEC_MAX_FW_SIZE/1024);	
    }
	
    return ret;
}

static int cmd_ack = 0;

int nvtec_battery_get_level(void)
{
	uint16_t cap = 0;
	uint16_t val = 0;
	int level = 0;
	int ret;

	mutex_lock(&nvtec->lock);
	ret = __nvtec_read_word(nvtec->client, 0x0D, &cap);
	mutex_unlock(&nvtec->lock);

    if (ret < 0 || cap < 0 || cap > 100)
    {
		return -EINVAL;
    }

    if (cap < 5)
		val = 0;
	else
		val = (int)(cap-5)*100/95;

	if ((val >= 0) && (val <= 5))
		level = 0;
	else if ((val > 5) && (val <= 35))
		level = 1;
	else if ((val > 35) && (val <= 70))
		level = 2;
	else if ((val > 70) && (val <= 99))
		level = 3;
	else
		level = 4;

    return level;
}

int nvtec_battery_full_charged(void)
{
	uint16_t val = 0;
	int ret;
	int full_charged;

	mutex_lock(&nvtec->lock);
	ret = __nvtec_read_word(nvtec->client, 0x16, &val);
	mutex_unlock(&nvtec->lock);

    if (ret < 0 )
    {
		return -EINVAL;
    }

	if (val & 0x0020)
		full_charged = 1;
	else
		full_charged = 0;

    return full_charged;
}

int nvtec_hw_button_lock_configured(void)
{
	uint8_t val = 0;
	int ret;
	int configured;

	mutex_lock(&nvtec->lock);
	ret = __nvtec_read_byte(nvtec->client, 0xD2, &val);
	mutex_unlock(&nvtec->lock);

    if (ret < 0 )
    {
		return -EINVAL;
    }

	if (val)
		configured = NV_TRUE;
	else
		configured = NV_FALSE;

    return configured;
}

int nvtec_mcu_chk_version(void)
{
	int ret;
	char version[32];
	int a, b, c;
	int i, offset=0;

	memset(version, '\0', sizeof(version));

	mutex_lock(&nvtec->lock);
	ret = __nvtec_read_block(nvtec->client, 0xC0, 32, version);
	mutex_unlock(&nvtec->lock);

    if (ret < 0) {
        return -ENODEV;
    }

	for (i=0; i<32; i++) {
		if (version[i] == 'A') {
			offset = i;
			break;
		}
	}

	a = version[offset+11]-'0';
	b = version[offset+12]-'0';
	c = 10*a +  b;

	//printk("version=%s, a=%d, b=%d, ab=%d\n", version, a, b,c);

	if (c > 6) //after T07 or V07 (include)
		return 1;
	else
		return 0;
}

static int nvtec_enter_flash_mode(struct i2c_client *client, uint16_t magic)
{
    uint16_t event;
    int ret;

    percentage = 0;	/* Daniel Wang */

    if (nvtec->flash_mode) {
        printk("nvtec: already in flash mode\n");
        return 0;
    }

    if (nvtec->state != NVTEC_STATE_S0) {
        printk("nvtec: EC not in S0 state\n");
        return -EINVAL;
    }

    if (magic != NVTEC_MAGIC_ENTER_FLASH_MODE_DEFAULT &&
        magic != NVTEC_MAGIC_ENTER_FLASH_MODE_CONFIG) {
        printk("nvtec: Invalid magic number\n");
        return -EINVAL;
    }

    /* Daniel Wang */
    if (upgrade_lock_flag == NV_FALSE){
    	wake_lock(&upgrade_lock);
	upgrade_lock_flag = NV_TRUE;
	printk("nvtec: upgrade wake_lock locked\n");
    }

    suspend_battery_thread();

    mutex_lock(&nvtec->lock);

    /* clean event */
    while (gpio_get_value(nvtec->ap_wake_pin) == 0) {
        ret = nvtec_read_event(client, &event);
        if (ret < 0 || event == 0)
            break;
    }

    printk("nvtec: enter flash mode\n");
    nvtec->flash_mode = 1;
    
    cmd_ack = 0;
    ret= __nvtec_write_word(client, 0xe0, magic);
    if (ret < 0) {
        nvtec->flash_mode = 0;
        mutex_unlock(&nvtec->lock);
        return ret;
    }
    
    wait_event(nvtec->wq_ack, cmd_ack == 1);

    mutex_unlock(&nvtec->lock);
    
    return 0;
}

static int nvtec_exit_flash_mode(struct i2c_client *client, uint16_t magic)
{
    int ret;

    if (nvtec->flash_mode == 0)
        return -EINVAL;

    if (magic != NVTEC_MAGIC_EXIT_FLASH_MODE) {
        printk("nvtec: Invalid magic number\n");
        return -EINVAL;
    }

    mutex_lock(&nvtec->lock);

    cmd_ack = 0;
    ret = __nvtec_write_word(client, 0xe3, magic);
    if (ret < 0) {
    	mutex_unlock(&nvtec->lock);
        return ret;
    }

    wait_event(nvtec->wq_ack, cmd_ack == 1);

    printk("nvtec: exit flash mode\n");
    
    nvtec->flash_mode = 0;

    mutex_unlock(&nvtec->lock);

    /* Daniel Wang */
    /* EC will reset itself when exit flash mode */
    /* after reset, EC at S0 state Scorpio */
    nvtec->state = NVTEC_STATE_S0; 

    resume_battery_thread();

    /* Daniel Wang */
    if (upgrade_lock_flag == NV_TRUE){
    	wake_unlock(&upgrade_lock);
	upgrade_lock_flag = NV_FALSE;
	printk("nvtec: upgrade wake_lock unlocked\n");
    }

    return 0;
}

static int nvtec_enter_data_flash_mode(struct i2c_client *client, uint16_t temp)
{
    if (nvtec->data_flash_mode) {
        printk("nvtec: already in data flash mode\n");
        return 0;
    }

    if (nvtec->state != NVTEC_STATE_S0) {
        printk("nvtec: EC not in S0 state\n");
        return -EINVAL;
    }

    if (upgrade_lock_flag == NV_FALSE){
    	wake_lock(&upgrade_lock);
	upgrade_lock_flag = NV_TRUE;
	printk("nvtec: upgrade wake_lock locked\n");
    }

    suspend_battery_thread();

    mutex_lock(&nvtec->lock);

    nvtec->data_flash_mode = 1;
    printk("nvtec: enter data flash mode\n");

    mutex_unlock(&nvtec->lock);
    
    return 0;
}

static int nvtec_exit_data_flash_mode(struct i2c_client *client, unsigned short *err)
{
    int ret = 0 ;
    uint8_t return_value [21] ; 

    if (nvtec->data_flash_mode == 0)
        return -EINVAL;
    ret = __nvtec_read_block(client, 0xc2, 21, return_value);
    if (ret != 0)
	return -EINVAL;

    *err = 0x0F;

    if (return_value[1] != 0){
	printk("nvtec: dat flash upgrading ... waiting [%d] [%d]\n", return_value[0] ,return_value[1]);
	data_flash_return = *err;
	return 0;
    }else if(return_value[2] != 0){
	printk("nvtec: data flash checksum  ... Fail \n");
	*err &= 0x07;
    }else if(return_value[3] != 0){
	printk("nvtec: data flash binary    ... Fail \n");
	*err &= 0x03;
    }else if(return_value[4] != 0){
	printk("nvtec: data flash upgrade   ... Fail \n");
	*err &= 0x01;
    }else{
	printk("nvtec: data flash upgrade   ... Successed \n");
	*err &= 0x00;
    }
    
    data_flash_return = *err;

    mutex_lock(&nvtec->lock);
    nvtec->data_flash_mode = 0;
    printk("nvtec: exit data flash mode\n");
    mutex_unlock(&nvtec->lock);

    resume_battery_thread();

    if (upgrade_lock_flag == NV_TRUE){
    	wake_unlock(&upgrade_lock);
	upgrade_lock_flag = NV_FALSE;
	printk("nvtec: upgrade wake_lock unlocked\n");
    }

    return 0;
}

static int nvtec_set_lock_key_status(struct i2c_client *client, char *status)
{
    int i;
    int ret = 0;

    for (i=0;i<3;i++){
	if (!strcmp(status,lock_mode[i].mode)){
	    mutex_lock(&nvtec->lock);
	    ret = __nvtec_write_byte(client, 0xd2, lock_mode[i].raw);
	    mutex_unlock(&nvtec->lock);
	    if (ret < 0)
		return ret;
	    /*printk("%s Set Lock key with [%s]\n", __func__,status);*/
	    break;
	}else if (i == 2){
	    printk("nvtec: %s Unknow command [%s] \n", __func__,status);
	    break;
	}
    } 
    return ret;
}

static int nvtec_get_lock_key_status(struct i2c_client *client, char *status)
{
    int i;
    int  ret = 0;
    uint8_t raw_data;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(client, 0xd2, &raw_data);
    mutex_unlock(&nvtec->lock);
    if (ret < 0)
	return ret;
    
    for (i=0;i<3;i++){
	if (raw_data == lock_mode[i].raw){
	    strcpy(status,lock_mode[i].mode);
	    /*printk("%s Get Lock key status [%s]\n", __func__,status);*/

	    if (nvtec != NULL)
            	nvtec->lock_state = i;
	    else
		printk("%s: nvtec is NULL\n", __func__);

	    break;
	}else if (i == 2){
	    strcpy(status,"unknow"); 
	    printk("nvtec: %s Get Lock key status [%s]\n", __func__,status);
	    break;
	} 
    }

    return ret;
}

bool nvtec_get_lock_key_enabled() {
	printk("%s enter\n", __func__);
	if (nvtec == NULL) {
		printk("%s: nvtec is NULL\n", __func__);
		return false;
	} else {
		return (nvtec->lock_state == 0);
	}
}
EXPORT_SYMBOL_GPL(nvtec_get_lock_key_enabled);

static int switch_gpio_notify(struct notifier_block *nb, unsigned long state, void *unused)
{
	struct nvtec *ec = container_of(nb, struct nvtec, switch_gpio_notifier); 
	struct i2c_client *client = ec->client;

	if (client == NULL) {
		printk("%s: i2c client is NULL\n", __func__);
		return NOTIFY_OK;
	}

	/*printk("%s: getting state %ld\n", __func__, state);*/

	if (state == SWITCH_LOCK_ENABLED) {
		nvtec_set_lock_key_status(client, "lock");
	} else if (state == SWITCH_LOCK_DISABLED) {
		nvtec_set_lock_key_status(client, "unlock");
	} else {
		printk("%s: state %ld is wrong\n", __func__, state);
	}
	return NOTIFY_OK;
}

static int nvtec_get_flash_id(struct i2c_client *client, uint8_t *id)
{
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_block(client, 0xc1, 32, id);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_get_state(struct i2c_client *client, uint16_t *state)
{
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(client, 0xd1, state);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_state(struct i2c_client *client, uint16_t state)
{
    int ret = 0;

    if (state > NVTEC_STATE_S4 &&
        state != NVTEC_STATE_TEST &&
        state != NVTEC_STATE_RESET &&
	state != NVTEC_STATE_FACTORY &&
	state != NVTEC_STATE_TEST1 &&
	state != NVTEC_STATE_TEST2 &&
	state != NVTEC_STATE_TEST3 &&
	state != NVTEC_STATE_TEST4 &&
	state != NVTEC_STATE_TEST5 &&
	state != NVTEC_STATE_TEST6 &&
	state != NVTEC_STATE_TEST7 &&
	state != NVTEC_STATE_TEST8 &&
	state != NVTEC_STATE_TEST9 &&
	state != NVTEC_STATE_TESTA &&
	state != NVTEC_STATE_DISABLE &&
	state != NVTEC_STATE_SHIP)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    /* Daniel Wang */
    if (nvtec->state == NVTEC_STATE_S0){
	if (state == NVTEC_STATE_S3){
		/* Suspend 	1> send cmd 2> trigger gpio (EC_1 = 1->0  EC_2 = 1->1) */	
		ret = __nvtec_write_word(client, 0xd1, state);
		gpio_set_value(nvtec->request_pin, 0);
		gpio_set_value(nvtec->pwr_state_pin, 1);
		if ((machine_is_avalon()) && (nvtec->hdmi_sw_pin != 0))
		    gpio_set_value(nvtec->hdmi_sw_pin, 0);
		printk("nvtec: Enter suspend status (S3) \n");
	}else if (state == NVTEC_STATE_S4){
		/* Power off	1> send cmd 2> trigger gpio (EC_1 = 1->0  EC_2 = 1->0) */
                //ret = __nvtec_write_word(client, 0xd1, state);
                gpio_set_value(nvtec->request_pin, 0);
                gpio_set_value(nvtec->pwr_state_pin, 0);
		if ((machine_is_avalon()) && (nvtec->hdmi_sw_pin != 0))
		    gpio_set_value(nvtec->hdmi_sw_pin, 0);
		printk("nvtec: Enter power off status (S4) \n");		
	}else{
		printk("nvtec: S0 Undefined status (S%d -> S%d)\n",nvtec->state,state);
		/* Do not change Daniel Wang */
		ret = __nvtec_write_word(client, 0xd1, state);
                mutex_unlock(&nvtec->lock);
    		return ret;
	}
    }else{
	if (state == NVTEC_STATE_S0){
    		/* Resume   1> trigger gpio 2> send cmd (EC_1 = 0->1  EC_2 = 1->1) */
		gpio_set_value(nvtec->request_pin, 1);
		gpio_set_value(nvtec->pwr_state_pin, 1);
		if ((machine_is_avalon()) && (nvtec->hdmi_sw_pin != 0))
		    gpio_set_value(nvtec->hdmi_sw_pin, 1);
		//ret = __nvtec_write_word(client, 0xd1, state);
                msleep(30);
		printk("nvtec: Enter resume status (S0) \n");
	}else{
		printk("nvtec: non-S0 Undefined status (S%d -> S%d)\n",nvtec->state,state);
		/* Do not change Daniel Wang */
		ret = __nvtec_write_word(client, 0xd1, state);
                mutex_unlock(&nvtec->lock);
    		return ret;
	}
    }
    nvtec->state = state;
    mutex_unlock(&nvtec->lock);
    return ret;
}

static int nvtec_get_board_version (struct i2c_client *client, unsigned short *board_id){

    if (machine_is_avalon()){
	printk("nvtec: machine_is_avalon \n");
	*board_id = 0;	
    }else if (machine_is_sphinx()){
	printk("nvtec: machine_is_sphinx \n");
	*board_id = 1;
    }else if (machine_is_titan()){
	printk("nvtec: machine_is_titan \n");
	*board_id = 2;
    }else{
	printk("nvtec: machine unknow \n");
	return -EINVAL;
    }
    return 0;
}

static int nvtec_get_hdmi_sw_mode(struct i2c_client *client, uint16_t *mode)
{
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(client, 0xd2, mode);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_hdmi_sw_mode(struct i2c_client *client, uint16_t mode)
{
    int ret;

    if (mode > NVTEC_HDMI_SW_MAX)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd2, mode);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_hdmi_p1_strength(struct i2c_client *client, int strength)
{
    int ret;
    uint16_t value;

    if (strength > 3)
        return -EINVAL;
    
    value = (0x0d << 8) | (strength & 0xff);

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd3, value);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_hdmi_p2_strength(struct i2c_client *client, int strength)
{
    int ret;
    uint16_t value;

    if (strength > 3)
        return -EINVAL;

    value = (0x0C << 8) | (strength & 0xff);

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd3, value);
    mutex_unlock(&nvtec->lock);

    return ret;
}

/* Daniel Wang */
static int nvtec_get_led_mode(struct i2c_client *client, uint16_t *mode)
{
    /* [Patch from Ants] Fine tune EC driver to enhance the communication between tegra and EC */
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_word(client, 0xd9, mode);
    mutex_unlock(&nvtec->lock);

    return ret;
}

/* Daniel Wang */
static int nvtec_set_led_mode(struct i2c_client *client, uint16_t mode)
{
    /* [Patch from Ants] Fine tune EC driver to enhance the communication between tegra and EC */
    int ret;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_word(client, 0xd9, mode);
    mutex_unlock(&nvtec->lock);

    return ret;
}

/* static int nvtec_reset(struct i2c_client *client) */
/* { */
/*     return nvtec_set_state(client, NVTEC_STATE_RESET); */
/* } */

/* static int nvtec_test_mode(struct i2c_client *client) */
/* { */
/*     return nvtec_set_state(client, NVTEC_STATE_TEST); */
/* } */

static int nvtec_read_gpio_input(struct i2c_client *client, uint8_t port, uint8_t *val)
{
	int ret;

    if (port > 9)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(client, 0xa0 + port, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_read_gpio_output(struct i2c_client *client, uint8_t port, uint8_t *val)
{
	int ret;

    if (port > 9)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_read_byte(client, 0xb0 + port, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_set_gpio_output(struct i2c_client *client, uint8_t port, uint8_t val)
{
	int ret;

    if (port > 9)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    ret = __nvtec_write_byte(client, 0xb0 + port, val);
    mutex_unlock(&nvtec->lock);

    return ret;
}

static int nvtec_transfer_firmware(struct i2c_client *client, uint8_t *buffer, int len)
{
    int ret;
    int byte_transferred = 0;
    int transfer_len = 0;
    
    if (NULL == buffer || len != NVTEC_MAX_FW_SIZE)
        return -EINVAL;
    
    if (nvtec->flash_mode == 0)
        return -EINVAL;

    mutex_lock(&nvtec->lock);
    while (byte_transferred != len) {
        if ((len - byte_transferred) >= I2C_SMBUS_BLOCK_MAX)
            transfer_len = I2C_SMBUS_BLOCK_MAX;
        else
            transfer_len = len - byte_transferred;

        cmd_ack = 0;
        ret = __nvtec_write_block(client, 0xe2,
                                  transfer_len,
                                  &buffer[byte_transferred]);

        if (ret != 0) {
            mutex_unlock(&nvtec->lock);
            return -EINVAL;
        }

        byte_transferred += transfer_len;
        
	percentage=((byte_transferred*100)/NVTEC_MAX_FW_SIZE);	/* Daniel Wang */
        
        printk("nvtec:(%3d) \b\b\b\b\b\b\b\b\b\b\b\b",percentage);
        wait_event(nvtec->wq_ack, cmd_ack == 1);

    }
    mutex_unlock(&nvtec->lock);

    return len;
}

static int nvtec_transfer_dataflash(struct i2c_client *client, uint8_t *buffer, int len){
    int ret = 0;
    
    if (NULL == buffer || len != NVTEC_MAX_DATAFLASH_SIZE)
        return -EINVAL;
    
    if (nvtec->data_flash_mode == 0)
	return -EINVAL;

    mutex_lock(&nvtec->lock);

    ret = __nvtec_write_block(client, 0xc2,
				len,
				&buffer[0]);

    if (ret != 0) {
        mutex_unlock(&nvtec->lock);
        return -EINVAL;
    }

    mutex_unlock(&nvtec->lock);

    return len;
}

static int nvtec_flash_acknowledge(struct i2c_client *client)
{
    uint8_t value;
    int ret;

    /* no need protect */
    ret = __nvtec_read_byte(client, 0x0, &value);
    if (ret < 0)
        return -EFAULT;

    if (value == 0xfa) {
        cmd_ack = 1;
        wake_up(&nvtec->wq_ack);
        
        return 0;
    } else
        return -EFAULT;
}

static int nvtec_read_event(struct i2c_client *client, uint16_t *event)
{
    /* no need protect */
    return __nvtec_read_word(client, 0xd0, event);
}

static int __remove_subdev(struct device *dev, void *unused)
{
    platform_device_unregister(to_platform_device(dev));
    return 0;
}

static int nvtec_remove_subdevs(struct nvtec *ec)
{
    return device_for_each_child(ec->dev, NULL, __remove_subdev);
}

static int __devinit nvtec_add_subdevs(struct nvtec *ec, struct nvtec_platform_data *pdata)
{
    struct nvtec_subdev_info *subdev;
    struct platform_device *pdev;
    int i, ret = 0;

    for (i = 0; i < pdata->num_subdevs; i ++) {
        subdev = &pdata->subdevs[i];
        
        pdev = platform_device_alloc(subdev->name, subdev->id);

        pdev->dev.parent = ec->dev;
        pdev->dev.platform_data = subdev->platform_data;

        ret = platform_device_add(pdev);
        if (ret)
            goto failed;
    }
    return 0;

 failed:
    nvtec_remove_subdevs(ec);
    return ret;
}

static irqreturn_t nvtec_irq(int irq, void *data)
{
    struct nvtec *ec = data;
    uint16_t event = 0;
    uint8_t event_type = 0;
    int ret;

    if (ec->flash_mode) {
        ret = nvtec_flash_acknowledge(ec->client);
        return IRQ_HANDLED;
    }
 
    nvtec_read_event(ec->client, &event);
    event_type = (uint8_t)(event >> 8) & 0xff;

    switch (event_type) {
    case NVTEC_EVENT_UNDOCK:
        blocking_notifier_call_chain(&ec->undock_notifier, event, NULL);
        break;
    case NVTEC_EVENT_CIR:
    case NVTEC_EVENT_CIR_REPEAT:
        blocking_notifier_call_chain(&ec->cir_notifier, event, NULL);
        break;
    case NVTEC_EVENT_BATTERY:
        blocking_notifier_call_chain(&ec->battery_notifier, event, NULL);
        break;
    case NVTEC_EVENT_SYSTEM:
        blocking_notifier_call_chain(&ec->system_notifier, event, NULL);
        break;
    default:
        break;
    }

    return IRQ_HANDLED;
}

static int __devinit nvtec_irq_init(struct nvtec *ec, int irq)
{
    int ret;

    ret = request_threaded_irq(irq, NULL, nvtec_irq, IRQF_TRIGGER_FALLING, 
                               "nvtec", ec);
    if (!ret) {
        device_init_wakeup(ec->dev, 1);
        enable_irq_wake(irq);
    }

    return ret;
}

static ssize_t nvtec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return -EINVAL;
}

static ssize_t nvtec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    uint8_t *fw_buf;
    int i;

    if (count == NVTEC_MAX_FW_SIZE){

        fw_buf = kzalloc(NVTEC_MAX_FW_SIZE, GFP_KERNEL);

        if (!fw_buf)
	    return -ENOMEM;

        memset(fw_buf, 0, NVTEC_MAX_FW_SIZE);

        if (copy_from_user(fw_buf, buf, count))
	    goto error;

        ret = nvtec_transfer_firmware(nvtec->client, fw_buf, count);
	if (ret != count)
	    goto error;

        kfree(fw_buf);
	return count;

    }else if (count == NVTEC_MAX_DATAFLASH_SIZE){

        fw_buf = kzalloc(NVTEC_MAX_DATAFLASH_SIZE, GFP_KERNEL);

        if (!fw_buf)
	    return -ENOMEM;

	memset(fw_buf, 0, NVTEC_MAX_DATAFLASH_SIZE);

        if (copy_from_user(fw_buf, buf, count))

	printk("\n");

	for (i=0;i<count;i++)
            printk("0x%02X ",*(fw_buf+i));

	printk("\n");

	ret = nvtec_transfer_dataflash(nvtec->client, fw_buf, count);
	if (ret != count){
	    printk("nvtec: ret = %d , count = %d \n", ret, count);
	    goto error;
	}

        kfree(fw_buf);
	return count;
    
    }else{
	return -ENOMEM; 
    }

 error:
    printk("nvtec: nvtec_write fail!\n");
    kfree(fw_buf);
    return -EINVAL;
}

struct flash_id {
    char _id[8];
};

#define NVTEC_IOCTL_ENTER_FLASH    _IOW('N', 1, unsigned short)
#define NVTEC_IOCTL_EXIT_FLASH     _IOW('N', 2, unsigned short)
#define NVTEC_IOCTL_SET_CONFIG     _IOW('N', 3, int)
#define NVTEC_IOCTL_GET_FLASH_ID   _IOR('N', 3, struct flash_id)
#define NVTEC_IOCTL_SET_LED_CONFIG _IOW('N', 4, unsigned int)
#define NVTEC_IOCTL_GET_LED_CONFIG _IOR('N', 5, unsigned int)
#define NVTEC_IOCTL_SET_STATE_CONFIG _IOW('N', 6, unsigned int)
#define NVTEC_IOCTL_GET_BOARD_ID   _IOR('N', 7, unsigned short)
#define NVTEC_ENTER_DATA_FLASH_MODE _IOW('N', 8, unsigned short)
#define NVTEC_EXIT_DATA_FLASH_MODE  _IOR('N', 9, unsigned short)
#define NVTEC_IOCTL_SET_LOCK_KEY    _IOW('N', 10, char *)
#define NVTEC_IOCTL_GET_LOCK_KEY    _IOR('N', 11, char *)

static long nvtec_ioctl(struct file *filp, unsigned int cmd, 
                       unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int ret = -EINVAL;
    uint16_t magic;
    uint16_t temp = 0;
    uint16_t err;
    uint16_t raw_config;
    uint16_t state_config;
    uint16_t id;
    char command[MAXLEN];

    switch (cmd) {
    case NVTEC_IOCTL_ENTER_FLASH:
        ret = copy_from_user(&magic, argp, sizeof(unsigned short));
        if (ret)
            return -EFAULT;

        ret = nvtec_enter_flash_mode(nvtec->client, magic);
        if (ret < 0)
            return -EFAULT;
        
        ret = 0;
        break;
    case NVTEC_IOCTL_EXIT_FLASH:
        ret = copy_from_user(&magic, argp, sizeof(unsigned short));
        if (ret)
            return -EFAULT;

        ret = nvtec_exit_flash_mode(nvtec->client, magic);
        if (ret < 0)
            return -EFAULT;
        
        ret = 0;
        break;
    case NVTEC_IOCTL_SET_CONFIG:
        break;

    case NVTEC_IOCTL_GET_FLASH_ID:
        break;
    /* Daniel Wang */
    case NVTEC_IOCTL_SET_LED_CONFIG:
    	ret = copy_from_user(&raw_config, argp, sizeof(uint16_t));
	if (ret < 0)
	    return -EFAULT;
	if (raw_config == 0x0)
	    raw_config = default_mcu_mode ? 0x0 : 0x200;
	ret = nvtec_set_led_mode(nvtec->client,raw_config);
	if (ret < 0)
	    return -EFAULT;
	ret = 0;
    	break;
    case NVTEC_IOCTL_GET_LED_CONFIG:
	ret = nvtec_get_led_mode(nvtec->client,&raw_config);
	if (ret < 0)
	    return -EFAULT;
	if (raw_config == 0x0 || raw_config == 0x200)
	    raw_config = 0;
	ret = copy_to_user(argp, &raw_config,sizeof(uint16_t));
	if (ret < 0)
	    return -EFAULT;
	ret = 0;
	break;
    case NVTEC_IOCTL_SET_STATE_CONFIG:
    	ret = copy_from_user(&state_config, argp, sizeof(uint16_t));
	if (ret < 0)
	    return -EFAULT;
	ret = nvtec_set_state(nvtec->client, state_config);
	if (ret < 0)
            return -EACCES;
	ret = 0;
    	break;
    case NVTEC_IOCTL_GET_BOARD_ID:
	ret = nvtec_get_board_version(nvtec->client, &id);
	if (ret < 0)
	   return -EFAULT; 
	ret = copy_to_user(argp, &id,sizeof(uint16_t));
	if (ret < 0)
	    return -EFAULT;
	ret = 0;
	break;
    case NVTEC_ENTER_DATA_FLASH_MODE:
	ret = nvtec_enter_data_flash_mode(nvtec->client, temp);
        if (ret < 0)
            return -EFAULT;
        ret = 0;
	break;
    case NVTEC_EXIT_DATA_FLASH_MODE:
	ret = nvtec_exit_data_flash_mode(nvtec->client, &err);
        if (ret < 0)
            return -EFAULT;
	ret = copy_to_user(argp, &err,sizeof(uint16_t));
        if (ret < 0)
            return -EFAULT;
        ret = 0;
	break;
    case NVTEC_IOCTL_SET_LOCK_KEY:
        memset(command, 0, sizeof(command));
    	ret = copy_from_user(command, argp, sizeof(command));
	if (ret < 0)
	    return -EFAULT;
	ret = nvtec_set_lock_key_status(nvtec->client, command);
	if (ret < 0)
	    return -EFAULT;
	ret = 0;
	break;
    case NVTEC_IOCTL_GET_LOCK_KEY:
        memset(command, 0, sizeof(command));
	ret = nvtec_get_lock_key_status(nvtec->client, command);
	if (ret < 0)
	    return -EFAULT;
	ret = copy_to_user(argp, command,sizeof(command));
	if (ret < 0)
	    return -EFAULT;
	ret = 0;
	break;
    default:
        break;
    }

    return ret;

}

const struct file_operations nvtec_fops = {
    .owner = THIS_MODULE,
    .read = nvtec_read,
    .write = nvtec_write,
    .unlocked_ioctl = nvtec_ioctl,
};

#define NVTEC_MINOR   111
static struct miscdevice nvtec_dev = {
    .minor = NVTEC_MINOR,
    .name = "nvtec",
    .fops = &nvtec_fops,
};

static char *get_state_string(uint16_t state)
{
    switch (state) {
    case NVTEC_STATE_S0:
        return "S0";
    case NVTEC_STATE_S1:
        return "S1";
    case NVTEC_STATE_S2:
        return "S2";
    case NVTEC_STATE_S3:
        return "S3";
    case NVTEC_STATE_S4:
        return "S4";
    case NVTEC_STATE_TEST:
        return "Test";
    case NVTEC_STATE_TEST1:
	return "Test_1";
    case NVTEC_STATE_TEST2:
	return "Test_2";
    case NVTEC_STATE_TEST3:
	return "Test_3";
    case NVTEC_STATE_TEST4:
	return "Test_4";
    case NVTEC_STATE_TEST5:
	return "Test_5";
    case NVTEC_STATE_TEST6:
	return "Test_6";
    case NVTEC_STATE_TEST7:
	return "Test_7";
    case NVTEC_STATE_TEST8:
	return "Test_8";
    case NVTEC_STATE_TEST9:
	return "Test_9";
    case NVTEC_STATE_TESTA:
	return "Test_A";
    case NVTEC_STATE_DISABLE:
	return "Disable";
    case NVTEC_STATE_SHIP:
	return "Ship";
    case NVTEC_STATE_RESET:
        return "Reset";
    case NVTEC_STATE_FACTORY:
	return "!QAZ2wsx"; 
    default:
        return NULL;
    }
}

static int str2state(const char *state_str)
{
    if (NULL == state_str)
        return -1;

    if(!strncmp("S0", state_str, 2))
        return NVTEC_STATE_S0;
    if(!strncmp("S1", state_str, 2))
        return NVTEC_STATE_S1;
    if(!strncmp("S2", state_str, 2))
        return NVTEC_STATE_S2;
    if(!strncmp("S3", state_str, 2))
        return NVTEC_STATE_S3;
    if(!strncmp("S4", state_str, 2))
        return NVTEC_STATE_S4;
    if(!strncmp("Test", state_str, 4))
        return NVTEC_STATE_TEST;
    if(!strncmp("1Test", state_str, 5))
        return NVTEC_STATE_TEST1;
    if(!strncmp("2Test", state_str, 5))
        return NVTEC_STATE_TEST2;
    if(!strncmp("3Test", state_str, 5))
        return NVTEC_STATE_TEST3;
    if(!strncmp("4Test", state_str, 5))
        return NVTEC_STATE_TEST4;
    if(!strncmp("5Test", state_str, 5))
        return NVTEC_STATE_TEST5;
    if(!strncmp("6Test", state_str, 5))
        return NVTEC_STATE_TEST6;
    if(!strncmp("7Test", state_str, 5))
        return NVTEC_STATE_TEST7;
    if(!strncmp("8Test", state_str, 5))
        return NVTEC_STATE_TEST8;
    if(!strncmp("9Test", state_str, 5))
        return NVTEC_STATE_TEST9;
    if(!strncmp("ATest", state_str, 5))
        return NVTEC_STATE_TESTA;
    if(!strncmp("Disable", state_str, 7))
        return NVTEC_STATE_DISABLE;
    if(!strncmp("Ship_mode", state_str, 9))
	return NVTEC_STATE_SHIP;
    if(!strncmp("Reset", state_str, 5))
        return NVTEC_STATE_RESET;
    if(!strncmp("!QAZ2wsx", state_str, 8))
	return NVTEC_STATE_FACTORY;

    return -1;
}

static ssize_t nvtec_state_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    int ret;
    uint16_t state;

    ret = nvtec_get_state(nvtec->client, &state);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", get_state_string(state));
}

static ssize_t nvtec_state_store(struct device *dev, 
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
    /* NvBool ret; */
    int ret;
    int state;

    state = str2state(buf);
    printk("state = 0x%x\n", state);
    if (state < 0){
	printk("nvtec: Error Unknow state \n");
	return count;
    }
        
    ret = nvtec_set_state(nvtec->client, (uint16_t) state);
    if (ret < 0){
	printk("nvtec: Error configure fail \n");	
    }
    
    return count;
}

static ssize_t nvtec_version_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    int ret;
    char version[32];
    
    ret = nvtec_get_firmware_version(nvtec->client, version);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", version);
}

static ssize_t nvtec_flash_id_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    int ret;
    char flash_id[32];
    
    ret = nvtec_get_flash_id(nvtec->client, flash_id);
    if (ret < 0)
        return 0;

    return sprintf(buf, "0x%02x 0x%02x 0x%02x 0x%02x\n0x%02x 0x%02x 0x%02x 0x%02x\n", 
                   flash_id[0], flash_id[1], flash_id[2], flash_id[3],
                   flash_id[4], flash_id[5], flash_id[6], flash_id[7]);
}


static char *get_mode_string(uint16_t mode)
{
    switch (mode) {
    case NVTEC_HDMI_SW_OFF:
        return "off";
    case NVTEC_HDMI_SW_MB:
        return "mb";
    case NVTEC_HDMI_SW_DOCK:
        return "dock";
    case NVTEC_HDMI_SW_AUTO:
        return "auto";
    default:
        return NULL;
    }
}

static int str2mode(const char *mode_str)
{
    if (NULL == mode_str)
        return -1;

    if(!strncmp("off", mode_str, 3))
        return NVTEC_HDMI_SW_OFF;
    if(!strncmp("mb", mode_str, 2))
        return NVTEC_HDMI_SW_MB;
    if(!strncmp("dock", mode_str, 4))
        return NVTEC_HDMI_SW_DOCK;
    if(!strncmp("auto", mode_str, 4))
        return NVTEC_HDMI_SW_AUTO;

    return -1;
}

static ssize_t nvtec_hdmi_switch_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int ret;
    uint16_t mode;

    ret = nvtec_get_hdmi_sw_mode(nvtec->client, &mode);
    if (ret < 0)
        return 0;

    return sprintf(buf, "%s\n", get_mode_string(mode));
}

static ssize_t nvtec_hdmi_switch_store(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    int ret;
    int mode;

    mode = str2mode(buf);
    if (mode < 0)
        return 0;

    ret = nvtec_set_hdmi_sw_mode(nvtec->client, (uint16_t) mode);
    if (ret < 0)
        return 0;
    
    return count;
}

static ssize_t nvtec_gpio_input_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int ret;
    uint8_t value[10];
    char temp_buf[16];
    int i;

    for (i = 0; i < 10; i ++) {
        ret = nvtec_read_gpio_input(nvtec->client, i , &value[i]);
        if (ret < 0)
            return 0;

        sprintf(temp_buf, "P%d:0x%02x\n", i, value[i]);
        strcat(buf, temp_buf);
    }

    return strlen(buf);
}

static ssize_t nvtec_gpio_output_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    int ret;
    uint8_t value[10];
    char temp_buf[16];
    int i;

    for (i = 0; i < 10; i ++) {
        ret = nvtec_read_gpio_output(nvtec->client, i, &value[i]);
        if (ret < 0)
            return 0;

        sprintf(temp_buf, "P%d:0x%02x\n", i, value[i]);
        strcat(buf, temp_buf);
    }

    return strlen(buf);
}

static ssize_t nvtec_gpio_output_store(struct device *dev, 
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    int ret;
    char *cp, *next;
    int port;
    int value;
    
    next = (char *)buf;

    cp = strsep(&next, " ");
    port = simple_strtoul(cp, NULL, 10);

    cp = strsep(&next, " ");
    value = simple_strtoul(cp, NULL, 16);

    ret = nvtec_set_gpio_output(nvtec->client, (uint8_t) port, (uint8_t) value & 0xff);
    if (ret < 0) {
        printk("NvtEC: can't set GPIO\n");
        return 0;
    }

    return count;
}

static ssize_t nvtec_cmd_write_store(struct device *dev, 
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    char *cp, *endp, *next;
    int cmd;
    uint8_t value[32];
    uint16_t word_value;
    int len = 0, i = 0;

    next = (char *)buf;
    cp = strsep(&next, " ");
    if (NULL == cp)
        return count;
    cmd = simple_strtoul(cp, &endp, 16);
    if (cp == endp || *endp != '\0') {
        return count;
    }

    cmd &= 0xff;

    while (len < 32 && cp != endp) {
           cp = strsep(&next, " ");
           if (NULL == cp) {
               break;
           }
           value[len++] = simple_strtoul(cp, &endp, 16);
    }

    printk("cmd:0x%02x\n", cmd);
    printk("value: len=%d\n", len);
    for (i = 0; i < len; i++)
        printk("0x%02x\n", value[i]);
    
    if (len == 0)
        return count;

    mutex_lock(&nvtec->lock);
    if (len == 1) {
        __nvtec_write_byte(nvtec->client, cmd, value[0]);
    }else if (len == 2) {
        word_value = value[1] << 8 | value[0];
        __nvtec_write_word(nvtec->client, cmd, word_value);
    } else {
        __nvtec_write_block(nvtec->client, cmd, len, value);
    }
    mutex_unlock(&nvtec->lock);
    
    return count;

}

static uint8_t g_byte;
static uint16_t g_word;
static uint8_t g_value[32];
static int read_type = 0;

static ssize_t nvtec_cmd_read_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    char temp_buf[32];
    int i;

    if (read_type == 0)
        return strlen(buf);
    else if (read_type == -1) {
        sprintf(buf, "read byte: 0x%02x\n", g_byte);
    } else if (read_type == -2) {
        sprintf(buf, "read word: 0x%04x\n", g_word);
    } else {
        sprintf(temp_buf, "read block:\n");
        strcat(buf, temp_buf);
        
        for (i = 0; i <= read_type; i ++) {
            sprintf(temp_buf, "value[%d]=0x%02x\n", i, g_value[i]);
            strcat(buf, temp_buf);
        }
    }
             
    return strlen(buf);
}

static ssize_t nvtec_cmd_read_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    char *cp, *endp, *next;
    int cmd;
    int i = 0;

    next = (char *)buf;
    cp = strsep(&next, " ");
    if (NULL == cp)
        return count;
    cmd = simple_strtoul(cp, &endp, 16);
    if (cp == endp || *endp != '\0') {
        return count;
    }

    cmd &= 0xff;
    printk("cmd: 0x%02x\n", cmd);

    cp = strsep(&next, " ");

    mutex_lock(&nvtec->lock);
    if (!strncmp(cp, "b", 1)) {
        __nvtec_read_byte(nvtec->client, cmd, &g_byte);
        printk("byte: 0x%02x\n", g_byte);
        read_type = -1;
    } else if (!strncmp(cp, "w", 1)) {
        __nvtec_read_word(nvtec->client, cmd, &g_word);
        printk("word: 0x%02x\n", g_word);
        read_type = -2;
    } else if  (!strncmp(cp, "B", 1)) {
        __nvtec_read_block(nvtec->client, cmd, 32, g_value);
        for (i = 0; i <= g_value[0]; i ++)
            printk("value[%d]: 0x%02x\n", i, g_value[i]);
        read_type = g_value[0];
    }
    mutex_unlock(&nvtec->lock);

    return count;
}

static ssize_t nvtec_hdmi_p1_driver_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    char *cp, *endp;
    int strength;

    cp = (char *)buf;
    strength = simple_strtoul(cp, &endp, 10);
    if (cp == endp) {
        return count;
    }

    if (strength > 3)
        return count;

    nvtec_set_hdmi_p1_strength(nvtec->client, strength);
    
    return count;
}

static ssize_t nvtec_hdmi_p2_driver_store(struct device *dev, 
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    char *cp, *endp;
    int strength;
    
    cp = (char *)buf;
    strength = simple_strtoul(cp, &endp, 10);
    if (cp == endp) {
        return count;
    }

    if (strength > 3)
        return count;

    nvtec_set_hdmi_p2_strength(nvtec->client, strength);
    
    return count;
}

/* Daniel Wang */
static ssize_t nvtec_upgrade_show(struct device *dev,
                                struct device_attribute *attr,
				char *buf)
{
    return sprintf(buf, "%d\n", percentage);
}

static ssize_t nvtec_fs_upgrade_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
    return sprintf(buf, "%d\n", data_flash_return);
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, nvtec_state_show, nvtec_state_store);
static DEVICE_ATTR(version, S_IRUGO, nvtec_version_show, NULL);
static DEVICE_ATTR(flash_id, S_IRUGO, nvtec_flash_id_show, NULL);
static DEVICE_ATTR(hdmi_sw, S_IWUSR | S_IRUGO, nvtec_hdmi_switch_show, nvtec_hdmi_switch_store);
static DEVICE_ATTR(gpio_in, S_IRUGO, nvtec_gpio_input_show, NULL);
static DEVICE_ATTR(gpio_out, S_IWUSR | S_IRUGO, nvtec_gpio_output_show, nvtec_gpio_output_store);
static DEVICE_ATTR(cmd_write, S_IWUSR, NULL, nvtec_cmd_write_store);
static DEVICE_ATTR(cmd_read, S_IWUSR | S_IRUGO, nvtec_cmd_read_show, nvtec_cmd_read_store);
static DEVICE_ATTR(hdmi_p1, S_IWUSR, NULL, nvtec_hdmi_p1_driver_store);
static DEVICE_ATTR(hdmi_p2, S_IWUSR, NULL, nvtec_hdmi_p2_driver_store);
static DEVICE_ATTR(up_state, S_IRUGO, nvtec_upgrade_show,NULL);		/* Daniel Wang */
static DEVICE_ATTR(fs_state, S_IRUGO, nvtec_fs_upgrade_show,NULL);	/* Daniel Wang */

static int nvtec_add_sysfs_entry(void)
{
    int ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_state);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_version);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_flash_id);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_hdmi_sw);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_gpio_in);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_gpio_out);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_cmd_write);
    if (ret < 0)
        return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_cmd_read);
    if (ret < 0)
        return ret;
    
    ret = device_create_file(nvtec_dev.this_device, &dev_attr_hdmi_p1);
    if (ret < 0)
        return ret;
    
    ret = device_create_file(nvtec_dev.this_device, &dev_attr_hdmi_p2);
    if (ret < 0)
        return ret;

    /* Daniel Wang */
    ret = device_create_file(nvtec_dev.this_device, &dev_attr_up_state);
    if (ret < 0)
    	return ret;

    ret = device_create_file(nvtec_dev.this_device, &dev_attr_fs_state);
    if (ret < 0)
    	return ret;

    return 0;

}

static void nvtec_remove_sysfs_entry(void)
{
    device_remove_file(nvtec_dev.this_device, &dev_attr_state);
    device_remove_file(nvtec_dev.this_device, &dev_attr_version);
    device_remove_file(nvtec_dev.this_device, &dev_attr_flash_id);
    device_remove_file(nvtec_dev.this_device, &dev_attr_hdmi_sw);
    device_remove_file(nvtec_dev.this_device, &dev_attr_gpio_in);
    device_remove_file(nvtec_dev.this_device, &dev_attr_gpio_out);
    device_remove_file(nvtec_dev.this_device, &dev_attr_cmd_write);
    device_remove_file(nvtec_dev.this_device, &dev_attr_cmd_read);
    device_remove_file(nvtec_dev.this_device, &dev_attr_hdmi_p1);
    device_remove_file(nvtec_dev.this_device, &dev_attr_hdmi_p2);
    device_remove_file(nvtec_dev.this_device, &dev_attr_up_state);	/* Daniel Wang */
    device_remove_file(nvtec_dev.this_device, &dev_attr_fs_state);	/* Daniel Wang */
}


static int __devinit nvtec_i2c_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct nvtec_platform_data *pdata = client->dev.platform_data;
    struct nvtec *ec;
    uint16_t event;
    char version[32];
    char lock_status[16];
    int ret = -EINVAL;

    if (!pdata) {
        dev_err(&client->dev, "nvtec requires platform data\n");
        return -ENOTSUPP;
    }

    /* Daniel Wang add for check micro-p exist or not */
    ret = __nvtec_read_block(client, 0xC0, 32, version);
    if (ret < 0) {
	dev_err(&client->dev, "Unknown EC firmware version: %d\n", ret);
	return -ENODEV;
    } else {
	dev_info(&client->dev, "EC version: %s\n", version);
    }

    ec = kzalloc(sizeof(struct nvtec), GFP_KERNEL);
    if (NULL == ec)
        return -ENOMEM;

    ec->client = client;
    ec->dev = &client->dev;
    i2c_set_clientdata(client, ec);

    mutex_init(&ec->lock);
    init_waitqueue_head(&ec->wq_ack);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->undock_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->system_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->battery_notifier);
    BLOCKING_INIT_NOTIFIER_HEAD(&ec->cir_notifier);

    /* waiting lock hardware enable notify*/
    ec->switch_gpio_notifier.notifier_call = switch_gpio_notify;
    switch_gpio_lock_hardware_notifier_register(&ec->switch_gpio_notifier);
    
    if (client->irq) {
        ret = nvtec_irq_init(ec, client->irq);
        if (ret) {
            dev_err(&client->dev, "IRQ init failed: %d\n", ret);
            goto err_irq_init;
        }
    }

    /* Daniel Wang */
    ec->request_pin = pdata->request_pin;
    ec->pwr_state_pin = pdata->pwr_state_pin;
    if ((machine_is_avalon()) && (pdata->hdmi_sw_pin != 0))
	ec->hdmi_sw_pin = pdata->hdmi_sw_pin; 	

    gpio_set_value(ec->request_pin, 1);
    gpio_set_value(ec->pwr_state_pin, 1);
    if ((machine_is_avalon()) && (ec->hdmi_sw_pin != 0))
	gpio_set_value(ec->hdmi_sw_pin, 1);

    ec->ap_wake_pin = pdata->ap_wake_pin;

    /* clean event */
    while (gpio_get_value(ec->ap_wake_pin) == 0) {
        ret = nvtec_read_event(ec->client, &event);
        if (ret < 0 || event == 0)
            break;
    }

    /*
    nvtec_set_hdmi_sw_mode(client, NVTEC_HDMI_SW_AUTO);
    */

    ret = nvtec_add_subdevs(ec, pdata);
    if (ret) {
        dev_err(&client->dev, "add devices failed: %d\n", ret);
        goto err_add_devs;
    }

    ret = misc_register(&nvtec_dev);
    if (ret) {
        dev_err(&client->dev, "fail to register misc device: %d\n", ret);
        goto err_misc_reg;
    }

    ret = nvtec_add_sysfs_entry();
    if (ret) {
        dev_err(&client->dev, "fail to add sysfs entry: %d\n", ret);
        goto err_add_sysfs;
    }

    /* Daniel Wang */
    wake_lock_init(&upgrade_lock, WAKE_LOCK_SUSPEND, "NvUpgradeSuspendLock");

    nvtec = ec;

    /* Daniel Wang */
    /*
    ret = nvtec_set_led_mode(client,default_mcu_mode ? 0x0 : 0x200);
    if (ret < 0){ 
	dev_err(&client->dev, "configure suspend led failed: %d\n", ret);
    }
    */

    nvtec_get_lock_key_status(client, lock_status);

    return 0;
 err_add_sysfs:
    misc_deregister(&nvtec_dev);
 err_misc_reg:
    nvtec_remove_subdevs(ec);
 err_add_devs:
    if (client->irq)
        free_irq(client->irq, ec);
 err_irq_init:
    switch_gpio_lock_hardware_notifier_unregister(&ec->switch_gpio_notifier);
    kfree(ec);
    return ret;
}

static int __devexit nvtec_i2c_remove(struct i2c_client *client)
{
    struct nvtec *ec = i2c_get_clientdata(client);

    wake_lock_destroy(&upgrade_lock);	/* Daniel Wang */

    gpio_free(ec->request_pin);
    gpio_free(ec->pwr_state_pin);	/* Daniel Wang */
    gpio_free(ec->ap_wake_pin);

    switch_gpio_lock_hardware_notifier_unregister(&ec->switch_gpio_notifier);

    if (client->irq)
        free_irq(client->irq, ec);

    kfree(ec);

    nvtec_remove_sysfs_entry();

    misc_deregister(&nvtec_dev);

    return 0;
}

static int nvtec_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    int ret;

    ret = nvtec_set_state(client, NVTEC_STATE_S3);
    if (ret < 0)
        return -EACCES;

    return 0;
}

static int nvtec_i2c_resume(struct i2c_client *client)
{
    int ret;

    ret = nvtec_set_state(client, NVTEC_STATE_S0);
    if (ret < 0)
        return -EACCES;

    return 0;
}

static void nvtec_i2c_shutdown(struct i2c_client *client)
{
    if (client->irq)
        disable_irq(client->irq);
    
    nvtec_set_state(client, NVTEC_STATE_S4);	/* Daniel Wang */
}

static const struct i2c_device_id nvtec_id_table[] = {
    { "nvtec", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, nvtec_id_table);

static struct i2c_driver nvtec_driver = {
    .driver = {
        .name = "nvtec",
        .owner = THIS_MODULE,
    },
    .probe  = nvtec_i2c_probe,
    .remove = nvtec_i2c_remove,
    .suspend = nvtec_i2c_suspend,
    .resume = nvtec_i2c_resume,
    .shutdown = nvtec_i2c_shutdown,
    .id_table = nvtec_id_table,
};

static int __init nvtec_init(void)
{
    return i2c_add_driver(&nvtec_driver);
}
subsys_initcall(nvtec_init);

static void __exit nvtec_exit(void)
{
    i2c_del_driver(&nvtec_driver);
}
module_exit(nvtec_exit);

MODULE_DESCRIPTION("Nuvoton EC core driver");
MODULE_AUTHOR("Ron Lee <ron1_lee@pegatroncorp.com>");
MODULE_LICENSE("GPL");
