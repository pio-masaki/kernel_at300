/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2009 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifdef 	IMMVIBESPIAPI
#undef 	IMMVIBESPIAPI
#endif
#define 	IMMVIBESPIAPI static

#include <linux/module.h> 
#include <linux/kernel.h>
#include <linux/init.h> 
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/kthread.h> 
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <../drivers/staging/android/timed_output.h>  /* add for type vibrator command */
#include <linux/ast_dock.h>
#include <mach/ast_tspdrv_pdata.h>

/* This SPI supports only one actuator. */
#define 	NUM_ACTUATORS 	1

struct pwm_vibrator {
    int vibrator_en_pin;
	struct pwm_device *pwm;
	unsigned long duty_cycle;
	unsigned long pwm_period;
	unsigned long	timeout;
    struct notifier_block dock_notifier;
    struct regulator *reg;
    bool on_dock;
    bool amp_enabled;
};

static int pwm_duty_val=0;

static struct pwm_vibrator *haptic_vibrator = NULL;

static struct pwm_vibrator *pwm_vib = NULL;
struct pwm_vibrator *get_pwm_vib(void)
{
	return pwm_vib;
}
EXPORT_SYMBOL(get_pwm_vib);

static __inline void vibrator_enable(void)
{
    gpio_direction_output(haptic_vibrator->vibrator_en_pin, 1);
    haptic_vibrator->amp_enabled = true;
}

static __inline void vibrator_disable(void)
{
    gpio_direction_output(haptic_vibrator->vibrator_en_pin, 0);
    haptic_vibrator->amp_enabled = false;
}

static void vibrator_enable_pwm(void)
{
    unsigned int	duty_cycle;

    duty_cycle = haptic_vibrator->pwm_period >> 1; // 50% duty
    pwm_config(haptic_vibrator->pwm, duty_cycle, 
               haptic_vibrator->pwm_period);
    pwm_enable(haptic_vibrator->pwm);
}

static __inline void vibrator_disable_pwm(void)
{
    pwm_disable(haptic_vibrator->pwm);
}

static void ast_vibrator_enable(struct timed_output_dev *dev, int value)
{
    haptic_vibrator->timeout = value;

	if (IS_ERR(haptic_vibrator->pwm)) {
		printk("haptic_vibrator->pwm error\n");
		return ;
	}

	if(pwm_duty_val != 1)
	{
		haptic_vibrator->duty_cycle=pwm_duty_val;
	}

    if (haptic_vibrator->on_dock == false) {
       	pwm_config(haptic_vibrator->pwm, haptic_vibrator->duty_cycle, 
                   haptic_vibrator->pwm_period);
		pwm_enable(haptic_vibrator->pwm);
        vibrator_enable();
		msleep(value);
    }

    vibrator_disable();
    vibrator_disable_pwm();

    return;
}

static int ast_vibrator_get_time(struct timed_output_dev *dev)
{
	return haptic_vibrator->timeout;
}

static ssize_t haptic_duty_store(struct device *dev, 
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count)
{
	pwm_duty_val=simple_strtol(buf, '\0', 10);
    printk("pwm duty = %d\n", pwm_duty_val);
    if (pwm_duty_val < 0)
	{
        return 0;
	}
    
    return count;
}

static ssize_t haptic_duty_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    return sprintf(buf, "%d\n", pwm_duty_val);
}

static DEVICE_ATTR(haptic_duty, S_IWUSR | S_IRUGO, haptic_duty_show, haptic_duty_store);

static struct timed_output_dev ast_vibrator = {
    .name       = "vibrator",
    .get_time   = ast_vibrator_get_time,
    .enable     = ast_vibrator_enable,
};

static int dock_notify(struct notifier_block *nb, unsigned long state, void *unused)
{
    if (state) {                /* dock inserted, disable vibrator */
        haptic_vibrator->on_dock = true;
        vibrator_disable();
        vibrator_disable_pwm();
    } else {                    /* dock removed*/
        haptic_vibrator->on_dock = false;
    }
    
    return NOTIFY_OK;
}

static int vibrator_init(struct platform_device *pdev)
{
    struct ast_tspdrv_platform_data *pdata = pdev->dev.platform_data;
	int	ret = 0;
	
	haptic_vibrator = kzalloc(sizeof(*haptic_vibrator), GFP_KERNEL);
	if (!haptic_vibrator)
	{
		printk( "[ImmVibeSPI][%s] fail to kzalloc\n", __func__ );
		return -ENOMEM;
	}

    haptic_vibrator->vibrator_en_pin = pdata->vibrator_en_pin;
    haptic_vibrator->pwm_period= pdata->pwm_period;
    haptic_vibrator->duty_cycle= pdata->pwm_duty_cycle;

    if (pdata->rail_name != NULL) {
        haptic_vibrator->reg = regulator_get(NULL, pdata->rail_name);
        if (IS_ERR_OR_NULL(haptic_vibrator->reg)) {
            printk("%s regulator_get() fail!!\n", pdata->rail_name);
        } else { 
            ret = regulator_enable(haptic_vibrator->reg);
            if (ret) {
                regulator_put(haptic_vibrator->reg);
                kfree(haptic_vibrator);
                return -ENODEV;
            }
        }
    }
	
	haptic_vibrator->pwm = pwm_request(pdata->pwm_id, "vibrator");
	if (IS_ERR(haptic_vibrator->pwm)) {
		printk("immVibe Failed to request pwm vibrator device\n");
		ret = -ENODEV;
        goto __init_pwm_request_error;
	}

	pwm_vib = haptic_vibrator;
	
	/* added for type vibrator command */
	ret = timed_output_dev_register(&ast_vibrator);
	if (ret) {
		printk("could not register Haptic vibrator device\n");
        ret = -ENODEV;
		goto __init_vibrator_error;

	} else {
		printk("Register Haptic vibrator ok\n");
	}
    
    haptic_vibrator->dock_notifier.notifier_call = dock_notify;
    ast_dock_notifier_register(&haptic_vibrator->dock_notifier);

	ret = device_create_file(ast_vibrator.dev, &dev_attr_haptic_duty);
	if (ret < 0)
		return ret;
	
	return 0;

 __init_vibrator_error:
    pwm_free(haptic_vibrator->pwm);
 __init_pwm_request_error:
    if (haptic_vibrator->reg != NULL) {
        regulator_disable(haptic_vibrator->reg);
        regulator_put(haptic_vibrator->reg);
    }

    kfree(haptic_vibrator);

 	return ret;
}

static void vibrator_free(struct platform_device *pdev)
{
    /* struct ast_tspdrv_platform_data *pdata =  */
    /*     (struct ast_tspdev_platform_data *) pdev->dev.platform_data; */

    ast_dock_notifier_unregister(&haptic_vibrator->dock_notifier);
    timed_output_dev_unregister(&ast_vibrator);
    pwm_free(haptic_vibrator->pwm);
    if (haptic_vibrator->reg != NULL) {
        regulator_disable(haptic_vibrator->reg);
        regulator_put(haptic_vibrator->reg);
    }
    kfree(haptic_vibrator);
}


/* Called to disable amp (disable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable( VibeUInt8 nActuatorIndex )
{
    if (haptic_vibrator->amp_enabled) {
		DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable\n"));
		
        vibrator_disable();
        vibrator_disable_pwm();
	}

	return VIBE_S_SUCCESS;
}

/* Called to enable amp (enable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable( VibeUInt8 nActuatorIndex )
{
    if (haptic_vibrator->on_dock == true) {
        vibrator_disable();
        vibrator_disable_pwm();
		return VIBE_S_SUCCESS;
	}
	
    if (haptic_vibrator->amp_enabled == false) {
		DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnable\n"));
        vibrator_enable_pwm();
        vibrator_enable();
	}

	return VIBE_S_SUCCESS;
}

/* Called at initialization time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" ));

   	ImmVibeSPI_ForceOut_AmpDisable( 0 );
	
	return VIBE_S_SUCCESS;
}

/* Called at termination time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate( void )
{
 	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" ));

	ImmVibeSPI_ForceOut_AmpDisable(0);

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Set( VibeUInt8 nActuatorIndex, VibeInt8 nForce )
{
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples( VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer )
{
    VibeInt8 nForce;

	unsigned int	DutyCycle;
	/* NvU32	ReturnedPeriod; */

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1) {
                // DbgOut((KERN_ERR "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
				return VIBE_E_FAIL;
            }
            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            return VIBE_E_FAIL;
    }
	 
    //	DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set nForce =  %d ,CURRENT_TIME = %d\n", nForce , CURRENT_TIME));

	//DbgOut(( "[ImmVibeSPI] : nForce = %d\n",nForce));

    if (haptic_vibrator->on_dock == false) {
		if ( nForce == 0 ) {
			ImmVibeSPI_ForceOut_AmpDisable(1);
			DutyCycle = haptic_vibrator->pwm_period >> 1; // 50% duty
		} else {
			ImmVibeSPI_ForceOut_AmpEnable(1);
			DutyCycle=((nForce + 128) * haptic_vibrator->pwm_period) >> 8;
            //DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set DutyCycle =  %d \n", DutyCycle ));
		}
	
		pwm_config(haptic_vibrator->pwm, DutyCycle, haptic_vibrator->pwm_period);
	}

	return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency( VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue )
{
   	/* This function is not called for ERM device */
	return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName( VibeUInt8 nActuatorIndex, char *szDevName, int nSize )
{
   	return VIBE_S_SUCCESS;
}
