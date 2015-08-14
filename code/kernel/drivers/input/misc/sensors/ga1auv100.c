/* drivers/input/misc/ga1auv100.c - GA1AUV100WP v2.0.2 uv sensor and ambient light sensor driver
*
* Copyright (C) 2014-2015 Sharp Corporation
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
/*==============================================================================

EDIT HISTORY FOR FILE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.



when        who  what, where, why
----------  ---  ------------------------------------------
2015-03-04  tk   Added UV data correction function
                 Fixed ALFA BETA coefficients
                 Change UV value output unit [mW/cm2] -> [uW/cm2]
2014-04-24  ik   Initial version of alsuv driver for ga1auv100wp
==============================================================================*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/slab.h>
//#include <linux/wakelock.h>
//#include <asm/arch/irqs.h>
#include <linux/of_gpio.h>
#include "ga1auv100.h"
#include <linux/regulator/consumer.h>

#define DEVICE_NAME       "GA1AUV100WP"
#define CLASS_NAME        "ga1auv100"
#define UV_SENSOR_NAME      "ga1a_uv"
#define LIGHT_SENSOR_NAME   "ga1a_light"
#define GA1A_MIN_UV	1800000
#define GA1A_MAX_UV	1800000

#define SENSOR_DEFAULT_DELAY   ( 200 )  /* 200 ms */
#define SENSOR_MAX_DELAY      ( 5000 )  /* 5000 ms */

#define OPERATION_UV      0
#define OPERATION_ALS     1

/* #define WITH_PANEL */          /* with panel or without panel */
#define OVER_FLOW_COUNT     65000 /* less tnan < 2^(resolution) -1 */

/* UV mode change */
#define LOW_UV_MODE     ( 0 )
#define HIGH_UV_MODE    ( 1 )
#define LOW_UV_RANGE    ( COMMAND3_RANGEX4 )
#define HIGH_UV_RANGE   ( COMMAND3_RANGEX128 )
/* coefficient for adjusting the difference in RANGE */
#define UV_L_to_H_counts    64000   /* less tnan < 2^(resolution) -1 */
#define UV_H_to_L_counts    1000    /* less than ( UV_L_to_H_counts / GAMMA_HIGH_UV_MODE) */

/* ALS mode change */
#define LOW_LUX_MODE    ( 0 )
#define HIGH_LUX_MODE   ( 1 )
#define LOW_LUX_RANGE   ( COMMAND3_RANGEX4 )
#define HIGH_LUX_RANGE  ( COMMAND3_RANGEX128 )
/* coefficient for adjusting the difference in RANGE */
#define ALS_L_to_H_counts   64000   /* less tnan < 2^(resolution) -1 */
#define ALS_H_to_L_counts   1000    /* less than ( ALS_L_to_H_counts/GAMMA_HIGH_UV_MODE) */


struct ga1a_data
{
  struct mutex          mutex ;
  struct i2c_client     *client ;
  struct class          *dev_class ;

  /* UV&ALS concurrent mode by software */
  int                   now_operation;

  /* UV */
  u8                    regUV[4] ;
  struct input_dev      *uv_input_dev ;
  struct delayed_work   uv_work ; 
  int                   uv_enabled ;
  int                   uv_mode ;
  int                   uv_delay ;
  int                   uv_val_prev ;
  u32                   data_uvclear ;
  u32                   data_uvcut ;

  /* ALS */
  u8                    regALS[4] ;
  struct input_dev      *als_input_dev ;
  struct delayed_work   als_work ; 
  int                   als_enabled ;
  int                   als_mode ;
  int                   als_delay ;
  int                   als_lux_prev ;
  u32                   data_alsclear ;
  u32                   data_alsir ;

	struct device dev;
	int power_enable;
} ;

static u8 ga1a_init_reg_uv[4] = {
  /* COMMAND1 shutdown */
  0x00,
  /* COMMAND2 PD:00 PIN:0 FREQ:0 NOWAIT:0 INTTYPE:0 RST:0 */
  0x00,
  /* COMMAND3 RANGE:LOW_RANGE RES:101 */
  ( LOW_UV_RANGE | COMMAND3_RES_16 ),
  /* INTERVAL INTVAL:000 */
  ( INTERVAL_0 )
} ;
static u8 ga1a_init_reg_als[4] = {
  /* COMMAND1 shutdown */
  COMMAND1_ALS,
  /* COMMAND2 PD:01 PIN:0 FREQ:0 NOWAIT:0 INTTYPE:0 RST:0 */
  COMMAND2_PD0,
  /* COMMAND3 RANGE:LOW_RANGE RES:101 */
  ( LOW_LUX_RANGE | COMMAND3_RES_16 ),
  /* INTERVAL INTVAL:000 */
  ( INTERVAL_0 )
} ;


static struct ga1a_data *_sinfo = NULL;


static void ga1a_change_operation( struct ga1a_data *data ) ;

/* *****************************************************************************
    Common
***************************************************************************** */
static char *strtok( char *s1, char *s2 )
{
  static char *str ;
  char        *start ;

  if( s1 != NULL )
  {
    str = s1 ;
  }
  start = str ;

  if( str == NULL )
  {
    return NULL ;
  }

  if( s2 != NULL )
  {
    str = strstr( str, s2 ) ;
    if( str != NULL )
    {
      *str = 0x00 ;
      str += strlen( s2 ) ;
    }
  }
  else
  {
    str = NULL ;
  }
  return start ;
}

/* *****************************************************************************
    I2C
***************************************************************************** */
static int ga1a_i2c_read( u8 reg, unsigned char *rbuf, int len, struct i2c_client *client )
{

  int               err = -1 ;
  struct i2c_msg    i2cMsg[2] ;
  uint8_t       buff ;

  if( client == NULL )
  {
    return -ENODEV ;
  }

  i2cMsg[0].addr = client->addr ;
  i2cMsg[0].flags = 0 ;
  i2cMsg[0].len = 1 ;
  i2cMsg[0].buf = &buff ;
  buff = reg ;
  i2cMsg[1].addr = client->addr ;
  i2cMsg[1].flags = I2C_M_RD ;
  i2cMsg[1].len = len ;
  i2cMsg[1].buf = rbuf ;

  err = i2c_transfer( client->adapter, &i2cMsg[0], 2 ) ;
/*
  if( err >= 0 )
  {
    i2cMsg.flags = I2C_M_RD ;
    i2cMsg.len = len ;
    i2cMsg.buf = rbuf ;
    err = i2c_transfer( client->adapter, &i2cMsg, 1 ) ;
  }
*/
  if( err < 0 )
  {
    printk( KERN_ERR "i2c transfer error(%d)!!\n", err ) ;
  }

  return err ;
}

static int ga1a_i2c_write( u8 reg, u8 *wbuf, struct i2c_client *client )
{
  int               err = 0 ;
  struct i2c_msg    i2cMsg ;
  unsigned char   buff[2] ;
  int         retry = 10 ;

  if( client == NULL )
  {
    return -ENODEV ;
  }

  while( retry-- )
  {
    buff[0] = reg ;
    buff[1] = *wbuf ;

    i2cMsg.addr = client->addr ;
    i2cMsg.flags = 0 ;
    i2cMsg.len = 2 ;
    i2cMsg.buf = buff ;

    err = i2c_transfer( client->adapter, &i2cMsg, 1 ) ;
    pr_debug( "ga1a_i2c_write : 0x%02x, 0x%02x\n", reg, *wbuf ) ;

    if( err >= 0 )
    {
      return 0 ;
    }
  }
  printk( KERN_ERR "i2c transfer error(%d)!!\n", err ) ;

  return err ;
}

static int ga1a_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
  struct ga1a_data                *data ;
//  struct ga1auv100_platform_data  *pdata ;

  pr_info( "ga1a_i2c_probe \n" ) ;

  data = _sinfo ;
  data->client = client ;
//  pdata = client->dev.platform_data ;

  pr_info( "ga1a i2c attach success!!!\n" ) ;

  return 0 ;
}

static int ga1a_i2c_remove( struct i2c_client *client )
{
  struct ga1a_data  *data ;

  pr_debug( "ga1a_i2c_remove \n" ) ;

  data = _sinfo ;
  data->client = NULL ;

  return 0 ;
}

static const struct i2c_device_id ga1a_device_id[] =
{
  { "ga1auv100", 0 },
  { }
} ;
MODULE_DEVICE_TABLE( i2c, ga1a_device_id ) ;

static const struct of_device_id  ga1a_i2c_dt_match[] = {
	{.compatible = "sharp, ga1auv100"},
	{}
};

MODULE_DEVICE_TABLE(of, ga1a_i2c_dt_match)

static struct i2c_driver ga1a_i2c_driver =
{
  .driver = {
    .name = "ga1auv100",
    .owner= THIS_MODULE,
    .of_match_table = ga1a_i2c_dt_match,
  },
  .probe    = ga1a_i2c_probe,
  .remove   = ga1a_i2c_remove,
  .id_table = ga1a_device_id,
} ;

static int ga1a_i2c_init( void )
{
  pr_info("gaia i2c init");
  if( i2c_add_driver( &ga1a_i2c_driver ) )
  {
    printk( KERN_ERR "i2c_add_driver failed \n" ) ;
    return -ENODEV ;
  }
  return 0 ;
}


/* *****************************************************************************
    Light Sensor
***************************************************************************** */
static ssize_t
als_delay_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  int  delay ;

  pr_debug( "als_delay_show \n" ) ;

  mutex_lock( &data->mutex ) ;
  delay = data->als_delay ;
  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%d", delay ) ;
}

static ssize_t
als_delay_store( struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  char        *endp ;
  int         delay = simple_strtoul( buf, &endp, 10 ) ;
  int         enable ;
  int         old_delay ;

  pr_debug( "als_delay_store data=%s\n", buf ) ;

  if( !( endp != buf &&
      ( endp == ( buf + strlen( buf ) )
        || ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid delay (%s)\n", buf ) ;
    return count ;
  }
  delay = delay / 1000000 ;/* time unit conversion from [nsec] to [msec] */
  if( delay < 0 || SENSOR_MAX_DELAY < delay )
  {
    printk( KERN_ERR "invalid delay (%s)\n", buf ) ;
    return count ;
  }

  if( data->als_enabled && data->now_operation==OPERATION_ALS )
  {
    cancel_delayed_work_sync( &data->als_work ) ;
    schedule_delayed_work( &data->als_work, msecs_to_jiffies( delay ) ) ;
  }

  mutex_lock( &data->mutex ) ;

  old_delay = data->als_delay ;
  data->als_delay = delay ;
  enable = data->als_enabled ;

  mutex_unlock( &data->mutex ) ;

  if( old_delay != delay )
  {
    input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enable << 16) | delay ) ;
  }

  return count ;
}

static int als_onoff( u8 onoff, struct ga1a_data *data )
{
  //u8    value ;

  pr_debug( "light_sensor onoff = %d\n", onoff ) ;

  if( onoff )
  {
    ga1a_change_operation( data ) ;
  }
  else
  {
    if( !data->uv_enabled )
    {
      data->regALS[REG_COMMAND1] = ( COMMAND1_SD | COMMAND1_ALS ) ; // shutdown
      ga1a_i2c_write( REG_COMMAND1, &( data->regALS[REG_COMMAND1] ), data->client ) ;
    }
    else
    {
      data->now_operation = OPERATION_UV ;
      ga1a_change_operation( data ) ;
    }
  }
  return 0 ;
}

static unsigned int
als_mode_change(  u32 *data_als, struct ga1a_data *data )
{
//  struct gp2ap052a_als_priv *obj = i2c_get_clientdata(client);   
  u8    value ;

  /*  Lux mode (Range) change */
  if( ( data_als[0] >= ALS_L_to_H_counts ) && ( data->als_mode == LOW_LUX_MODE ) )
  {
    data->als_mode = HIGH_LUX_MODE ;
    
    pr_debug( "change lux mode high!! \n" ) ;
    
    value = ( COMMAND1_SD ) ;                           // Shutdown
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;
    if( !data->uv_enabled )
    {
      /* change Range of ALS */
      data->regALS[REG_COMMAND3] = ( HIGH_LUX_RANGE | ( data->regALS[REG_COMMAND3] & 0x07 ) );
      ga1a_i2c_write( REG_COMMAND3, &( data->regALS[REG_COMMAND3] ), data->client ) ;
      data->regALS[REG_COMMAND1] = ( COMMAND1_WAKEUP | COMMAND1_ALS ) ;   // ALS mode
      ga1a_i2c_write( REG_COMMAND1, &( data->regALS[REG_COMMAND1] ), data->client ) ;
    }

    return 1 ;

  }
  else if( ( data_als[0] < ALS_H_to_L_counts ) && ( data->als_mode == HIGH_LUX_MODE ) )
  {
    data->als_mode = LOW_LUX_MODE ;
    
    pr_debug( "change lux mode low!! \n" ) ;
    
    value = ( COMMAND1_SD ) ;                           // Shutdown
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;
    
    if( !data->uv_enabled )
    {
      /* change Range of ALS */
      data->regALS[REG_COMMAND3] = ( LOW_LUX_RANGE | ( data->regALS[REG_COMMAND3] & 0x07 ) );
      ga1a_i2c_write( REG_COMMAND3, &( data->regALS[REG_COMMAND3] ), data->client ) ;
      data->regALS[REG_COMMAND1] = ( COMMAND1_WAKEUP | COMMAND1_ALS ) ;   // ALS mode
      ga1a_i2c_write( REG_COMMAND1, &( data->regALS[REG_COMMAND1] ), data->client ) ;
    }
    
    return 1 ;

  }

  return 0 ;

}

static ssize_t
als_enable_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  int  enabled ;

  pr_debug( "als_enable_show \n" ) ;

  mutex_lock( &data->mutex ) ;
  enabled = data->als_enabled ;
  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%d", enabled ) ;
}

static ssize_t
als_enable_store( struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  char *endp ;
  int  enabled = simple_strtoul( buf, &endp, 10 ) ;

  pr_debug( "als_enable_store data=%s\n", buf ) ;

  if( !( endp != buf &&
      ( endp == ( buf + strlen( buf ) )
        || ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }
  if( enabled != 0 && enabled != 1 )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  //printk( KERN_INFO "Before enable/disable uv_enable:%d, als_enable:%d, now:%d \n",
  //        data->uv_enabled, data->als_enabled, data->now_operation) ;

  if ( data->uv_enabled && data->now_operation==OPERATION_UV )
    cancel_delayed_work_sync( &data->uv_work ) ;
  
  if( data->als_enabled && !enabled )
  {
    if ( data->now_operation==OPERATION_ALS )
          cancel_delayed_work_sync( &data->als_work ) ;
      mutex_lock( &data->mutex ) ;
      data->als_mode = LOW_LUX_MODE ;
      als_onoff( 0, data ) ;
      data->als_enabled = enabled ;
      mutex_unlock( &data->mutex ) ;
      printk( KERN_INFO "light_sensor disable!! \n" ) ;
      input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->als_delay ) ;
    if ( data->uv_enabled )
    {
      msleep( 150 ) ;
      schedule_delayed_work( &data->uv_work, 0 ) ;
      //input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->uv_delay ) ;
    }
  }
  else if( !data->als_enabled && enabled )
  {
    mutex_lock( &data->mutex ) ;
    data->now_operation = OPERATION_ALS;
    als_onoff( 1, data ) ;
    data->als_enabled = enabled ;
    mutex_unlock( &data->mutex ) ;
    msleep( 150 ) ;
    schedule_delayed_work( &data->als_work, 0 ) ;
    printk( KERN_INFO "light_sensor enable!! \n" ) ;
    input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->als_delay ) ;
  }
  //printk( KERN_INFO "After enable/disable uv_enable:%d, als_enable:%d, now:%d \n",
  //        data->uv_enabled, data->als_enabled, data->now_operation) ;

  return count ;
}




static ssize_t
als_raw_data_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  u32         data_clear ;
  u32         data_ir ;
  u8          rdata[4] ;
  u32         ratio ;
  int         mode ;

  pr_debug( "als_raw_data_show \n" ) ;

  mutex_lock( &data->mutex ) ;

  if( data->als_enabled && data->now_operation==OPERATION_ALS )
  {
    ga1a_i2c_read( REG_D0_LSB, rdata, sizeof( rdata ), data->client ) ;
    data_clear = rdata[0]+rdata[1]*256;
    data_ir = rdata[2]+rdata[3]*256;
  }
  else
  {
    data_clear = data->data_alsclear;
    data_ir = data->data_alsir;
  }

  if( data_clear == 0 )
  {
    ratio = 100 ;
  }
  else
  {
    ratio = ( data_ir * 100 ) / data_clear ;
  }

  mode = data->als_mode ;

  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%d,%d,%d.%02d,%d", data_clear, data_ir, ( ratio / 100 ), ( ratio % 100 ), mode ) ;
}

static ssize_t
als_setting_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  u8          rdata[4] ;
  int         i ;

  pr_debug( "als_setting_show \n" ) ;

  mutex_lock( &data->mutex ) ;

  for( i = 0 ; i < sizeof( rdata ) ; i++ )
    rdata[i] = data->regALS[i] ;

  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%02x,%02x,%02x,%02x",
            rdata[0], rdata[1], rdata[2], rdata[3] ) ;
}

static ssize_t
als_setting_store( struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  char        *reg_p ;
  char        *value_p ;
  u32         reg ;
  u32         value ;
  char        tmpbuf[8] ;
  char        *endp ;
  //u8          temp ;

  pr_debug( "als_settign_store data=%s\n", buf ) ;

  if( strlen( buf ) > 7 )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  memset( tmpbuf, 0x00, sizeof( tmpbuf ) ) ;
  memcpy( tmpbuf, buf, strlen( buf ) ) ;

  reg_p = strtok( tmpbuf, "," ) ;
  value_p = strtok( NULL, NULL ) ;

  if( reg_p == NULL || value_p == NULL )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  reg = simple_strtoul( reg_p, &endp, 16 ) ;
  if( !( endp != reg_p &&
      ( endp == ( reg_p + strlen( reg_p ) )
        || ( endp == ( reg_p + strlen( reg_p ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }
  value = simple_strtoul( value_p, &endp, 16 ) ;
  if( !( endp != value_p &&
      ( endp == ( value_p + strlen( value_p ) )
        || ( endp == ( value_p + strlen( value_p ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  if( reg <= 0x03 && value <= 255 )
  {
    pr_debug( "  reg=%02x,value=%02x\n", reg, value ) ;
    mutex_lock( &data->mutex ) ;
    data->regALS[reg] = ( u8 )value ;
    if( !data->uv_enabled )
      ga1a_i2c_write( ( u8 )reg, &( data->regALS[reg] ), data->client ) ;
    mutex_unlock( &data->mutex ) ;
  }
  else
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
  }

  return count ;

}
static DEVICE_ATTR( als_delay, S_IRUGO|S_IWUGO, als_delay_show, als_delay_store ) ;
static DEVICE_ATTR( als_enable, S_IRUGO|S_IWUGO, als_enable_show, als_enable_store ) ;
static DEVICE_ATTR( als_raw_data, S_IRUGO, als_raw_data_show, NULL ) ;
static DEVICE_ATTR( als_setting, S_IRUGO|S_IWUGO, als_setting_show, als_setting_store ) ;

static struct attribute *als_attributes[] =
{
  &dev_attr_als_delay.attr,
  &dev_attr_als_enable.attr,
  &dev_attr_als_raw_data.attr,
  &dev_attr_als_setting.attr,
  NULL
} ;

static struct attribute_group als_attribute_group =
{
  .attrs = als_attributes
} ;

static int als_input_init( struct ga1a_data *data )
{
  struct input_dev   *dev ;
  int         err = 0 ;

  dev = input_allocate_device( ) ;
  if( !dev )
  {
    printk( KERN_ERR "%s, input_allocate_device error(%d)!!\n", __func__, err ) ;
    return -ENOMEM ;
  }

  set_bit( EV_ABS, dev->evbit ) ;
  input_set_capability( dev, EV_ABS, ABS_LUX_REPORT ) ;
  input_set_abs_params( dev, ABS_LUX_REPORT, 0, 0x7fffffff, 0, 0 ) ;
  input_set_capability( dev, EV_ABS, ABS_WAKE ) ;
  input_set_abs_params( dev, ABS_WAKE, 0, 0x7fffffff, 0, 0 ) ;
  input_set_capability( dev, EV_ABS, ABS_CONTROL_REPORT ) ;
  input_set_abs_params( dev, ABS_CONTROL_REPORT, 0, 0x1ffff, 0, 0 ) ;

  dev->name = LIGHT_SENSOR_NAME ;

  err = input_register_device( dev ) ;
  if( err < 0 )
  {
    input_free_device( dev ) ;
    printk( KERN_ERR "%s, input_register_device error(%d)!!\n", __func__, err ) ;
    return err ;
  }
  input_set_drvdata( dev, data ) ;

  data->als_input_dev = dev ;

  return 0 ;
}

/* *****************************************************************************
    ALS DATA POLLING
*******************************************************************************/
static void als_data_polling( struct work_struct *work )
{
  struct ga1a_data  *data = container_of( ( struct delayed_work * )work,
                            struct ga1a_data, als_work ) ;
//  unsigned int    lux ;
  u8          rdata[4] ;
  u32         data_als[2];
  int         ret;
  

  if( data != NULL )
  {
    mutex_lock( &data->mutex ) ;

    /* confirm validity before reading data */
    ga1a_i2c_read( REG_COMMAND1, rdata, 1, data->client ) ;
    if( ( rdata[0] & 0x12 ) != 0x12 ) /* Check operation ALS and Flag assertion */
    {
      /* suspect that data are read out before the sensor finishes measuring 
      * when FLAG bit isn't asserted 
      */
      printk( KERN_INFO "Operation isn't ALS mode or FLAG isn't asserted \n" ) ;
    }
    else
    {
      /* read data */
      ga1a_i2c_read( REG_D0_LSB, rdata, sizeof( rdata ), data->client ) ;
      data_als[0] = rdata[0]+rdata[1]*256;
      data_als[1] = rdata[2]+rdata[3]*256;
      pr_debug( "read sensor clear=%d,ir=%d\n", data_als[0], data_als[1] ) ;
    
      ret = als_mode_change(data_als, data);

      mutex_unlock( &data->mutex ) ;

      if(ret == 0)
      {
        //input_report_abs( data->als_input_dev, ABS_LUX_REPORT, lux ) ;
        input_report_abs( data->als_input_dev, ABS_LUX_REPORT, ( data->als_mode << 17 ) | ( 0x0 << 16 ) | data_als[0] ) ;
        input_report_abs( data->als_input_dev, ABS_LUX_REPORT, ( data->als_mode << 17 ) | ( 0x1 << 16 ) | data_als[1] ) ;
        input_sync( data->als_input_dev ) ;
      }

      if ( data->uv_enabled ) 
      {
        mutex_lock( &data->mutex ) ;
        data->now_operation = OPERATION_UV;
        ga1a_change_operation(data);
        mutex_unlock( &data->mutex ) ;
        schedule_delayed_work( &data->uv_work, msecs_to_jiffies( data->uv_delay ) ) ;
        //printk( KERN_INFO "Operation changed in ALS polling now:%d \n", data->now_operation) ;
      }
      else if( data->als_enabled )
      {
        schedule_delayed_work( &data->als_work, msecs_to_jiffies( data->als_delay ) ) ;
      }
    }
  }
}

/* *****************************************************************************
    UV Sensor
***************************************************************************** */
static ssize_t
uv_delay_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  int  delay ;

  pr_debug( "uv_delay_show \n" ) ;

  mutex_lock( &data->mutex ) ;
  delay = data->uv_delay ;
  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%d", delay ) ;
}

static ssize_t
uv_delay_store( struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  char        *endp ;
  int         delay = simple_strtoul( buf, &endp, 10 ) ;
  int         enable ;
  int         old_delay ;

  pr_debug( "uv_delay_store data=%s\n", buf ) ;

  if( !( endp != buf &&
      ( endp == ( buf + strlen( buf ) )
        || ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid delay (%s)\n", buf ) ;
    return count ;
  }
  delay = delay / 1000000 ;/* time unit conversion from [nsec] to [msec] */
  if( delay < 0 || SENSOR_MAX_DELAY < delay )
  {
    printk( KERN_ERR "invalid delay (%s)\n", buf ) ;
    return count ;
  }

  if( data->uv_enabled && data->now_operation == OPERATION_UV )
  {
    cancel_delayed_work_sync( &data->uv_work ) ;
    schedule_delayed_work( &data->uv_work, msecs_to_jiffies( delay ) ) ;
  }

  mutex_lock( &data->mutex ) ;

  old_delay = data->uv_delay ;
  data->uv_delay = delay ;
  enable = data->uv_enabled ;

  mutex_unlock( &data->mutex ) ;

  if( old_delay != delay )
  {
    input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enable << 16) | delay ) ;
  }

  return count ;
}

static int uv_onoff( u8 onoff, struct ga1a_data *data )
{
  //u8    value ;

  pr_debug( "UV_sensor onoff = %d\n", onoff ) ;

  if( onoff )
  {
    ga1a_change_operation( data ) ;
  }
  else
  {
    if( !data->als_enabled )
    {
      data->regUV[REG_COMMAND1] = ( COMMAND1_SD ) ; // shutdown
      ga1a_i2c_write( REG_COMMAND1, &( data->regUV[REG_COMMAND1] ), data->client ) ;
    }
    else
    {
      data->now_operation = OPERATION_ALS ;
      ga1a_change_operation( data ) ;
    }
  }
  return 0 ;
}

static ssize_t
uv_enable_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  int  enabled ;

  pr_debug( "uv_enable_show \n" ) ;

  mutex_lock( &data->mutex ) ;
  enabled = data->uv_enabled ;
  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%d", enabled ) ;
}

static ssize_t
uv_enable_store( struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  char  *endp ;
  int   enabled = simple_strtoul( buf, &endp, 10 ) ;

  pr_err( "uv_enable_store data=%d\n", enabled ) ;
#if 0
  if( !( endp != buf &&
      ( endp == ( buf + strlen( buf ) )
        || ( endp == ( buf + strlen( buf ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }
#endif
  if( enabled != 0 && enabled != 1 )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  //printk( KERN_INFO "Before enable/disable uv_enable:%d, als_enable:%d, now:%d \n",
  //          data->uv_enabled, data->als_enabled, data->now_operation) ;

  if ( data->als_enabled && data->now_operation==OPERATION_ALS )
        cancel_delayed_work_sync( &data->als_work ) ;

  if( data->uv_enabled && !enabled )
  {
    if ( data->now_operation==OPERATION_UV )
      cancel_delayed_work_sync( &data->uv_work ) ;

    mutex_lock( &data->mutex ) ;
    data->uv_mode = LOW_UV_MODE ;
    uv_onoff( 0, data ) ;
    data->uv_enabled = enabled ;
    mutex_unlock( &data->mutex ) ;
    printk( KERN_INFO "UV_sensor disable!! \n" ) ;
    input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->uv_delay ) ;

    if ( data->als_enabled )
    {
      msleep( 150 ) ;
      schedule_delayed_work( &data->als_work, 0 ) ;
      //input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->als_delay ) ;
    }
  }
  else if( !data->uv_enabled && enabled )
  {
    mutex_lock( &data->mutex ) ;
    data->now_operation = OPERATION_UV ;
    uv_onoff( 1, data ) ;
    data->uv_enabled = enabled ;
    mutex_unlock( &data->mutex ) ;
    msleep( 150 ) ;
    schedule_delayed_work( &data->uv_work, 0 ) ;
    printk( KERN_INFO "UV_sensor enable!! \n" ) ;
    input_report_abs( input_dev, ABS_CONTROL_REPORT, ( enabled << 16 ) | data->uv_delay ) ;
  }
  //printk( KERN_INFO "After enable/disable uv_enable:%d, als_enable:%d, now:%d \n",
  //          data->uv_enabled, data->als_enabled, data->now_operation) ;

  return count ;
}

static unsigned int
uv_mode_change(  u32 *data_uv, struct ga1a_data *data )
{
//  struct gp2ap052a_als_priv *obj = i2c_get_clientdata(client);   
  u8    value ;

  /*  UV mode (Range) change */
  if( ( data_uv[0] >= UV_L_to_H_counts ) && ( data->uv_mode == LOW_UV_MODE ) )
  {
    data->uv_mode = HIGH_UV_MODE ;
    
    pr_debug( "change uv mode high!! \n" ) ;
    
    value = ( COMMAND1_SD ) ;                 // Shutdown
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;
    if( !data->als_enabled )
    {
      /* change Range of UV */
      data->regUV[REG_COMMAND3] = ( HIGH_UV_RANGE | ( data->regUV[REG_COMMAND3] & 0x07 ) );
      ga1a_i2c_write( REG_COMMAND3, &( data->regUV[REG_COMMAND3] ), data->client ) ;
      data->regUV[REG_COMMAND1] = ( COMMAND1_WAKEUP ) ;     // UV mode
      ga1a_i2c_write( REG_COMMAND1, &( data->regUV[REG_COMMAND1] ), data->client ) ;
    }

    return 1 ;

  }
  else if( ( data_uv[0] < UV_H_to_L_counts ) && ( data->uv_mode == HIGH_UV_MODE ) )
  {
    data->uv_mode = LOW_UV_MODE ;
    
    pr_debug( "change uv mode low!! \n" ) ;
    value = ( COMMAND1_SD ) ;                 // Shutdown
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;
    if( !data->als_enabled )
    {
      /* change Range of UV */
      data->regUV[REG_COMMAND3] = ( LOW_UV_RANGE | ( data->regUV[REG_COMMAND3] & 0x07 ) );
      ga1a_i2c_write( REG_COMMAND3, &( data->regUV[REG_COMMAND3] ), data->client ) ;
      data->regUV[REG_COMMAND1] = ( COMMAND1_WAKEUP ) ;     // UV mode
      ga1a_i2c_write( REG_COMMAND1, &( data->regUV[REG_COMMAND1] ), data->client ) ;
    }

    return 1 ;

  }

  return 0 ;
}

static void uv_data_correct(u32 *data_uv, struct ga1a_data *data )
{

  u8 value;
  u8 rdata[4];
  u32 ratio;
  u16 NOT_CORR_RATIO = 50;
  u16 NOT_CORR_LO_TH = 500;
  u16 NOT_CORR_HI_TH = 15500;
  int i = 0;
  u16 corr_d1[2]={0, 0};
  u16 corr_d2[2]={0, 0};

  /* [1] Exception of UV correction */
  /*  if D1/D0 < NOT_CORR_RATIO */
  ratio = 100*data_uv[1]/data_uv[0];
  if( ratio < NOT_CORR_RATIO || data_uv[0] == 0 )
    return;

  /* UV value correction with ALS mode */
  for(i=0; i<2; i++)
  {

    /* Reset */
    value = ( COMMAND2_RST ) ;
    ga1a_i2c_write( REG_COMMAND2, &value, data->client ) ;

    if( i == 1 )
    {
      /* ALS mode PD swap OFF */
      value = 0x10;
      ga1a_i2c_write( REG_TEST, &value, data->client ) ;
    }
    /* Setting ALS mode for UV data correction */
    value = COMMAND2_PD0 ;
    ga1a_i2c_write( REG_COMMAND2, &value, data->client ) ;
    value = COMMAND3_RANGEX256 | COMMAND3_RES_14 ;
    ga1a_i2c_write( REG_COMMAND3, &value, data->client ) ;
    value = INTERVAL_0 ;
    ga1a_i2c_write( REG_INTERVAL, &value, data->client ) ;

    /* Start */
    value = ( COMMAND1_ALS | COMMAND1_WAKEUP) ;
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;

    /* Wait until sensor data update */
    pr_debug( "UV correction wait %d start", i);
    mdelay(10); /* 10 msec */
    pr_debug( "UV correction wait %d end", i);


    /* read data */
    ga1a_i2c_read( REG_D0_LSB, rdata, sizeof( rdata ), data->client ) ;

    /* Store data for correction */
    if( i == 0)
    {
      corr_d1[0] = (u16)rdata[0] + rdata[1]*256;
      corr_d1[1] = (u16)rdata[2] + rdata[3]*256;
    }
    else
    {
      corr_d2[0] = (u16)rdata[0] + rdata[1]*256;
      corr_d2[1] = (u16)rdata[2] + rdata[3]*256;
    }

  }

  /* Re-start UV mode */
  ga1a_change_operation( data );

  /* [2] Exception of UV correction */
  /* als data : NOT_CORR_LO_TH or lower, NOT_CORR_HI_TH or higher */
  for(i=0; i<2; i++)
  {

    if( corr_d1[i] <= NOT_CORR_LO_TH ||
      corr_d1[i] >= NOT_CORR_HI_TH )
      return;
    if( corr_d2[i] <= NOT_CORR_LO_TH ||
      corr_d2[i] >= NOT_CORR_HI_TH )
      return;
  }

  pr_debug( "corr_d1[0] = %d, corr_d1[1] = %d",corr_d1[0], corr_d1[1] ) ;
  pr_debug( "corr_d2[0] = %d, corr_d2[1] = %d",corr_d2[0], corr_d2[1] ) ;

  /* UV data correction */
  for(i=0; i<2; i++)
  {
    data_uv[i] = data_uv[i] * corr_d1[i] / corr_d2[i] ;
    /* Precaution of overflow */
    if( data_uv[i] > 65535 )
      data_uv[i] = 65535;
  }
  pr_debug( "After correction: data_uv[0] = %d, data_uv[1] = %d",data_uv[0], data_uv[1] ) ;

  return;
}

static ssize_t
uv_raw_data_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  u32         data_clear ;
  u32         data_cut ;
  u8          rdata[4] ;
  u32         ratio ;
  int         mode ;

  pr_debug( "uv_raw_data_show \n" ) ;

  mutex_lock( &data->mutex ) ;

  if( data->uv_enabled && data->now_operation==OPERATION_UV )
  {
    ga1a_i2c_read( REG_D0_LSB, rdata, sizeof( rdata ), data->client ) ;
    data_clear = rdata[0]+rdata[1]*256;
    data_cut = rdata[2]+rdata[3]*256;
  }
  else
  {
    data_clear = data->data_uvclear;
    data_cut = data->data_uvcut;
  }

  if( data_clear == 0 )
  {
    ratio = 100 ;
  }
  else
  {
    ratio = ( data_cut * 100 ) / data_clear ;
  }

  mode = data->uv_mode ;

  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%d,%d,%d.%02d,%d", data_clear, data_cut, ( ratio / 100 ), ( ratio % 100 ), mode ) ;
}

static ssize_t
uv_setting_show( struct device *dev,
  struct device_attribute *attr,
  char *buf )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  u8          rdata[4] ;
  int         i ;

  pr_debug( "uv_setting_show \n" ) ;

  mutex_lock( &data->mutex ) ;

  for( i = 0 ; i < sizeof( rdata ) ; i++ )
    rdata[i] = data->regUV[i] ;

  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%02x,%02x,%02x,%02x",
            rdata[0], rdata[1], rdata[2], rdata[3] ) ;
}

static ssize_t
uv_setting_store( struct device *dev,
  struct device_attribute *attr,
  const char *buf,
  size_t count )
{
  struct input_dev  *input_dev = to_input_dev( dev ) ;
  struct ga1a_data  *data = input_get_drvdata( input_dev ) ;
  char        *reg_p ;
  char        *value_p ;
  u32         reg ;
  u32         value ;
  char        tmpbuf[8] ;
  char        *endp ;
  //u8          temp ;

  pr_debug( "uv_settign_store data=%s\n", buf ) ;

  if( strlen( buf ) > 7 )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  memset( tmpbuf, 0x00, sizeof( tmpbuf ) ) ;
  memcpy( tmpbuf, buf, strlen( buf ) ) ;

  reg_p = strtok( tmpbuf, "," ) ;
  value_p = strtok( NULL, NULL ) ;

  if( reg_p == NULL || value_p == NULL )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  reg = simple_strtoul( reg_p, &endp, 16 ) ;
  if( !( endp != reg_p &&
      ( endp == ( reg_p + strlen( reg_p ) )
        || ( endp == ( reg_p + strlen( reg_p ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }
  value = simple_strtoul( value_p, &endp, 16 ) ;
  if( !( endp != value_p &&
      ( endp == ( value_p + strlen( value_p ) )
        || ( endp == ( value_p + strlen( value_p ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  if( reg <= 0x03 && value <= 255 )
  {
    pr_debug( "  reg=%02x,value=%02x\n", reg, value ) ;
    mutex_lock( &data->mutex ) ;
    data->regUV[reg] = ( u8 )value ;
    if( !data->als_enabled )
      ga1a_i2c_write( ( u8 )reg, &( data->regUV[reg] ), data->client ) ;
    mutex_unlock( &data->mutex ) ;
  }
  else
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
  }

  return count ;

}

static DEVICE_ATTR( uv_delay, S_IRUGO|S_IWUGO, uv_delay_show, uv_delay_store ) ;
static DEVICE_ATTR( uv_enable, S_IRUGO|S_IWUGO, uv_enable_show, uv_enable_store ) ;
static DEVICE_ATTR( uv_raw_data, S_IRUGO, uv_raw_data_show, NULL ) ;
static DEVICE_ATTR( uv_setting, S_IRUGO|S_IWUGO, uv_setting_show, uv_setting_store ) ;

static struct attribute *uv_attributes[] =
{
  &dev_attr_uv_delay.attr,
  &dev_attr_uv_enable.attr,
  &dev_attr_uv_raw_data.attr,
  &dev_attr_uv_setting.attr,
  NULL
} ;

static struct attribute_group uv_attribute_group =
{
  .attrs = uv_attributes
} ;

static int uv_input_init( struct ga1a_data *data )
{
  struct input_dev   *dev ;
  int         err = 0 ;

  dev = input_allocate_device( ) ;
  if( !dev )
  {
    printk( KERN_ERR "%s, input_allocate_device error(%d)!!\n", __func__, err ) ;
    return -ENOMEM ;
  }

  set_bit( EV_ABS, dev->evbit ) ;
  input_set_capability( dev, EV_ABS, ABS_UV_REPORT ) ;
  input_set_abs_params( dev, ABS_UV_REPORT, 0, 0x7fffffff, 0, 0 ) ;
  input_set_capability( dev, EV_ABS, ABS_WAKE ) ;
  input_set_abs_params( dev, ABS_WAKE, 0, 0x7fffffff, 0, 0 ) ;
  input_set_capability( dev, EV_ABS, ABS_CONTROL_REPORT ) ;
  input_set_abs_params( dev, ABS_CONTROL_REPORT, 0, 0x1ffff, 0, 0 ) ;

  dev->name = UV_SENSOR_NAME ;

  err = input_register_device( dev ) ;
  if( err < 0 )
  {
    input_free_device( dev ) ;
    printk( KERN_ERR "%s, input_register_device error(%d)!!\n", __func__, err ) ;
    return err ;
  }
  input_set_drvdata( dev, data ) ;

  data->uv_input_dev = dev ;

  return 0 ;
}

static void uv_data_polling( struct work_struct *work )
{
  struct ga1a_data  *data = container_of( ( struct delayed_work * )work,
                            struct ga1a_data, uv_work ) ;
  //unsigned int    uv_val ;
  u8          rdata[4] ;
  u32         data_uv[2];
  int         ret;

  if( data != NULL )
  {
    mutex_lock( &data->mutex ) ;
    
    /* confirm validity before reading data */
    ga1a_i2c_read( REG_COMMAND1, rdata, 1, data->client ) ;
    if( ( rdata[0] & 0x12 ) != 0x02 ) /* Check operation UV and Flag assertion */
    {
      /* suspect that data are read out before the sensor finishes measuring 
      * when FLAG bit isn't asserted 
      */
      printk( KERN_INFO "Operation isn't UV mode or FLAG isn't asserted \n" ) ;
      
    }
    else
    {
      /* read data */
      ga1a_i2c_read( REG_D0_LSB, rdata, sizeof( rdata ), data->client ) ;
      data_uv[0] = rdata[0]+rdata[1]*256;
      data_uv[1] = rdata[2]+rdata[3]*256;
      pr_debug( "read sensor uv_clear=%d,uv_cut=%d\n", data_uv[0], data_uv[1] ) ;
     
      /*    UV mode (Range) change */
      ret = uv_mode_change(data_uv, data);
      
      if(ret == 0)
      {
        /* UV data correction */ 
        uv_data_correct( data_uv, data ) ;

        //input_report_abs( data->uv_input_dev, ABS_UV_REPORT, uv_val ) ;
        input_report_abs( data->uv_input_dev, ABS_UV_REPORT, ( data->uv_mode << 17 ) | ( 0x0 << 16 ) | data_uv[0] ) ;
        input_report_abs( data->uv_input_dev, ABS_UV_REPORT, ( data->uv_mode << 17 ) | ( 0x1 << 16 ) | data_uv[1] ) ;
        input_sync( data->uv_input_dev ) ;
      }

      mutex_unlock( &data->mutex ) ;

      if ( data->als_enabled ) 
      {
        mutex_lock( &data->mutex ) ;
        data->now_operation = OPERATION_ALS;
        ga1a_change_operation(data);
        mutex_unlock( &data->mutex ) ;
        schedule_delayed_work( &data->als_work, msecs_to_jiffies( data->als_delay ) ) ;
        //printk( KERN_INFO "Operation changed in UV polling now:%d \n", data->now_operation) ;
      }
      else if( data->uv_enabled )
      {
        schedule_delayed_work( &data->uv_work, msecs_to_jiffies( data->uv_delay ) ) ;
      }
    }
  }
}



/* *****************************************************************************
    ga1auv100wp
***************************************************************************** */

static void ga1a_change_operation( struct ga1a_data *data )
{
  int   i ; 
  u8    value ;

  value = ( COMMAND2_RST ) ;
  ga1a_i2c_write( REG_COMMAND2, &value, data->client ) ;

  if( data->now_operation == OPERATION_ALS )
  {
    if( data->als_mode == HIGH_LUX_MODE )
      data->regALS[REG_COMMAND3] = ( HIGH_LUX_RANGE | ( data->regALS[REG_COMMAND3] & 0x07 ) ) ;
    else
      data->regALS[REG_COMMAND3] = ( LOW_LUX_RANGE | ( data->regALS[REG_COMMAND3] & 0x07 ) ) ;
    
    for( i = 1 ; i < sizeof( data->regALS ) ; i++ )
      ga1a_i2c_write( i, &data->regALS[i], data->client ) ;

    data->regALS[REG_COMMAND1] = ( COMMAND1_WAKEUP | COMMAND1_ALS ) ;
    ga1a_i2c_write( REG_COMMAND1, &data->regALS[REG_COMMAND1], data->client ) ;
    
  }
  else /* OPERATION_UV */
  {
    if( data->uv_mode == HIGH_UV_MODE )
      data->regUV[REG_COMMAND3] = ( HIGH_UV_RANGE | ( data->regUV[REG_COMMAND3] & 0x07 ) ) ;
    else
      data->regUV[REG_COMMAND3] = ( LOW_UV_RANGE | ( data->regUV[REG_COMMAND3] & 0x07 ) ) ;

    for( i = 1 ; i < sizeof( data->regUV ) ; i++ )
      ga1a_i2c_write( i, &data->regUV[i], data->client ) ;

    data->regUV[REG_COMMAND1] = ( COMMAND1_WAKEUP ) ;
    ga1a_i2c_write( REG_COMMAND1, &data->regUV[REG_COMMAND1], data->client ) ;

  }
  
}


static ssize_t
ga1a_reg_show( struct class *class,
  struct class_attribute *attr,
  char *buf )
{
  struct ga1a_data  *data ;
  u8  rdata[8] ;

  pr_debug( "ga1a_reg_show\n" ) ;

  data = _sinfo ;
  mutex_lock( &data->mutex ) ;
  ga1a_i2c_read( REG_COMMAND1, rdata, sizeof( rdata ), data->client ) ;
  mutex_unlock( &data->mutex ) ;

  return sprintf( buf, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x",
            rdata[0], rdata[1], rdata[2], rdata[3], rdata[4], rdata[5], rdata[6], rdata[7] ) ;
}

static ssize_t
ga1a_reg_store( struct class *class,
  struct class_attribute *attr,
  const char *buf,
  size_t count )
{
  struct ga1a_data  *data ;
  char        *reg_p ;
  char        *value_p ;
  u32         reg ;
  u32         value ;
  char        tmpbuf[8] ;
  char        *endp ;
  u8          temp ;

  pr_debug( "ga1a_reg_store data=%s\n", buf ) ;

  data = _sinfo;

  if( strlen( buf ) > 7 )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  memset( tmpbuf, 0x00, sizeof( tmpbuf ) ) ;
  memcpy( tmpbuf, buf, strlen( buf ) ) ;

  reg_p = strtok( tmpbuf, "," ) ;
  value_p = strtok( NULL, NULL ) ;

  if( reg_p == NULL || value_p == NULL )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  reg = simple_strtoul( reg_p, &endp, 16 ) ;
  if( !( endp != reg_p &&
      ( endp == ( reg_p + strlen( reg_p ) )
        || ( endp == ( reg_p + strlen( reg_p ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }
  value = simple_strtoul( value_p, &endp, 16 ) ;
  if( !( endp != value_p &&
      ( endp == ( value_p + strlen( value_p ) )
        || ( endp == ( value_p + strlen( value_p ) - 1 ) && *endp == '\n' ) ) ) )
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
    return count ;
  }

  if( reg <= 0x03 && value <= 255 )
  {
    pr_debug( "  reg=%02x,value=%02x\n", reg, value ) ;
    mutex_lock( &data->mutex ) ;
    temp = ( u8 )value ;
    ga1a_i2c_write( ( u8 )reg, &temp, data->client ) ;
    mutex_unlock( &data->mutex ) ;
  }
  else
  {
    printk( KERN_ERR "invalid value (%s)\n", buf ) ;
  }

  return count ;
}

static CLASS_ATTR( reg, S_IRUGO|S_IWUGO, ga1a_reg_show, ga1a_reg_store ) ;

static int ga1a_probe( struct platform_device *pdev )
{
  struct ga1a_data *ga1a ;
  u8  value = 0 ;
  int err = 0 ;
  int i ;

  pr_info( "ga1a_probe start !! \n" ) ;

  ga1a = kzalloc( sizeof( struct ga1a_data ), GFP_KERNEL ) ;
  if( !ga1a )
  {
    printk( KERN_ERR "kzalloc error !! \n" ) ;
    return -ENOMEM ;
  }

  _sinfo = ga1a;
  mutex_init(&ga1a->mutex);

  ga1a->power_enable = of_get_named_gpio_flags(pdev->dev.of_node,
			"ga1a,power", 0, NULL);
  pr_info("ga1a power pin = %d",ga1a->power_enable);
  gpio_request(ga1a->power_enable, "ga1a,power_enable");
  gpio_direction_output(ga1a->power_enable, 1);
  pr_info("ga1a power pin status = %d",gpio_get_value(ga1a->power_enable));

  ga1a->dev_class = class_create( THIS_MODULE, CLASS_NAME ) ;
  if( IS_ERR( ga1a->dev_class ) )
  {
    printk( KERN_ERR "%s: could not create dev_class\n", __func__) ;
    goto err_dev_class_create ;
  }

  if( class_create_file( ga1a->dev_class, &class_attr_reg ) < 0 )
  {
    printk( KERN_ERR "%s: could not create class file(%s)!\n", __func__, class_attr_reg.attr.name ) ;
    goto err_class_create_file ;
  }

  ga1a->als_enabled = 0 ;
  ga1a->als_delay = SENSOR_DEFAULT_DELAY ;

  INIT_DELAYED_WORK( &ga1a->als_work, als_data_polling ) ;

  err = als_input_init( ga1a ) ;
  if( err < 0 )
  {
    goto err_als_input_device ;
  }

  err = sysfs_create_group( &ga1a->als_input_dev->dev.kobj, &als_attribute_group ) ;
  if( err )
  {
    printk( KERN_ERR
          "sysfs_create_group failed[%s]\n",
            ga1a->als_input_dev->name ) ;
    goto err_sysfs_create_group_als ;
  }


  ga1a->uv_enabled = 0 ;
  ga1a->uv_delay = SENSOR_DEFAULT_DELAY ;

  INIT_DELAYED_WORK( &ga1a->uv_work, uv_data_polling ) ;

  err = uv_input_init( ga1a ) ;
  if( err < 0 )
  {
    goto err_uv_input_device ;
  }

  err = sysfs_create_group( &ga1a->uv_input_dev->dev.kobj, &uv_attribute_group ) ;
  if( err )
  {
    printk( KERN_ERR
          "sysfs_create_group failed[%s]\n",
            ga1a->uv_input_dev->name ) ;
    goto err_sysfs_create_group_uv ;
  }

  platform_set_drvdata( pdev, ga1a ) ;

  ga1a_i2c_init( ) ;

  value = COMMAND1_SD ;
  err = ga1a_i2c_write( REG_COMMAND1, &value, ga1a->client ) ;
  if( err < 0 )
  {
    printk( KERN_ERR "there is no such device. !! \n" ) ;
    goto err_no_device ;
  }

  ga1a->now_operation = OPERATION_UV ;

  for( i = 0 ; i < sizeof( ga1a_init_reg_uv ) ; i++ )
  {
    ga1a->regUV[i] = ga1a_init_reg_uv[i] ;
    ga1a->regALS[i] = ga1a_init_reg_als[i] ;
  }

  device_init_wakeup( &pdev->dev, 1 ) ;
  pr_info( "ga1a sensor probed !! \n" ) ;

  return 0 ;

err_no_device:
  i2c_del_driver( &ga1a_i2c_driver ) ;
err_sysfs_create_group_uv:
  sysfs_remove_group( &ga1a->uv_input_dev->dev.kobj,
              &uv_attribute_group ) ;
  input_unregister_device( ga1a->uv_input_dev ) ;
  input_free_device( ga1a->uv_input_dev ) ;
err_uv_input_device:
  class_remove_file( ga1a->dev_class, &class_attr_reg ) ;


err_sysfs_create_group_als:
  sysfs_remove_group( &ga1a->als_input_dev->dev.kobj,
              &als_attribute_group ) ;
  input_unregister_device( ga1a->als_input_dev ) ;
  input_free_device( ga1a->als_input_dev ) ;
err_als_input_device:
  class_remove_file( ga1a->dev_class, &class_attr_reg ) ;
err_class_create_file:
  class_destroy( ga1a->dev_class ) ;
err_dev_class_create:
  mutex_destroy( &ga1a->mutex ) ;
  kfree( ga1a ) ;
  return err ;
}

static int
ga1a_remove( struct platform_device *pdev )
{
  struct ga1a_data  *data = platform_get_drvdata( pdev ) ;
  u8          value ;

  pr_debug( "ga1a_remove\n" ) ;

  if( data != NULL )
  {
    value = COMMAND1_SD ; // shutdown
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;

    if( data->als_enabled )
    {
      flush_scheduled_work( ) ;
      cancel_delayed_work( &data->als_work ) ;
    }
    if( data->als_input_dev != NULL )
    {
      sysfs_remove_group( &data->als_input_dev->dev.kobj, &als_attribute_group ) ;
      input_unregister_device( data->als_input_dev ) ;
      input_free_device( data->als_input_dev ) ;
    }
    if( data->uv_enabled )
    {
      flush_scheduled_work( ) ;
      cancel_delayed_work( &data->uv_work ) ;
    }
    if( data->uv_input_dev != NULL )
    {
      sysfs_remove_group( &data->uv_input_dev->dev.kobj, &uv_attribute_group ) ;
      input_unregister_device( data->uv_input_dev ) ;
      input_free_device( data->uv_input_dev ) ;
    }
    if( data->dev_class != NULL )
    {
      class_remove_file( data->dev_class, &class_attr_reg ) ;
      class_destroy( data->dev_class ) ;
    }
    i2c_del_driver( &ga1a_i2c_driver ) ;
    device_init_wakeup( &pdev->dev, 0 ) ;
    mutex_destroy( &data->mutex ) ;
    kfree( data ) ;
  }

  return 0 ;
}

static int
ga1a_suspend( struct platform_device *pdev, pm_message_t state )
{
  struct ga1a_data  *data = platform_get_drvdata( pdev ) ;

  pr_debug( "ga1a_suspend \n" ) ;

  if( data->als_enabled && data->now_operation==OPERATION_ALS )
  {
    cancel_delayed_work_sync( &data->als_work ) ;
  }
  if( data->uv_enabled && data->now_operation==OPERATION_UV )
  {
    cancel_delayed_work_sync( &data->uv_work ) ;
  }

  return 0 ;
}

static int
ga1a_resume( struct platform_device *pdev )
{
  struct ga1a_data  *data = platform_get_drvdata( pdev ) ;

  pr_debug( "ga1a_resume \n" ) ;

  if( data->als_enabled && data->now_operation==OPERATION_ALS )
  {
    schedule_delayed_work( &data->als_work, 0 ) ;
  }
  if( data->uv_enabled && data->now_operation==OPERATION_UV )
  {
    schedule_delayed_work( &data->uv_work, 0 ) ;
  }

  return 0 ;
}

static void
ga1a_shutdown( struct platform_device *pdev )
{
  struct ga1a_data  *data = platform_get_drvdata( pdev ) ;
  u8          value ;

  pr_debug( "ga1a_shutdown\n" ) ;

  if( data != NULL )
  {
    value = COMMAND1_SD ; // shutdown
    ga1a_i2c_write( REG_COMMAND1, &value, data->client ) ;

    if( data->als_enabled )
    {
      flush_scheduled_work( ) ;
      cancel_delayed_work( &data->als_work ) ;
    }
    if( data->als_input_dev != NULL )
    {
      sysfs_remove_group( &data->als_input_dev->dev.kobj, &als_attribute_group ) ;
      input_unregister_device( data->als_input_dev ) ;
      input_free_device( data->als_input_dev ) ;
    }
    if( data->uv_enabled )
    {
      flush_scheduled_work( ) ;
      cancel_delayed_work( &data->uv_work ) ;
    }
    if( data->uv_input_dev != NULL )
    {
      sysfs_remove_group( &data->uv_input_dev->dev.kobj, &uv_attribute_group ) ;
      input_unregister_device( data->uv_input_dev ) ;
      input_free_device( data->uv_input_dev ) ;
    }
    if( data->dev_class != NULL )
    {
      class_remove_file( data->dev_class, &class_attr_reg ) ;
      class_destroy( data->dev_class ) ;
    }
    i2c_del_driver( &ga1a_i2c_driver ) ;
    device_init_wakeup( &pdev->dev, 0 ) ;
    mutex_destroy( &data->mutex ) ;
    kfree( data ) ;
  }
}

static const struct of_device_id ga1a_dt_match[] = {
	{.compatible = "sharp, ga1a"},
	{}
};

MODULE_DEVICE_TABLE(of, ga1a_dt_match);

static struct platform_driver ga1a_driver = {
  .probe      = ga1a_probe,
  .remove     = ga1a_remove,
  .suspend    = ga1a_suspend,
  .resume     = ga1a_resume,
  .shutdown   = ga1a_shutdown,
  .driver = {
    .name   = DEVICE_NAME,
    .owner  = THIS_MODULE,
    .of_match_table = ga1a_dt_match,
  },
} ;

static int __init ga1a_module_init( void )
{
  int ret;

  pr_debug( "ga1a_module_init \n" ) ;
  ret = platform_driver_register( &ga1a_driver ) ;

  return ret;
}

static void __exit ga1a_module_exit( void )
{
  pr_info( "ga1a_module_exit \n" ) ;

  platform_driver_unregister( &ga1a_driver ) ;
}

module_init( ga1a_module_init ) ;
module_exit( ga1a_module_exit ) ;

MODULE_AUTHOR( "SHARP" ) ;
MODULE_DESCRIPTION( "Optical Sensor driver for ga1auv100wp" ) ;
MODULE_LICENSE( "GPL" ) ;
