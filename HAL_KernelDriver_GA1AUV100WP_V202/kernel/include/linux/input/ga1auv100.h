#ifndef __GA1AUV100WP_H__
#define __GA1AUV100WP_H__

/* Reg. */
#define REG_COMMAND1    0x00  // Read & Write
#define REG_COMMAND2    0x01  // Read & Write
#define REG_COMMAND3    0x02  // Read & Write
#define REG_INTERVAL    0x03  // Read & Write
#define REG_D0_LSB      0x04  // Read Only
#define REG_D0_MSB      0x05  // Read Only
#define REG_D1_LSB      0x06  // Read Only
#define REG_D1_MSB      0x07  // Read Only
#define REG_DEVICE_ID   0x08  // Read Only
#define REG_TEST        0x09  // Read & Write

/* ADDR:0x00 COMMAND1 */
#define COMMAND1_SD     0x00  // OP3:0
#define COMMAND1_WAKEUP 0x80  // OP3:1
#define COMMAND1_ALS    0x10  // OP01:01
#define COMMAND1_NOCLR  0x02  // FLAG:1

/* ADDR:0x01 COMMAND2 */
#define COMMAND2_PD1     0x80  // PD1:1
#define COMMAND2_PD0     0x40  // PD0:1
#define COMMAND2_PIN     0x10  // PIN:1
#define COMMAND2_FREQ    0x08  // FREQ:1
#define COMMAND2_NOWAIT  0x04  // NOWAIT:1
#define COMMAND2_INTTYPE 0x02  // INTTYPE:1
#define COMMAND2_RST     0x01  // RST:1

/* ADDR:0x02 COMMAND3 */
#define COMMAND3_RANGEX1    0x00  // RANGE:0000
#define COMMAND3_RANGEX2    0x10  // RANGE:0001
#define COMMAND3_RANGEX4    0x20  // RANGE:0010
#define COMMAND3_RANGEX8    0x30  // RANGE:0011
#define COMMAND3_RANGEX16   0x40  // RANGE:0100
#define COMMAND3_RANGEX32   0x50  // RANGE:0101
#define COMMAND3_RANGEX64   0x60  // RANGE:0110
#define COMMAND3_RANGEX128  0x70  // RANGE:0111
#define COMMAND3_RANGEX256  0xF0  // RANGE:1111
#define COMMAND3_HALF       0x08  // HALF:1
#define COMMAND3_RES_21     0x00  // RES:000
#define COMMAND3_RES_20     0x01  // RES:001
#define COMMAND3_RES_19     0x02  // RES:010
#define COMMAND3_RES_18     0x03  // RES:011
#define COMMAND3_RES_17     0x04  // RES:100
#define COMMAND3_RES_16     0x05  // RES:101
#define COMMAND3_RES_15     0x06  // RES:110
#define COMMAND3_RES_14     0x07  // RES:111

/* ADDR:0x03 INTERVAL */
#define INTERVAL_0      0x00  // INTVAL:000
#define INTERVAL_12     0x01  // INTVAL:001
#define INTERVAL_25     0x02  // INTVAL:010
#define INTERVAL_50     0x03  // INTVAL:011
#define INTERVAL_100    0x04  // INTVAL:100
#define INTERVAL_200    0x05  // INTVAL:101
#define INTERVAL_400    0x06  // INTVAL:110
#define INTErVAL_800    0x07  // INTVAL:111


/* event code */
#define ABS_WAKE           ( ABS_BRAKE )
#define ABS_CONTROL_REPORT ( ABS_THROTTLE )
#define ABS_LUX_REPORT     ( ABS_MISC )
#define ABS_UV_REPORT      ( ABS_MISC )

/* platform data */
struct ga1auv100_platform_data
{
//  int   gpio ;
} ;

#endif
