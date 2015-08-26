#ifndef __LINUX_FT6X36_DOWNLOAD_LIB_H__
#define __LINUX_FT6X36_DOWNLOAD_LIB_H__

typedef int (*FTS_I2c_Read_Function)(unsigned char * , int , unsigned char *, int);
typedef int (*FTS_I2c_Write_Function)(unsigned char * , int );

extern int Init_I2C_Read_Func(FTS_I2c_Read_Function fpI2C_Read);
extern int Init_I2C_Write_Func(FTS_I2c_Write_Function fpI2C_Write);

extern int ft6x36_Lib_DownloadMain(unsigned char MainBuf[], unsigned short fwlen);
extern int ft6x36_Lib_ReadMainFlash(unsigned char MainBuf[], unsigned short buflen);
extern int ft6x36_Lib_Enter_Download_Mode(void);
#endif
