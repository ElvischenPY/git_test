#ifndef __KERNEL_LETVLOG__
#define __KERNEL_LETVLOG__
#include <linux/printk.h>
#include <linux/sysctl.h>

#define	LETVLOG_DEBUG_0  0x10
#define	LETVLOG_DEBUG_1  0x20
#define	LETVLOG_DEBUG_2  0x40
#define	LETVLOG_DEBUG_3  0x80
enum letvlog_level{
	LETVLOG_DISABLE = 0x00,
	LETVLOG_ERROR   = 0x01,
	LETVLOG_WARNING = 0x02,
	LETVLOG_INFO    = 0x03,
	LETVLOG_DEBUG   = 0x04,
};

#define LETVLOG_DEBUG0_EN (LETVLOG_DEBUG|LETVLOG_DEBUG_0)
#define LETVLOG_DEBUG1_EN (LETVLOG_DEBUG|LETVLOG_DEBUG_1)
#define LETVLOG_DEBUG2_EN (LETVLOG_DEBUG|LETVLOG_DEBUG_2)
#define LETVLOG_DEBUG3_EN (LETVLOG_DEBUG|LETVLOG_DEBUG_3)

#define LETVLOG_MOD_NAME_LENGTH (32+4)  //max name string length , make sure 8 bytes aligned
#define LETVLOG_SETING_STR_MAX  64
struct letvlog_mod{
	char name[LETVLOG_MOD_NAME_LENGTH];
	int  log_en;
};

extern struct letvlog_mod __start_letvlog[];
extern  struct letvlog_mod __stop_letvlog[];
#define  modtable_start (__start_letvlog)
#define  modtable_stop  (__stop_letvlog)

#define LETVLOG_DEBUG_ALL = 0xF3;

/*
*if this flag been set, the default output level will be defined by this flag..
* this flag may be set by defconfig file when relase a version.
*/
#ifndef CONFIG_LETVLOG_DEFAULT_DEBUG_EN
#define CONFIG_LETVLOG_DEFAULT_DEBUG_EN LETVLOG_DEBUG_0
#endif

#ifndef CONFIG_LETVLOG_DEFAULT_LEVEL_EN
#define CONFIG_LETVLOG_DEFAULT_LEVEL_EN LETVLOG_DEBUG
#endif

#define LETVLOG_DEFAULT_EN (CONFIG_LETVLOG_DEFAULT_LEVEL_EN|CONFIG_LETVLOG_DEFAULT_DEBUG_EN)

/**** parse log_en bits****/
#define format_letvlog_enable_bits(level,debug)	(level| debug)

#define log_level_en(log_en) (log_en&0x07)
#define log_debug_en(log_en) log_en
/*********** end of parse log_en bits**************/


extern int proc_do_letvlog(struct ctl_table *table, int write, void __user *buffer, size_t *lenp, loff_t *ppos);

#define TAG_STRUCT_NAME(name) letvlog_##name

#if defined(CONFIG_PRINTK)&&defined(CONFIG_LETVLOG)

#define DECLARE_LETVLOG(name,level_en)\
	static struct letvlog_mod TAG_STRUCT_NAME(name)\
	__used	\
	__attribute__ ((unused,__section__ ("__letvlog_mod"))) \
	= { #name, level_en};\
	static struct letvlog_mod *mod_TAG = &TAG_STRUCT_NAME(name);
//	char* lvel = &(mod_TAG->level_en);

#define DECLARE_LETVLOG_DEFAULT(name)\
	DECLARE_LETVLOG(name,(CONFIG_LETVLOG_DEFAULT_LEVEL_EN|CONFIG_LETVLOG_DEFAULT_DEBUG_EN))

//likely(log_level_en(mod_TAG->log_en) >= LETVLOGERROR)
#define letv_err(fmt, args...)\
	do{\
		if(likely(log_level_en(mod_TAG->log_en) >= LETVLOG_ERROR)){\
			printk(KERN_ALERT "[%s/E](%s %d): " fmt,mod_TAG->name,__func__,__LINE__,## args);\
		}\
	}while(0)

#define letv_war(fmt, args...)\
	do{\
		if(likely(log_level_en(mod_TAG->log_en)>LETVLOG_ERROR)){\
			printk(KERN_ALERT "[%s/W](%s): " fmt,mod_TAG->name,__func__,## args);\
		}\
	}while(0)

#define letv_info(fmt, args...)\
	do{\
		if(likely(log_level_en(mod_TAG->log_en)>LETVLOG_WARNING)){\
			printk(KERN_ALERT "[%s/I](%s): " fmt,mod_TAG->name,__func__,## args);\
		}\
	}while(0)


#define letv_debug(fmt, args...)\
	do{\
		if(unlikely((mod_TAG->log_en& LETVLOG_DEBUG0_EN) == LETVLOG_DEBUG0_EN)){\
			printk(KERN_ALERT "[%s/D](%s): " fmt,mod_TAG->name,__func__,## args);\
		}\
	}while(0)

#define letv_debug1(fmt, args...)\
	do{\
		if(unlikely((mod_TAG->log_en&LETVLOG_DEBUG1_EN) == LETVLOG_DEBUG1_EN)){\
			printk(KERN_ALERT "[%s/D1](%s): " fmt,mod_TAG->name,__func__,## args);\
		}\
	}while(0)

#define letv_debug2(fmt, args...)\
	do{\
		if(unlikely((mod_TAG->log_en&LETVLOG_DEBUG2_EN) == LETVLOG_DEBUG2_EN)){\
			printk(KERN_ALERT "[%s/D2](%s): " fmt,mod_TAG->name,__func__,## args);\
		}\
	}while(0)

#define letv_debug3(fmt, args...)\
	do{\
		if(unlikely((mod_TAG->log_en&LETVLOG_DEBUG3_EN) == LETVLOG_DEBUG3_EN)){\
			printk(KERN_ALERT "[%s/D3](%s): " fmt,mod_TAG->name,__func__,## args);\
		}\
	}while(0)

#else
#define DECLARE_LETVLOG(name,level_en)
#define DECLARE_LETVLOG_DEFAULT(name)
#define letv_err(fmt, args...)
#define letv_war(fmt, args...)
#define letv_info(fmt, args...)
#define letv_debug(fmt, args...)
#define letv_debug1(fmt, args...)
#define letv_debug2(fmt, args...)
#define letv_debug3(fmt, args...)
#endif  //end of CONFIG_PRINTK
#endif
