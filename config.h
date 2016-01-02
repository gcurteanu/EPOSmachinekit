/*
Configuration defines
*/

#ifndef __EPOS_CONFIG_H__
#define __EPOS_CONFIG_H__

#if defined(RTAPI)
#define eprintf(...) rtapi_print (__VA_ARGS__)
#elif defined(__XENO__)
#define eprintf(...) rt_printf (__VA_ARGS__)
#else
#define eprintf(...) printf (__VA_ARGS__)
#endif

#ifdef __DEBUG__
#define MSG_DBG(...)    eprintf(__VA_ARGS__)
#else
#define MSG_DBG(...)
#endif

#define MSG_WARN(...)   eprintf(__VA_ARGS__)

#define MSG_ERR(...)    eprintf(__VA_ARGS__)

/*
    Bit routines
*/
#define SET_BIT(val, bitIndex) val |= (1 << bitIndex)
#define CLEAR_BIT(val, bitIndex) val &= ~(1 << bitIndex)
#define TOGGLE_BIT(val, bitIndex) val ^= (1 << bitIndex)
#define BIT_IS_SET(val, bitIndex) (val & (1 << bitIndex))

/*
    Various configuration defines
*/

/* maximum Concise DCF data size per node */
#define EPOS_DCF_MAX_SIZE   16384
/* maximum Concise DCF node entries in the file
(eq. how many distinct nodes in a DCF file, each having the size above) */
#define EPOS_DCF_MAX_NODES  16

// max time in us for a boot (10 seconds?)
#define NODE_BOOT_TIME 10*1000*1000

/* maximum number of drives supported */
#define EPOS_MAX_DRIVES     5
/* maximum number of errors per drive */
#define EPOS_MAX_ERRORS     32

#endif