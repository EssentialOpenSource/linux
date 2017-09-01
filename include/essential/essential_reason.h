#ifndef __ESSENTIAL_REASON_H
#define __ESSENTIAL_REASON_H

#include <linux/input/qpnp-power-on.h>

#define REASON_ANDROID_MODE         0x00
#define REASON_RECOVERY_MODE        0x01
#define REASON_FASTBOOT_MODE        0x02
#define REASON_ALARM_BOOT           0x03
#define REASON_DM_VERITY_LOGGING    0x04
#define REASON_DM_VERITY_ENFORCING  0x05
#define REASON_DM_VERITY_KEYSCLEAR  0x06
#define REASON_EMERGENCY_DLOAD      0xFF

#define REASON_KERNEL_BUG           0x20
#define REASON_KERNEL_PANIC         0x21
#define REASON_KERNEL_RESTART       0x22
#define REASON_KERNEL_SHUTDOWN      0x23
#define REASON_KERNEL_WDOG          0x24
#define REASON_MODEM_FATAL          0x25
#define REASON_SYSTEM_CRASH         0x26
#define REASON_UNKNOWN_RESET        0x27
#define REASON_OVER_TEMPERATURE     0x28
#define REASON_MEMORY_TEST          0x29

#endif
