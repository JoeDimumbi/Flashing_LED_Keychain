/* 
 * File:   config.h
 * Author: Lumkani Team
 *
 * Created on 10 September 2018
 */

#ifndef CONFIG_H
#define	CONFIG_H

#define CONFIG_FALSE 0
#define CONFIG_TRUE  1

// Configuration flags

#define ENABLE_TESTING                 CONFIG_FALSE                             // If defined testing functions are defined
#define ENABLE_DEBUG                   CONFIG_TRUE                              // If defined enable debug functionality

// Other configuration

#define FIRMWARE_BUILD_VERSION         mSTRINGIZE_VALUE_OF(BUILD_VERSION)       // Git hash used as firmware version for easy identification.

#endif	/* CONFIG_H */

