#ifndef __ONENET_CONFIG_H__
#define __ONENET_CONFIG_H__

//#define CPU_FREQ_160MHZ

/* Here needs to be changed according to your envirnoment. */
#define WIFI_SSID           "MIspartazhc"//"115_X11"
#define WIFI_PASS           "123123123"//"19980527"

#define ONENET_HOST         "183.230.40.39"
#define ONENET_PORT         (6002)

/* Here needs to be changed accoding to your oneONET configure. */
#define ONENET_DEVICE_ID    "522001840"                  // mqtt client id
#define ONENET_PROJECT_ID   "226658"                    // mqtt username
#define ONENET_AUTH_INFO    "spartazhc"   // mqtt password

/* Here needs to be changed accoding to your oneONET configure. */
#define ONENET_DATA_STREAM  "temperature"

#define ONENET_PUB_INTERVAL (60) // unit: s

#endif

