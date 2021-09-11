#define BUILD_NUMBER 2

#define LMIC_DEBUG_LEVEL 1
#define LMIC_ENABLE_event_logging 1

#undef CC_PLATFORM_LORA32
#define CC_PLATFORM_MINI32
#undef CC_PLATFORM_UNO // does not fit in flash

#undef USE_GPS
#undef USE_OLED

//#define SINGLE_CHANNEL 8

// dev-01
#if defined(CC_PLATFORM_LORA32)
const char *devAddr = "26061CE8";
const char *nwkSKey = "58C1564AACA3ED1625F8F7B57F8103E3";
const char *appSKey = "8DC8A1F7A121FD0B6D43AE41CD442BA4";
#define ADDR_MPU6050 (uint8_t)0x69
#elif defined(CC_PLATFORM_MINI32)
// dev-02
const char *devAddr = "260615F2";
const char *nwkSKey = "CDA94424A43EE02EC9DF85EF331BF636";
const char *appSKey = "41419A0705B04547172B81C8F539053F";
#else
const char *devAddr = "260615F2";
const char *nwkSKey = "CDA94424A43EE02EC9DF85EF331BF636";
const char *appSKey = "41419A0705B04547172B81C8F539053F";
#endif

#define SEND_TIMER 900
#define CLOUDCROWD_VERSION 1
