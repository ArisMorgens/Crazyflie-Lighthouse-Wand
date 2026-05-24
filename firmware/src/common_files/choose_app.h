#pragma once

#if defined(CONFIG_WAND_APP)
    #define BUILD_WAND_APP
#elif defined(CONFIG_RECEIVER_APP)
    #define BUILD_RECEIVER_APP
#elif defined(CONFIG_RECEIVER_HLC_APP)
    #define BUILD_RECEIVER_HLC_APP
#else
    #error "No app selected! Build with APP_TYPE=wand, receiver, or receiver_hlc"
#endif