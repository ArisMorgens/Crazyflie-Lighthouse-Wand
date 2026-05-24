#include "choose_app.h"
#ifdef BUILD_WAND_APP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"
#include "stabilizer_types.h"
#include "log.h"
#include "cfassert.h"
#include "param.h"
#include "sensors.h"

#define DEBUG_MODULE "Wand"
#include "debug.h"

extern bool button_is_pressed(void);        // From button.c
extern void wand_sounds_init(void);         // From wand_sounds.c
extern void wand_sounds_stop_startup(void);
extern void wand_sounds_play_melody(void);
extern void wand_sounds_stop(void);

void appMain()
{
    DEBUG_PRINT("WAND ACTIVE - Broadcasting position...\n");

    wand_sounds_init();  // plays startup sound while sensors calibrate

    paramVarId_t idLowVoltage = paramGetVarId("pm", "lowVoltage");
    paramSetFloat(idLowVoltage, 3.2f);
    paramVarId_t idCriticalLowVoltage = paramGetVarId("pm", "criticalLowVoltage");
    paramSetFloat(idCriticalLowVoltage, 3.0f);

    while (!sensorsAreCalibrated()) {
        vTaskDelay(M2T(50));
    }
    wand_sounds_stop_startup();
    DEBUG_PRINT("Sensors calibrated, starting broadcast\n");

    static P2PPacket pkt;
    pkt.port = 0x01;                    // choose a port > 0 to avoid collisions
    pkt.size = 1 + sizeof(float)*6;     // [id][x,y,z,dx,dy,dz]

    uint64_t addr = configblockGetRadioAddress();
    uint8_t my_id = (uint8_t)(addr & 0xFF);
    pkt.data[0] = my_id;

    // set up variables to read position from estimator
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idZ = logGetVarId("stateEstimate", "z");
    logVarId_t idPitch = logGetVarId("stateEstimate", "pitch");
    logVarId_t idYaw   = logGetVarId("stateEstimate", "yaw");

    ASSERT(idX!=0 && idY!=0 && idZ!=0);

    bool lastButtonState = false;

    while(1) {
        float x = logGetFloat(idX);
        float y = logGetFloat(idY);
        float z = logGetFloat(idZ);
        float pitch = - logGetFloat(idPitch);
        float yaw   = logGetFloat(idYaw);

        float pitch_rad = pitch * 0.0174533f;   // pi / 180
        float yaw_rad   = yaw   * 0.0174533f;   // pi / 180

        float cp = cosf(pitch_rad);
        float sp = sinf(pitch_rad);
        float cy = cosf(yaw_rad);
        float sy = sinf(yaw_rad);

        // body X axis expressed in world frame
        float dx = cp * cy;
        float dy = cp * sy;
        float dz = -sp;

        float n = sqrtf(dx*dx + dy*dy + dz*dz);
        dx /= n; dy /= n; dz /= n;

        // pack into P2P packet
        memcpy(&pkt.data[1], &x, sizeof(float));
        memcpy(&pkt.data[1+4], &y, sizeof(float));
        memcpy(&pkt.data[1+8], &z, sizeof(float));
        memcpy(&pkt.data[1+12], &dx, sizeof(float));
        memcpy(&pkt.data[1+16], &dy, sizeof(float));
        memcpy(&pkt.data[1+20], &dz, sizeof(float));

        bool btn = button_is_pressed();

        if (btn && !lastButtonState) {
            wand_sounds_play_melody();
        } else if (!btn && lastButtonState) {
            wand_sounds_stop();
        }
        lastButtonState = btn;

        if (btn) {
            radiolinkSendP2PPacketBroadcast(&pkt);
            DEBUG_PRINT("[TX] Wand line: pos=(%.2f %.2f %.2f) pitch=%.2f, yaw=%.2f, dir=(%.2f %.2f %.2f)\n",
                (double)x, (double)y, (double)z, (double)pitch, (double)yaw,
                (double)dx, (double)dy, (double)dz);
        }

        vTaskDelay(M2T(100));  // send @10Hz
    }
}

#endif // BUILD_WAND_APP
