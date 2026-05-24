#include "choose_app.h"
#ifdef BUILD_RECEIVER_APP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "log.h"
#include "param.h"
#include "cfassert.h"
#include "commander.h"
#include "supervisor.h"

#define DEBUG_MODULE "Receiver"
#include "debug.h"

static float attemptScore = 0.0f;  // smoothed confidence value 0-100
static bool grasped = false;
static uint32_t lastPacketTime = 0;
static float graspRange = 0.0f;      // <-- stored only on grasp
static bool landing_init = false;
static float landing_target_z = 0.0f;

static float graspDist      = 0.15f;  // within this range, the attempt is valid
static float graspThres     = 30.0f;  // score required to trigger grasp
static float buildRate      = 2.0f;   // how fast score increases per packet in range
static uint32_t lossTimeout = 2000;   // ms without packets before release
static float landHeight     = 0.3f;   // land if released below this height (m)
static uint8_t appEnabled   = 1;      // 0 = idle, cfclient has full control
static float setpointX = 0.0f;
static float setpointY = 0.0f;
static float setpointZ = 0.0f;

static bool armed = false;

// Packet format: [id][x,y,z][dx,dy,dz]
typedef struct {
    uint8_t id;
    float x, y, z;
    float dx, dy, dz;
} WandLinePacket;

static WandLinePacket lastPkt;   // store latest wand packet globally

static logVarId_t idX, idY, idZ;

static paramVarId_t idSef;
static paramVarId_t idWrgb;

#define LED_OFF    0
#define LED_GREEN  0x00008000
#define LED_YELLOW 0x00808000
#define LED_RED    0x00800000

static setpoint_t setpoint;

static float vector_norm(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

static void cross_product(float ax, float ay, float az,
                          float bx, float by, float bz,
                          float *rx, float *ry, float *rz)
{
    *rx = ay*bz - az*by;
    *ry = az*bx - ax*bz;
    *rz = ax*by - ay*bx;
}

static void setAbsolutePosition(setpoint_t *sp, float x, float y, float z, float yaw)
{
    sp->mode.x = modeAbs;
    sp->mode.y = modeAbs;
    sp->mode.z = modeAbs;

    sp->position.x = x;
    sp->position.y = y;
    sp->position.z = z;

    sp->mode.yaw = modeAbs;
    sp->attitude.yaw = yaw;
}

/*
 * Grasping algorithm
 * ------------------
 * The wand broadcasts a 3-D line: origin point P and unit direction D.
 *
 * 1. Distance to line: ||(r - P) × D||  where r is the drone's position.
 *    While this distance is below graspDist, the confidence score accumulates.
 *
 * 2. On grasp (score > graspThres): the range along the line is stored once as
 *    dot(r - P, D).  This locks the drone to a fixed point on the wand line.
 *
 * 3. Each subsequent packet: target = P + range * D.  The drone is commanded
 *    to that absolute position, so it "sticks" to the same spot on the wand.
 *
 * 4. Release: no packet for lossTimeout ms → grasp cleared, drone hovers.
 */
void p2pReceive(P2PPacket *p)
{
    if (!appEnabled) return;
    if (p->port != 0x01) return; // only wand broadcasts
    if(p->size != 1 + 6*sizeof(float)) return; // sanity check

    lastPacketTime = xTaskGetTickCount();        // wand still alive

    WandLinePacket pkt;
    pkt.id = p->data[0];
    memcpy(&pkt.x,  &p->data[1],    sizeof(float));
    memcpy(&pkt.y,  &p->data[1+4],  sizeof(float));
    memcpy(&pkt.z,  &p->data[1+8],  sizeof(float));
    memcpy(&pkt.dx, &p->data[1+12], sizeof(float));
    memcpy(&pkt.dy, &p->data[1+16], sizeof(float));
    memcpy(&pkt.dz, &p->data[1+20], sizeof(float));

    lastPkt = pkt;  // save for appMain

    // ---------------- distance calculation ----------------
    // Get receiver's own position
    float rx = logGetFloat(idX);
    float ry = logGetFloat(idY);
    float rz = logGetFloat(idZ);

    // Vector from line start to receiver
    float vx = rx - pkt.x;
    float vy = ry - pkt.y;
    float vz = rz - pkt.z;

    // Cross product v × d
    float cx, cy, cz;
    cross_product(vx, vy, vz, pkt.dx, pkt.dy, pkt.dz, &cx, &cy, &cz);

    // Distance = |v × d| / |d|  (direction vector should already be unit vector)
    float dist = vector_norm(cx, cy, cz);
    float dot = vx*pkt.dx + vy*pkt.dy + vz*pkt.dz;
    float vnorm = vector_norm(vx,vy,vz);
    float angle_deg = acosf(dot / vnorm) * 57.2958f;
    DEBUG_PRINT("angle=%.1f°  dist=%.2f\n", (double)angle_deg, (double)dist);

    DEBUG_PRINT("[RSSI -%d dBm] Wand(%d) Wandpos=(%.2f %.2f %.2f), Wandnorm=(dx=%.2f, dy=%.2f, dz=%.2f) Recpos=(%.2f %.2f %.2f) -> distance = %.2f m\n",
                p->rssi, pkt.id, (double)pkt.x,(double)pkt.y,(double)pkt.z, (double)pkt.dx,(double)pkt.dy,(double)pkt.dz,
                (double)rx,(double)ry,(double)rz,
                (double)dist);

    //------------------ ACQUIRE GRASP (only once) ------------------//
    if (!grasped) {
        if (dist < graspDist) {
            attemptScore = fminf(attemptScore + buildRate, 100.0f);
            paramSetInt(idWrgb, LED_YELLOW);
        }

        if (attemptScore > graspThres) {
            grasped = true;
            // compute range ONCE -> projection of v onto d
            graspRange = vx*pkt.dx + vy*pkt.dy + vz*pkt.dz;
            if (graspRange < 0) graspRange = 0;
            paramSetInt(idWrgb, LED_GREEN);
            DEBUG_PRINT("GRASPED! Wand=%d range=%.5f\n", pkt.id, (double)graspRange);
        }
    }
}

void appMain()
{
    DEBUG_PRINT("RECEIVER ACTIVE - Listening to Wand...\n");

    idX = logGetVarId("stateEstimate", "x");
    idY = logGetVarId("stateEstimate", "y");
    idZ = logGetVarId("stateEstimate", "z");
    idSef = paramGetVarId("sound", "effect");
    idWrgb = paramGetVarId("colorLedBot", "wrgb8888");

    paramVarId_t xVelMax = paramGetVarId("posCtlPid", "xVelMax");
    paramVarId_t yVelMax = paramGetVarId("posCtlPid", "yVelMax");
    paramVarId_t zVelMax = paramGetVarId("posCtlPid", "zVelMax");
    paramSetFloat(xVelMax, 2.0f);
    paramSetFloat(yVelMax, 2.0f);
    paramSetFloat(zVelMax, 2.0f);

    p2pRegisterCB(p2pReceive);

    paramSetInt(idSef, 12);

    bool prevEnabled = true;

    while(1) {
        if (!appEnabled) {
            if (prevEnabled) {
                grasped = false;
                armed = false;
                attemptScore = 0.0f;
                paramSetInt(idWrgb, LED_OFF);
                commanderRelaxPriority();
                prevEnabled = false;
            }
            vTaskDelay(M2T(100));
            continue;
        }
        prevEnabled = true;

        bool flying = supervisorIsFlying();
        float height = logGetFloat(idZ);   // read current altitude

        // ----- AUTO LAND CONDITION -----
        if (armed && !grasped && supervisorIsFlying() && height < landHeight) {
            paramSetInt(idWrgb, LED_RED);

            if (!landing_init) {
                setpointX = logGetFloat(idX);
                setpointY = logGetFloat(idY);
                landing_target_z = height;
                landing_init = true;
            }

            if (height > 0.05f) {
                landing_target_z -= 0.003f;
                if (landing_target_z < 0.0f) landing_target_z = 0.0f;
                setAbsolutePosition(&setpoint, setpointX, setpointY, landing_target_z, 0.0f);
                commanderSetSetpoint(&setpoint, 3);
                vTaskDelay(M2T(20));
            } else {
                DEBUG_PRINT("LANDED, disarming\n");
                supervisorRequestArming(false);
                armed = false;
                landing_init = false;
                memset(&setpoint, 0, sizeof(setpoint_t));
                commanderSetSetpoint(&setpoint, 3);
                vTaskDelay(M2T(500));
            }
            continue;
        }
        landing_init = false;

        if (grasped) {

            // Arm once at grasp moment
            if (!armed) {
                supervisorRequestCrashRecovery(true);
                if (supervisorRequestArming(true)) {
                    armed = true;
                    DEBUG_PRINT("ARMED\n");
                    vTaskDelay(M2T(500));
                }
            }

            // Only send setpoints once actually armed
            if (armed) {
                setpointX = lastPkt.x + graspRange * lastPkt.dx;
                setpointY = lastPkt.y + graspRange * lastPkt.dy;
                setpointZ = lastPkt.z + graspRange * lastPkt.dz;

                setAbsolutePosition(&setpoint, setpointX, setpointY, setpointZ, 0.0f);
                commanderSetSetpoint(&setpoint, 3);
            }

            if (xTaskGetTickCount() - lastPacketTime > M2T(lossTimeout)) {
                grasped = false;
                attemptScore = 0.0f;
                paramSetInt(idWrgb, LED_OFF);
                DEBUG_PRINT("RELEASED (lost wand signal)\n");
            }
        }
        else {
            if (armed && flying) {
                // -> HOVER IN PLACE (app-armed flight, wand lost)
                setAbsolutePosition(&setpoint, setpointX, setpointY, setpointZ, 0.0f);
                commanderSetSetpoint(&setpoint, 3);
            }
            // When not armed: send nothing, cfclient has full control
            paramSetInt(idWrgb, LED_OFF);
        }
        vTaskDelay(M2T(100));
    }
}

PARAM_GROUP_START(receiver)

/**
 * @brief Max distance [m] from the drone to the wand line for a packet to count as a valid grasp attempt
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, graspDist, &graspDist)

/**
 * @brief Accumulated score (0-100) required to trigger a grasp
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, graspThres, &graspThres)

/**
 * @brief Score added per packet received while within graspDist
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, buildRate, &buildRate)

/**
 * @brief Time [ms] without packets from the wand before the grasp is released
 */
PARAM_ADD(PARAM_UINT32 | PARAM_PERSISTENT, lossTimeout, &lossTimeout)

/**
 * @brief Altitude [m] below which the drone lands automatically when the wand is released
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, landHeight, &landHeight)

/**
 * @brief Set to 0 to disable the receiver app and give cfclient full control
 */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, appEnabled, &appEnabled)

PARAM_GROUP_STOP(receiver)

#endif // BUILD_RECEIVER_APP
