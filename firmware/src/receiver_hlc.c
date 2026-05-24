#include "choose_app.h"
#ifdef BUILD_RECEIVER_HLC_APP

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
#include "supervisor.h"
#include "crtp_commander_high_level.h"

#define DEBUG_MODULE "ReceiverHLC"
#include "debug.h"

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
typedef enum {
    STATE_IDLE,      // on ground, waiting for grasp
    STATE_FOLLOWING, // tracking wand position
    STATE_HOVER,     // wand lost, holding last position
    STATE_LANDING,   // auto-landing
} ReceiverState;

static ReceiverState state = STATE_IDLE;
static TickType_t stateTimer = 0;

// ---------------------------------------------------------------------------
// Grasp state (same logic as receiver.c)
// ---------------------------------------------------------------------------
static float attemptScore = 0.0f;
static bool  grasped      = false;
static uint32_t lastPacketTime = 0;
static float graspRange   = 0.0f;
static bool  armed        = false;

// Last wand-computed setpoint (used for hover hold)
static float holdX = 0.0f, holdY = 0.0f, holdZ = 0.0f;

// Packet format: [id][x,y,z][dx,dy,dz]
typedef struct {
    uint8_t id;
    float x, y, z;
    float dx, dy, dz;
} WandLinePacket;

static WandLinePacket lastPkt;

static logVarId_t idX, idY, idZ;

static paramVarId_t idSef;
static paramVarId_t idWrgb;

#define LED_OFF    0
#define LED_GREEN  0x00008000
#define LED_YELLOW 0x00808000
#define LED_RED    0x00800000

// ---------------------------------------------------------------------------
// Runtime-settable persistent parameters
// ---------------------------------------------------------------------------
static float    graspDist      = 0.15f;
static float    graspThres     = 30.0f;
static float    buildRate      = 2.0f;
static uint32_t lossTimeout    = 2000;
static float    landHeight     = 0.3f;
static uint8_t  appEnabled     = 1;

// HLC-specific parameters
static float gotoDuration    = 0.2f;   // s, trajectory duration per goTo call

// ---------------------------------------------------------------------------
// P2P receive (same geometry as receiver.c)
// ---------------------------------------------------------------------------
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
 *    to that absolute position via crtpCommanderHighLevelGoTo.
 *
 * 4. Release: no packet for lossTimeout ms → grasp cleared, drone hovers.
 */
void p2pReceive(P2PPacket *p)
{
    if (!appEnabled) return;
    if (p->port != 0x01) return;
    if (p->size != 1 + 6*sizeof(float)) return;

    lastPacketTime = xTaskGetTickCount();

    WandLinePacket pkt;
    pkt.id = p->data[0];
    memcpy(&pkt.x,  &p->data[1],    sizeof(float));
    memcpy(&pkt.y,  &p->data[1+4],  sizeof(float));
    memcpy(&pkt.z,  &p->data[1+8],  sizeof(float));
    memcpy(&pkt.dx, &p->data[1+12], sizeof(float));
    memcpy(&pkt.dy, &p->data[1+16], sizeof(float));
    memcpy(&pkt.dz, &p->data[1+20], sizeof(float));

    lastPkt = pkt;

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
    float vnorm = vector_norm(vx, vy, vz);
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

// ---------------------------------------------------------------------------
// Main task
// ---------------------------------------------------------------------------
void appMain()
{
    DEBUG_PRINT("RECEIVER-HLC ACTIVE - Listening to Wand...\n");

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

    while (1) {
        // ---- appEnabled = 0: hand control to cfclient ----
        if (!appEnabled) {
            if (state != STATE_IDLE) {
                crtpCommanderHighLevelStop();
                if (armed) {
                    supervisorRequestArming(false);
                    armed = false;
                }
                state   = STATE_IDLE;
                grasped = false;
                attemptScore = 0.0f;
                paramSetInt(idWrgb, LED_OFF);
            }
            vTaskDelay(M2T(100));
            continue;
        }

        float height = logGetFloat(idZ);
        bool  flying = supervisorIsFlying();

        switch (state) {

        // ------ IDLE: wait for grasp, arm, then follow immediately ------
        case STATE_IDLE:
            if (!grasped) {
                paramSetInt(idWrgb, LED_OFF);  // p2pReceive sets YELLOW on in-range packets
            }
            if (grasped) {
                supervisorRequestCrashRecovery(true);
                if (supervisorRequestArming(true)) {
                    armed = true;
                    DEBUG_PRINT("ARMED\n");
                    vTaskDelay(M2T(500));
                    holdX = logGetFloat(idX);
                    holdY = logGetFloat(idY);
                    holdZ = logGetFloat(idZ);
                    state = STATE_FOLLOWING;
                }
            }
            break;

        // ------ FOLLOWING: track wand at 50 Hz ------
        case STATE_FOLLOWING:
            if (grasped) {
                float tx = lastPkt.x + graspRange * lastPkt.dx;
                float ty = lastPkt.y + graspRange * lastPkt.dy;
                float tz = lastPkt.z + graspRange * lastPkt.dz;
                holdX = tx; holdY = ty; holdZ = tz;
                crtpCommanderHighLevelGoTo(tx, ty, tz, 0.0f, gotoDuration, false);
            }
            // Check loss timeout
            if (xTaskGetTickCount() - lastPacketTime > M2T(lossTimeout)) {
                grasped      = false;
                attemptScore = 0.0f;
                paramSetInt(idWrgb, LED_OFF);
                state = STATE_HOVER;
                DEBUG_PRINT("RELEASED (timeout)\n");
            }
            break;

        // ------ HOVER: re-grab or auto-land ------
        case STATE_HOVER:
            if (grasped) {
                state = STATE_FOLLOWING;
                break;
            }
            paramSetInt(idWrgb, LED_OFF);
            if (armed && flying && height < landHeight) {
                paramSetInt(idWrgb, LED_RED);
                crtpCommanderHighLevelLand(0.0f, 3.0f);
                stateTimer = xTaskGetTickCount() + M2T(4000);
                state = STATE_LANDING;
            }
            break;

        // ------ LANDING: wait until on ground ------
        case STATE_LANDING:
            if (height < 0.05f || xTaskGetTickCount() >= stateTimer) {
                DEBUG_PRINT("LANDED\n");
                supervisorRequestArming(false);
                armed        = false;
                grasped      = false;
                attemptScore = 0.0f;
                state = STATE_IDLE;
                paramSetInt(idWrgb, LED_OFF);
                vTaskDelay(M2T(500));
            }
            break;
        }

        vTaskDelay(M2T(100));
    }
}

// ---------------------------------------------------------------------------
// Parameters — same group name as receiver.c so write_params_all.py works
// on both versions unchanged.
// ---------------------------------------------------------------------------
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

/**
 * @brief Trajectory duration [s] per goTo command while following the wand (lower = more responsive)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, gotoDuration, &gotoDuration)

PARAM_GROUP_STOP(receiver)

#endif // BUILD_RECEIVER_HLC_APP
