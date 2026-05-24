#include "choose_app.h"
#ifdef BUILD_WAND_APP

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"

// ---------------------------------------------------------------------------
// Note frequencies (Hz)
// ---------------------------------------------------------------------------
#define _OFF  0
#define _C4   261
#define _E4   329
#define _G4   392
#define _C5   523
#define _D5   587
#define _E5   659
#define _Gb5  739
#define _G5   783
#define _A5   880
#define _Bb5  932
#define _B5   987
#define _C6   1046
#define _Db6  1109
#define _D6   1175
#define _B4   493

// ---------------------------------------------------------------------------
// Durations at BPM=120 (Q = 500 ms)
// ---------------------------------------------------------------------------
#define _Q     500
#define _H     (_Q * 2)
#define _W     (_Q * 4)
#define _E     (_Q / 2)
#define _DQ    (_Q * 3 / 2)
#define _DH    (_H * 3 / 2)

// ---------------------------------------------------------------------------
// Note table
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t freq;    // Hz; 0 = rest
    uint16_t dur_ms;  // ms; 0 = end of melody
} WNote;

// Two-beep pattern repeated until sensors are calibrated
static const WNote startupBeep[] = {
    {_C5, 300}, {_OFF, 120}, {_C5, 300}, {_OFF, 700},
    {_OFF, 0},
};

// Wizard theme
static const WNote wizardTheme[] = {
    {_B4,  _Q},
    {_E5,  _Q},  {_E5,  _E},              // E5 DQ
    {_G5,  _E},  {_Gb5, _Q},  {_E5, _H},
    {_B5,  _Q},  {_A5,  _H},  {_A5, _Q}, // A5 DH
    {_Gb5, _H},  {_E5,  _Q},
    {_G5,  _Q},  {_G5,  _E},              // G5 DQ
    {_Gb5, _E},  {_D5,  _Q},  {_D5, _H}, {_D5, _Q}, // D5 DH
    {_B4,  _Q},
    {_OFF, 0},
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
typedef enum {
    SND_NONE = 0,
    SND_STARTUP_LOOP,
    SND_MELODY,
} SoundRequest;

static volatile SoundRequest sndRequest = SND_NONE;
static volatile bool sndShouldStop = false;
static volatile bool sndPlaying = false;

static uint8_t melodyEnabled = 0;

#define SND_EFFECT_OFF    0
#define SND_EFFECT_BYPASS 12   // bypass effect: plays sound.freq continuously

static paramVarId_t soundEffectId;
static paramVarId_t soundFreqId;


// Sleep in 10 ms slices so sndShouldStop is noticed within one slice.
static void sleepInterruptible(uint16_t ms)
{
    while (ms > 0 && !sndShouldStop) {
        uint16_t chunk = ms > 10 ? 10 : ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        ms -= chunk;
    }
}

static void playNote(const WNote *n)
{
    if (n->freq == 0) {
        paramSetInt(soundEffectId, SND_EFFECT_OFF);
        sleepInterruptible(n->dur_ms);
    } else {
        uint16_t gap_ms = n->dur_ms / 20;
        if (gap_ms < 8) gap_ms = 8;
        uint16_t on_ms  = n->dur_ms - gap_ms;

        paramSetInt(soundFreqId,   n->freq);
        paramSetInt(soundEffectId, SND_EFFECT_BYPASS);
        sleepInterruptible(on_ms);

        paramSetInt(soundEffectId, SND_EFFECT_OFF);
        if (!sndShouldStop) {
            vTaskDelay(pdMS_TO_TICKS(gap_ms));
        }
    }
}

static void soundTask(void *param)
{
    (void)param;
    while (1) {
        if (sndRequest == SND_STARTUP_LOOP) {
            for (int i = 0; startupBeep[i].dur_ms != 0; i++) {
                if (sndRequest != SND_STARTUP_LOOP) break;
                playNote(&startupBeep[i]);
            }
            paramSetInt(soundEffectId, SND_EFFECT_OFF);

        } else if (sndRequest == SND_MELODY) {
            sndRequest = SND_NONE;
            sndPlaying = true;
            for (int i = 0; wizardTheme[i].dur_ms != 0; i++) {
                if (sndShouldStop) break;
                playNote(&wizardTheme[i]);
            }
            paramSetInt(soundEffectId, SND_EFFECT_OFF);
            sndShouldStop = false;
            sndPlaying = false;

        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void wand_sounds_init(void)
{
    soundEffectId = paramGetVarId("sound", "effect");
    soundFreqId   = paramGetVarId("sound", "freq");
    xTaskCreate(soundTask, "WAND_SND", 256, NULL, 2, NULL);
    sndRequest = SND_STARTUP_LOOP;
}

void wand_sounds_stop_startup(void)
{
    sndRequest = SND_NONE;
    paramSetInt(soundEffectId, SND_EFFECT_OFF);
}

void wand_sounds_play_melody(void)
{
    if (melodyEnabled && !sndPlaying && sndRequest == SND_NONE) {
        sndShouldStop = false;
        sndRequest = SND_MELODY;
    }
}

void wand_sounds_stop(void)
{
    sndShouldStop = true;
}

PARAM_GROUP_START(wand)
/**
 * @brief Enable Wizard melody on button press (0=off, 1=on)
 */
PARAM_ADD(PARAM_UINT8, melodyEnabled, &melodyEnabled)
PARAM_GROUP_STOP(wand)

#endif // BUILD_WAND_APP
