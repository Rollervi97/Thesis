//
// Created by ASUS on 07/12/2021.
//

#ifndef SYS_ID_WAVE_H
#define SYS_ID_WAVE_H

#include "std.h"
#include "math.h"

#include "paparazzi.h"


extern uint8_t wave_active;
extern uint8_t wave_axis;
extern pprz_t wave_amplitude;
extern float frequency_hz_;
extern float lag_rad_;

extern uint8_t wave_axis;

extern void sys_id_wave_init(void);
extern void sys_id_wave_run(void);

extern void sys_id_wave_frequency_hz_set(void);

// handlers for changing in GCS variables
extern void sys_id_wave_activate_handler(void);
extern void sys_id_wave_add_values(void);



#endif
