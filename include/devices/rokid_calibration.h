#pragma once

#include "imu.h"

#include <stdbool.h>
#include <stdint.h>

/* -----------------------------------------------------------------------
 * Rokid Max yaw-drift calibration
 *
 * The Rokid SDK performs its own sensor fusion on-glass and delivers
 * orientation as quaternions via GAME_ROTATION_EVENT.  Despite this,
 * the yaw axis accumulates a small, roughly-constant drift over time.
 *
 * This module measures that drift rate once (55 s, jump-aware) and
 * stores it to disk so that subsequent runs load it instantly.  At
 * runtime it builds a correction quaternion that is multiplied onto
 * the NWU pose quaternion each frame before it is ingested.
 *
 * Drift file path: $XDG_CONFIG_HOME/xr_driver/rokid_yaw_drift.txt
 * (falls back to ~/.config/xr_driver/rokid_yaw_drift.txt)
 * ----------------------------------------------------------------------- */

/* Calibration parameters — tune if needed */
#define ROKID_CAL_WARMUP_MS        5000   /* fusion settle time before sampling */
#define ROKID_CAL_TOTAL_SAMPLES    5500   /* samples @ ~10 ms each = 55 s       */
#define ROKID_CAL_SAMPLE_MS        10     /* ms between calibration samples      */
#define ROKID_CAL_JUMP_THRESH_DEG  0.5    /* ignore deltas larger than this      */
#define ROKID_CAL_TOTAL_MS         55000.0 /* TOTAL_SAMPLES * SAMPLE_MS          */

typedef struct {
    double   drift_rate_rad_per_ms; /* measured or loaded yaw drift            */
    bool     is_calibrated;
    uint64_t tare_time_ms;          /* monotonic ms at end of calibration/load */
} rokid_calibration_type;

/* ----- public API ----------------------------------------------------- */

/**
 * rokid_calibration_init
 *
 * Call once from rokid_block_on_device(), before the GlassWaitEvent loop.
 * Blocks for up to ~60 s on first run (calibration), or returns immediately
 * when a saved drift file exists.
 *
 * @param cal      Pointer to a zero-initialised rokid_calibration_type.
 * @param instance The GlassEvent instance (already open).
 * @param handle   The registered GAME_ROTATION_EVENT handle.
 * @param adj_quat The NWU adjustment quaternion applied in rokid.c.
 */
void rokid_calibration_init(rokid_calibration_type *cal,
                            void *instance,
                            void *handle,
                            imu_quat_type adj_quat);

/**
 * rokid_calibration_apply
 *
 * Given the NWU quaternion produced this frame, return a yaw-corrected
 * quaternion.  Roll and pitch are untouched.
 *
 * @param cal              Calibration state (must be initialised).
 * @param nwu_quat         This frame's NWU orientation quaternion.
 * @param current_time_ms  Current monotonic millisecond timestamp.
 * @return                 Drift-corrected NWU quaternion.
 */
imu_quat_type rokid_calibration_apply(const rokid_calibration_type *cal,
                                      imu_quat_type nwu_quat,
                                      uint64_t current_time_ms);

/**
 * rokid_calibration_invalidate
 *
 * Deletes the saved drift file and marks calibration as stale.
 * Does NOT block or re-measure — the fresh measurement happens
 * automatically the next time rokid_calibration_init() is called
 * (i.e. after the device reconnects following reset_calibration()).
 *
 * Call this from reset_calibration() in driver.c when the Rokid
 * driver is active.
 */
void rokid_calibration_invalidate(rokid_calibration_type *cal);

/** Monotonic millisecond clock (CLOCK_MONOTONIC). */
uint64_t rokid_cal_get_millis(void);
