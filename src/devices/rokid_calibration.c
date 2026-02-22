#include "devices/rokid_calibration.h"
#include "logging.h"
#include "sdks/rokid.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -----------------------------------------------------------------------
 * Drift file path helpers
 * Mirrors the pattern used by src/files.c / src/config.c in this repo:
 * prefer $XDG_CONFIG_HOME, fall back to $HOME/.config.
 * ----------------------------------------------------------------------- */

#define ROKID_CAL_FILENAME "rokid_yaw_drift.txt"
#define ROKID_CAL_SUBDIR   "xr_driver"

static void get_drift_file_path(char *buf, size_t buflen) {
    const char *xdg = getenv("XDG_CONFIG_HOME");
    if (xdg && xdg[0] != '\0') {
        snprintf(buf, buflen, "%s/%s/%s", xdg, ROKID_CAL_SUBDIR, ROKID_CAL_FILENAME);
    } else {
        const char *home = getenv("HOME");
        if (!home) home = "/tmp";
        snprintf(buf, buflen, "%s/.config/%s/%s", home, ROKID_CAL_SUBDIR, ROKID_CAL_FILENAME);
    }
}

/* -----------------------------------------------------------------------
 * Monotonic clock
 * ----------------------------------------------------------------------- */

uint64_t rokid_cal_get_millis(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}

/* -----------------------------------------------------------------------
 * Disk I/O
 * ----------------------------------------------------------------------- */

static double load_drift(void) {
    char path[512];
    get_drift_file_path(path, sizeof(path));

    FILE *f = fopen(path, "r");
    if (!f) return NAN;

    double val = NAN;
    if (fscanf(f, "%lf", &val) != 1) val = NAN;
    fclose(f);
    return val;
}

static void save_drift(double drift) {
    char path[512];
    get_drift_file_path(path, sizeof(path));

    FILE *f = fopen(path, "w");
    if (!f) {
        log_error("rokid_cal: could not save drift to %s\n", path);
        return;
    }
    fprintf(f, "%.15f", drift);
    fclose(f);
    log_message("rokid_cal: drift saved to %s\n", path);
}

/* -----------------------------------------------------------------------
 * Yaw extraction from a NWU quaternion
 *
 * The driver works in NWU after applying adjustment_quat.
 * In NWU the yaw (rotation about the Up/Z axis) is extracted with the
 * standard ZYX Euler formula — the same component used for heading.
 * This matches the role of `attitude_frd_rad().z` in your Rust code
 * (FRD z == yaw, NWU z == yaw; only the sign convention differs, but
 * we measure and correct within the same convention so signs cancel).
 * ----------------------------------------------------------------------- */

static double quat_to_nwu_yaw_rad(imu_quat_type q) {
    /* ZYX yaw from a quaternion: atan2(2(wz + xy), 1 - 2(y² + z²)) */
    double siny_cosp = 2.0 * ((double)q.w * q.z + (double)q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * ((double)q.y * q.y + (double)q.z * q.z);
    return atan2(siny_cosp, cosy_cosp);
}

/* -----------------------------------------------------------------------
 * Extract a NWU quaternion from a GAME_ROTATION_EVENT frame.
 * Mirrors the logic already in rokid_block_on_device().
 * ----------------------------------------------------------------------- */

static bool get_nwu_quat_from_event(void *instance, void *handle,
                                    imu_quat_type adj_quat,
                                    imu_quat_type *out_quat) {
    struct EventData ed;
    if (!GlassWaitEvent(instance, handle, &ed, 1000)) return false;

    struct RotationData rd = ed.rotation;
    imu_quat_type raw = {
        .w = rd.Q[3],
        .x = rd.Q[0],
        .y = rd.Q[1],
        .z = rd.Q[2]
    };
    *out_quat = multiply_quaternions(raw, adj_quat);
    return true;
}

/* -----------------------------------------------------------------------
 * Core calibration routine — direct translation of your Rust algorithm.
 *
 * Returns the drift rate in rad/ms.
 * ----------------------------------------------------------------------- */

static double run_calibration(void *instance, void *handle, imu_quat_type adj_quat) {
    log_message("rokid_cal: --- JUMP-AWARE YAW CALIBRATION (%d s) ---\n",
                (int)(ROKID_CAL_TOTAL_MS / 1000));
    log_message("rokid_cal: KEEP GLASSES STILL. Filtering fusion jumps...\n");

    /* Warmup: drain events for WARMUP_MS, let the SDK fusion settle */
    uint64_t warmup_end = rokid_cal_get_millis() + ROKID_CAL_WARMUP_MS;
    while (rokid_cal_get_millis() < warmup_end) {
        struct EventData discard;
        GlassWaitEvent(instance, handle, &discard, 100);
    }

    /* Grab the first clean yaw sample */
    imu_quat_type q;
    while (!get_nwu_quat_from_event(instance, handle, adj_quat, &q)) {
        /* retry until we get one */
    }
    double prev_y = quat_to_nwu_yaw_rad(q);
    double total_clean_drift = 0.0;

    for (int i = 0; i < ROKID_CAL_TOTAL_SAMPLES; i++) {
        usleep(ROKID_CAL_SAMPLE_MS * 1000);

        if (!get_nwu_quat_from_event(instance, handle, adj_quat, &q)) {
            /* Timed-out frame — skip without advancing prev_y */
            continue;
        }

        double current_y = quat_to_nwu_yaw_rad(q);
        double delta = current_y - prev_y;

        /* Wrap-around correction (identical to your Rust logic) */
        if      (delta >  M_PI) delta -= 2.0 * M_PI;
        else if (delta < -M_PI) delta += 2.0 * M_PI;

        /* Jump filter: only accumulate small, plausible drift deltas */
        double delta_deg = delta * (180.0 / M_PI);
        if (fabs(delta_deg) < ROKID_CAL_JUMP_THRESH_DEG) {
            total_clean_drift += delta;
        } else if (i % 100 == 0) {
            log_message("rokid_cal: jump detected, ignoring frame %d "
                        "(delta %.3f deg)\n", i, delta_deg);
        }

        prev_y = current_y;

        if (i % 1000 == 0) {
            log_message("rokid_cal: progress %d/%d | accumulated %.4f deg\n",
                        i, ROKID_CAL_TOTAL_SAMPLES,
                        total_clean_drift * (180.0 / M_PI));
        }
    }

    double drift = total_clean_drift / ROKID_CAL_TOTAL_MS;
    save_drift(drift);
    log_message("rokid_cal: calibration complete. drift = %.15f rad/ms\n", drift);
    return drift;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void rokid_calibration_init(rokid_calibration_type *cal,
                            void *instance,
                            void *handle,
                            imu_quat_type adj_quat) {
    memset(cal, 0, sizeof(*cal));

    double loaded = load_drift();
    if (!isnan(loaded)) {
        cal->drift_rate_rad_per_ms = loaded;
        cal->is_calibrated         = true;
        log_message("rokid_cal: loaded drift %.12f rad/ms\n", loaded);
    } else {
        cal->drift_rate_rad_per_ms = run_calibration(instance, handle, adj_quat);
        cal->is_calibrated         = true;
    }

    cal->tare_time_ms = rokid_cal_get_millis();
}

imu_quat_type rokid_calibration_apply(const rokid_calibration_type *cal,
                                      imu_quat_type nwu_quat,
                                      uint64_t current_time_ms) {
    if (!cal->is_calibrated) return nwu_quat;

    /*
     * Total yaw drift accumulated since tare (radians).
     * Your Rust formula: corrected_yaw = drift_offset - frd_z
     * Here we apply the inverse sign because we're rotating the
     * quaternion in NWU space rather than computing a scalar Euler angle.
     * Subtracting accumulated drift = rotating by -drift around NWU Up (Z).
     */
    double elapsed_ms   = (double)(current_time_ms - cal->tare_time_ms);
    double drift_rad    = elapsed_ms * cal->drift_rate_rad_per_ms;

    /*
     * Build a pure-yaw correction quaternion: rotate by -drift_rad around
     * the NWU Up axis (Z), which cancels the accumulated sensor drift.
     *
     *   q_correction = { w = cos(-d/2), x = 0, y = 0, z = sin(-d/2) }
     */
    double half_angle = -drift_rad * 0.5;
    imu_quat_type correction = {
        .w = (float)cos(half_angle),
        .x = 0.0f,
        .y = 0.0f,
        .z = (float)sin(half_angle)
    };

    /*
     * Apply correction: corrected = correction ⊗ nwu_quat
     * Left-multiply so the correction is in world (NWU) space.
     */
    return multiply_quaternions(correction, nwu_quat);
}

void rokid_calibration_invalidate(rokid_calibration_type *cal) {
    char path[512];
    get_drift_file_path(path, sizeof(path));
    remove(path);
    cal->is_calibrated = false;
    log_message("rokid_cal: drift file removed — fresh calibration will run on next device connect\n");
}
