#include "trajectory.h"
#include <math.h>

void trajectory_get(float t, RcCommand *rc) {
    rc->roll = 0.0f;
    rc->pitch = 0.0f;
    rc->yaw = 0.0f;
    rc->throttle = 0.55f;

    if (t < 2.0f) {
        rc->throttle = 0.55f;
    } else if (t < 4.0f) {
        rc->throttle = 1.0f;
    } else if (t < 5.0f) {
        rc->throttle = 0.7f;
        rc->pitch = -1.0f;
    } else if (t < 6.0f) {
        rc->throttle = 0.55f;
    } else if (t < 8.0f) {
        rc->throttle = 0.6f;
        rc->roll = 1.0f;
    } else if (t < 9.0f) {
        rc->throttle = 0.55f;
    } else if (t < 11.0f) {
        float local = t - 9.0f;
        rc->throttle = (local < 1.0f) ? (1.0f - local * 0.7f) : (0.3f + (local - 1.0f) * 0.7f);
        rc->pitch = -1.0f;
    } else if (t < 13.0f) {
        float local = t - 11.0f;
        rc->throttle = local * 0.5f;
        rc->pitch = 1.0f;
    } else if (t < 15.0f) {
        float local = t - 13.0f;
        float phase = fmodf(local, 0.5f);
        rc->throttle = (phase < 0.25f) ? 1.0f : 0.3f;
    } else if (t < 18.0f) {
        rc->throttle = 1.0f;
    } else if (t < 19.0f) {
        rc->throttle = 0.55f;
    } else {
        float local = t - 19.0f;
        rc->throttle = 0.55f - local * 0.5f;
    }
}
