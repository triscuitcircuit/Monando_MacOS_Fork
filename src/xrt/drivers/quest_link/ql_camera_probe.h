// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stdbool.h>
#include <stdint.h>


#define QL_CAMERA_PROBE_RESTART_PAUSE_NS 300000000LL
#define QL_CAMERA_PROBE_MAX_RESTARTS 3
#define QL_CAMERA_PROBE_RESTART_GRACE_NS 2000000000LL

struct ql_camera_probe_state {
    bool attempted;
    bool active;
    int64_t stop_ns;
    bool restart_pending;
    int64_t restart_ns;
    int64_t grace_until_ns;
    int restarts;
};

enum ql_camera_probe_action {
    QL_CAMERA_PROBE_NONE,
    QL_CAMERA_PROBE_START,
    QL_CAMERA_PROBE_STOP,
    QL_CAMERA_PROBE_RESTART_STOP,
    QL_CAMERA_PROBE_RESTART_START,
};

static inline enum ql_camera_probe_action
ql_camera_probe_recover(struct ql_camera_probe_state *state, int64_t now_ns, bool decoder_stalled)
{
    if (state->restart_pending) {
        if (now_ns < state->restart_ns) return QL_CAMERA_PROBE_NONE;
        state->restart_pending = false;
        state->grace_until_ns = now_ns + QL_CAMERA_PROBE_RESTART_GRACE_NS;
        return QL_CAMERA_PROBE_RESTART_START;
    }
    if (decoder_stalled && now_ns >= state->grace_until_ns && state->restarts < QL_CAMERA_PROBE_MAX_RESTARTS) {
        state->restart_pending = true;
        state->restart_ns = now_ns + QL_CAMERA_PROBE_RESTART_PAUSE_NS;
        state->restarts++;
        return QL_CAMERA_PROBE_RESTART_STOP;
    }
    return QL_CAMERA_PROBE_NONE;
}

static inline enum ql_camera_probe_action
ql_camera_probe_step(struct ql_camera_probe_state *state, int64_t now_ns, bool session_ready, bool decoder_stalled,
                     int64_t duration_ns)
{
    if (state->active && (!session_ready || now_ns >= state->stop_ns)) {
        state->active = false;
        state->restart_pending = false;
        return QL_CAMERA_PROBE_STOP;
    }
    if (state->active) {
        const enum ql_camera_probe_action action = ql_camera_probe_recover(state, now_ns, decoder_stalled);
        if (action != QL_CAMERA_PROBE_NONE) return action;
    }
    if (!state->attempted && session_ready) {
        state->attempted = state->active = true;
        state->stop_ns = now_ns + duration_ns;
        return QL_CAMERA_PROBE_START;
    }
    return QL_CAMERA_PROBE_NONE;
}


static inline enum ql_camera_probe_action
ql_camera_portal_step(struct ql_camera_probe_state *state, int64_t now_ns, bool session_ready, bool requested,
                      bool decoder_stalled)
{
    if (state->active && (!session_ready || !requested)) {
        state->active = false;
        state->restart_pending = false;
        state->stop_ns = now_ns;
        return QL_CAMERA_PROBE_STOP;
    }
    if (state->active) return ql_camera_probe_recover(state, now_ns, decoder_stalled);
    if (session_ready && requested && (!state->attempted || now_ns - state->stop_ns >= QL_CAMERA_PROBE_RESTART_PAUSE_NS)) {
        state->attempted = state->active = true;
        state->restarts = 0;
        state->grace_until_ns = 0;
        return QL_CAMERA_PROBE_START;
    }
    return QL_CAMERA_PROBE_NONE;
}
