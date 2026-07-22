"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Fly a Distance (PID on Position)
Integrate forward velocity into position and PID to a target distance,
while a proportional term keeps altitude.
"""

import drone_core
import drone_utils as uav_utils

# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
TARGET_DIST = 4.0    # meters forward
TARGET_HEIGHT = 3.0  # hold launch height
KP = 0.15
KI = 0.0
KD = 0.5    # strong velocity damping to avoid overshoot
PITCH_LIMIT = 0.5
ALT_KP = 0.12
THROTTLE_LIMIT = 0.5
MIN_TRAVEL = 5.0   # fly at least this long before checking 'arrived'
SETTLE_SPEED = 0.25  # must slow below this to count as arrived
HOLD_TIME = 1.5
INT_CLAMP = 3.0      # anti-windup limit on the integral
TOL = 0.3

# -- Module-level state -----------------------------------------------------
_pos = 0.0
_err_int = 0.0
_prev_err = 0.0
_t = 0.0
_hold = 0.0
_done = False
_dist = 0.0

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########
    output = 0.0

    P = kp * err
    I = ki * err_int
    D = kd * err_dot
    output = P + I + D

    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _pos, _err_int, _prev_err, _t, _hold, _done, _dist
    _pos = 0.0
    _err_int = 0.0
    _prev_err = 0.0
    _t = 0.0
    _hold = 0.0
    _done = False
    _dist = 0.0


def update(drone):
    global _pos, _err_int, _prev_err, _t, _hold, _done, _dist
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # There is no direct (x, z) readout, so estimate forward distance by dead reckoning:
    # integrate the forward component of drone.physics.get_linear_velocity() over time.
    # PID that distance to TARGET_DIST for the pitch command (clamped to PITCH_LIMIT), and
    # use a proportional term (ALT_KP) on height to hold TARGET_HEIGHT. Count as arrived
    # only after MIN_TRAVEL, once speed drops below SETTLE_SPEED for HOLD_TIME. See the
    # README (Key terms) for dead reckoning and the PID law.

    dt = drone.get_delta_time()
    _t += dt
    altitude = drone.physics.get_altitude()
    forward_v = drone.physics.get_linear_velocity()[2]  # forward speed in m/s
    _dist += forward_v * dt  # forward dist

    alt_err = TARGET_HEIGHT - altitude
    forward_err = TARGET_DIST - _dist

    # clamp integral
    _err_int += forward_err * dt
    _err_int = max(-INT_CLAMP, min(INT_CLAMP, _err_int))

    # pid update
    output = pid_control(forward_err, _err_int, (forward_err - _prev_err) / dt, KP, KI, KD)
    _prev_err = forward_err  # update prev error
    pitch = max(-PITCH_LIMIT, min(PITCH_LIMIT, output))
    throt = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, ALT_KP * alt_err))
    drone.flight.send_pcmd(pitch, 0, 0, throt)
    
    print(f"Forward dist: {_dist:2f}/{TARGET_DIST}  |  Forward v: {forward_v:2f}/{SETTLE_SPEED}")

    if _t >= MIN_TRAVEL and abs(forward_v) < SETTLE_SPEED:
        print(f"Forward dist within tolerance: {_dist:.2f}/{TARGET_DIST:.2f} meters, speed: {forward_v:.2f} m/s")
        _hold += dt
        if _hold >= HOLD_TIME:
            _done = True


    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Fly a Distance (PID on Position)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
