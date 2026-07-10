"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Blur & Edge Detection
Averaging blur then a Sobel edge-magnitude image.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

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
KERNEL_SIZE = 5
HOVER_TIME  = 3.0

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False

def reset():
    global _timer, _done
    _timer = 0.0
    _done  = False


def update(drone):
    global _timer, _done
    if _done:
        return True
    drone.flight.stop()   # hover in place
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: report the average edge strength in the downward image.
    #
    # Grayscale the image, blur it with a KERNEL_SIZE box filter so single-pixel noise
    # does not dominate, then use a Sobel filter (across and down) to measure how fast
    # brightness changes and combine the two directions into one edge magnitude. Print the
    # mean magnitude. Advance _timer and finish at HOVER_TIME. See the README (Key terms).

    img = drone.camera.get_downward_image()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.blur(gray, (KERNEL_SIZE, KERNEL_SIZE))

    # Sobel filter across and down
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=KERNEL_SIZE)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=KERNEL_SIZE) 

    # "The second argument cv2.CV_64F tells OpenCV to use 64-bit floats for the output. This matters because gradients can be negative, meaning intensity is decreasing, and if you use an unsigned 8-bit integer those negative values get clipped to zero and you lose half your edges. Computing in float and converting afterward keeps everything intact."

    # Combine two directions into 1 edge magnitude
    sobel = cv2.magnitude(sobel_x, sobel_y)
    sobel = np.uint8(np.clip(sobel, 0, 255))
    mean = np.mean(sobel)

    print(f'Sobel mean magnitude: {mean}')

    _timer += drone.get_delta_time()
    if _timer >= HOVER_TIME:
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
        print("Step 3: Blur & Edge Detection")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
