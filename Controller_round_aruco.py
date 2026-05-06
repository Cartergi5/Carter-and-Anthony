#!/usr/bin/env python3
"""
controller.py — UB Racer student controller template.

This is YOUR file.  Implement your AI/control logic in the sections below.
The racerlib backend handles all communication with server.py and the car.

─── Quick start ──────────────────────────────────────────────────────────────

Normal mode (requires a running host):
    python server.py --host https://HOST_IP:8086
    python controller.py --port CLIENT_PORT

Dev mode (no host required — use your own camera URL):
    python server.py --dev
    python controller.py --dev --port CLIENT_PORT

─── How it works ─────────────────────────────────────────────────────────────

1. conn.run() connects to server.py and blocks until stopped.
2. The system calls your callbacks as events arrive:
     on_session_start  → a car has been assigned to you
     on_session_end    → the session is over
     on_telemetry      → fresh car data arrived (~10 Hz); call conn.drive() here
     on_system_status  → queue / availability update (~1 Hz)
     on_confirm_required → you are next; confirm within timeoutSec or lose your spot
     on_estop            → toggle whether the controller is in state of emergency stop     
3. Call conn.join() when you are ready to enter the queue.
4. Call conn.drive(<steering>, <throttle>) to drive the car.
5. Call conn.stop() when you are done.

Publishing notices to your client webpage:
conn.notice(<severity level>, "<some message>"
	Valid Severity Levels:
	ub_utils.SEVERITY_EMERGENCY       ub_utils.SEVERITY_ALERT       ub_utils.SEVERITY_CRITICAL
	ub_utils.SEVERITY_ERROR           ub_utils.SEVERITY_WARNING     ub_utils.SEVERITY_NOTICE
	ub_utils.SEVERITY_INFO            ub_utils.SEVERITY_DEBUG
Ex:  conn.notice(ub_utils.SEVERITY_INFO, "You are connected")
"""

import argparse

from lib.racerlib import Racer

import ub_camera, ub_utils
import cv2
import numpy as np
import time

# Check version and get update notification:
ub_camera.checkVersion()

# ── CLI args ──────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description="UB Racer controller")
parser.add_argument("--dev",    action="store_true",
                                help="Dev mode — no host or car required")
parser.add_argument("--port",   default=8443, 
								help="Port used by client server")
parser.add_argument("--server", default=None,
                                help="Override server.py URL (auto-detected if omitted)")
args = parser.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
#  YOUR CODE — implement your control logic below
# ══════════════════════════════════════════════════════════════════════════════

# ── Algo params ───────────────────────────────────────────────────────────────
# The browser (index.html Algo Params panel) is the canonical source of truth.
# These values are automatically pushed to controller.py at the start of every
# session (dev or real), overwriting whatever is here.
#
# Edit these only as a fallback for headless/autonomous operation (no browser).
# For normal use, set your defaults in the browser — they persist via localStorage.
#
# All color values below are in cv2 ranges (pre-converted by the browser):
#   hue:        [0, 179]   (half of the UI's [0, 360])
#   saturation: [0, 255]   (scaled from the UI's [0, 100])
#   value:      [0, 255]   (scaled from the UI's [0, 100])

_params = {
    "cropTop":          0,
    "cropBottom":       0,
    "color":            {"h": 90, "s": 255, "v": 255},
    "hueTolerance":     {"min": 5,  "max": 175},
    "satTolerance":     {"min": 0,  "max": 255},
    "valTolerance":     {"min": 0,  "max": 255},
    "maxThrottle":      30,
    "steeringPerPixel": 0.5,
    "deadZonePixels":   10,

    # new ball settings
    "minArea":          400,
    "minCircularity":   0.65,
    "followMode":       "ball",   # "line" or "ball"

    # ArUco goal settings
    "goalArucoID":      14,
    "goalAssist":       True,
    "goalAlignGain":    0.5,
}

isDriving = False   # set by E-Stop button; True = driving enabled

cam = {}

# Initialize steering/throttle limits with 0 values:
throttleLimits = {"min": 0, "max": 0}

STEERING_MIN = -100  # full left
STEERING_MAX =  100  # full right
last_ball_seen_time = 0
last_ball_radius = 0
last_ball_x = None
PUSH_MEMORY_SEC = 2.0



def detect_goal_aruco(display):
    """Detect the ArUco tag used as the goal marker.

    Returns:
        goal_point: (gx, gy) if the selected tag is found, otherwise None
        display: frame with marker drawings added
    """
    gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is None:
        return None, display

    cv2.aruco.drawDetectedMarkers(display, corners, ids)

    goal_id = _params.get("goalArucoID", 0)
    print("Detected IDs:", ids, "Goal ID:", goal_id)

    for i, marker_id in enumerate(ids.flatten()):
        if int(marker_id) == int(goal_id):
            pts = corners[i][0]
            gx = int(np.mean(pts[:, 0]))
            gy = int(np.mean(pts[:, 1]))

            cv2.circle(display, (gx, gy), 8, (255, 0, 0), -1)
            cv2.putText(display, "GOAL", (gx + 10, gy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            tag_width = np.linalg.norm(pts[0] - pts[1])
            tag_height = np.linalg.norm(pts[1] - pts[2])
            tag_size = (tag_width + tag_height) / 2

            return (gx, gy, tag_size), display

    return None, display


def my_pipeline(frame):
    h, w, d = frame.shape

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_color = np.array([
        _params['hueTolerance']['min'],
        _params['satTolerance']['min'],
        _params['valTolerance']['min']
    ])
    upper_color = np.array([
        _params['hueTolerance']['max'],
        _params['satTolerance']['max'],
        _params['valTolerance']['max']
    ])

    mask = cv2.inRange(hsv, lower_color, upper_color)

    # crop top/bottom
    mask[0:_params['cropTop'], 0:w] = 0
    mask[h-_params['cropBottom']:h, 0:w] = 0

    # clean up noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    display = frame.copy()
    display[0:_params['cropTop'], 0:w] = 100
    display[h-_params['cropBottom']:h, 0:w] = 100

    # Detect the ArUco tag that marks the goal.
    # If the goal is not visible, the car will still follow the ball normally.
    goal_point, display = detect_goal_aruco(display)

    target_found = False
    cx, cy = None, None

    mode = "ball"

    if mode == "ball":
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < _params.get("minArea", 400):
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity >= _params.get("minCircularity", 0.65):
                if area > best_area:
                    best_area = area
                    best_contour = cnt

        if best_contour is not None:
            ((x, y), radius) = cv2.minEnclosingCircle(best_contour)
            M = cv2.moments(best_contour)

            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                target_found = True
                
                global last_ball_seen_time, last_ball_radius, last_ball_x
                last_ball_seen_time = time.time()
                last_ball_radius = radius
                last_ball_x = cx

                cv2.circle(display, (cx, cy), int(radius), (0, 255, 0), 2)
                cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(display, "BALL", (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    else:
        # line mode: keep your original centroid behavior
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            target_found = True

            cv2.circle(display, (cx, cy), 20, (0, 0, 255), -1)
            cv2.putText(display, "LINE", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if target_found:
        # Normal behavior: aim directly at the ball.
        # Goal-assist behavior: if the ArUco goal is visible too, aim slightly
        # behind the ball so the car lines up to push the ball toward the goal.
        ball_is_captured = radius > 80

        if ball_is_captured and goal_point is not None and _params.get("goalAssist", True):
            goal_x, goal_y, goal_size = goal_point
            align_gain = _params.get("goalAlignGain", 0.5)

            # If the goal is to the right of the ball, target left of the ball.
            # If the goal is to the left of the ball, target right of the ball.
            target_x = cx - align_gain * (goal_x - cx)

            cv2.line(display, (cx, cy), (goal_x, goal_y), (255, 0, 0), 2)
            cv2.circle(display, (int(target_x), cy), 8, (255, 255, 0), -1)
            cv2.putText(display, "ALIGN", (int(target_x) + 10, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            error = target_x - (w / 2)
        else:
            error = cx - (w / 2)

        if abs(error) <= _params['deadZonePixels']:
            error = 0

        steering = max(STEERING_MIN, min(STEERING_MAX,
                        -error * _params['steeringPerPixel']))

        if radius > 80:
            # VERY CLOSE → pushing zone
            base_throttle = 12   # keep moving to push
        elif radius > 60:
            # close → slow approach
            base_throttle = 10
        elif radius > 30:
            # mid distance
            base_throttle = 14
        else:
            # far away
            base_throttle = 20

        throttle = base_throttle * max(0.4, 1 - abs(steering)/100)

        print(f"Ball radius={radius:.1f} steering={steering:.1f} throttle={throttle:.1f} isDriving={isDriving}")

        if isDriving:
            conn.drive(steering, throttle)

    else:
        # Ball is lost. Use the ArUco tag to keep pushing toward the goal.
        if goal_point is not None:
            goal_x, goal_y, goal_size = goal_point

            # Stop when the goal tag looks big enough in the camera
            if goal_size > 140:
                steering = 0
                throttle = 0

                cv2.putText(display, "AT GOAL: STOP", (30, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                error = goal_x - (w / 2)

                steering = max(STEERING_MIN, min(STEERING_MAX,
                                -error * _params['steeringPerPixel']))

                throttle = 10

                cv2.putText(display, "BALL LOST: PUSH TO GOAL", (30, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if isDriving:
                conn.drive(steering, throttle)

        else:
            # No ball and no goal tag visible, so stop.
            if isDriving:
                conn.drive(0, 0)

    return display
	
	
	
def on_session_start(data: dict) -> None:
	"""Called once when a driving session begins.
	
	Use this to initialise per-session state (reset PID integrals, clear
	buffers, etc.).
	
	Useful keys in data:
		data["carID"]                         — which car you have
		data["timeLimitSec"]                  — seconds until the session ends
		data["mjpegURL"]                      — camera stream URL
	"""
	print(f"[session] Started — car: {data.get('carID')}")
	conn.notice(ub_utils.SEVERITY_INFO,  f"Session Started - Car: {data.get('carID')}")
	conn.notice(ub_utils.SEVERITY_DEBUG, f"[DEBUG] Session Start Data: {data}")
	
	# ── YOUR CODE HERE ──────────────────────────────────────────────────── #
	global cam
	
	port = ub_utils.findOpenPort(8000, options=range(8000,8011))
	
	device = data['mjpegURL']
	if isinstance(device, str) and device.isdigit():
		device = int(device)
	cam[data['carID']] = ub_camera.CameraUSB(device=device)

	if data.get('cameraIntrinsics'):
		for res, params in data['cameraIntrinsics'].items():
			cam[data['carID']].setIntrinsics(res, **params)

	cam[data['carID']].frameProcessor = my_pipeline
	cam[data['carID']].start(startStream=True, port=port)
		
	# streamURL is like 'https://192.168.2.14:8000/stream.mjpg'
	conn.set_camera_url(cam[data['carID']].streamURL)    # tells the browser where to display
	conn.notice(ub_utils.SEVERITY_INFO, f"Your camera stream is available at {cam[data['carID']].streamURL}")
	
def on_session_end(data: dict) -> None:
	"""Called when the session ends for any reason.

	data["reason"] is one of: "timeout", "user_exit", "admin_boot",
	"car_disconnect".

	To re-queue automatically after each session, call conn.join() here.
	"""
	print(f"[session] Ended — reason: {data.get('reason')}")     
	conn.notice(ub_utils.SEVERITY_INFO, f"Session Ended — reason: {data.get('reason')}")     
	conn.notice(ub_utils.SEVERITY_DEBUG, f"[DEBUG] Session End Data: {data}")

	# ── YOUR CODE HERE ──────────────────────────────────────────────────── #
	global cam
	cam[data['carID']].stop()

	# Let any in-process camera frames clear
	time.sleep(1)

	# Zero the steering and throttle
	conn.drive(0, 0)

	# Uncomment to re-queue automatically after each session:
	# conn.join()


def on_telemetry(data: dict) -> None:
	"""Called at ~10 Hz with the latest car data during a session.

	Call conn.drive(steering, throttle) here to move the car.
	Not called in dev mode (no car connected).

	data keys:
		carID, timestamp,
		steering (current, degrees),
		throttle (current, percent),
		compass  (heading in degrees, or None if unavailable)
	"""
	# ── YOUR CODE HERE ──────────────────────────────────────────────────── #
	#
	# Example — drive straight at 20 % throttle:
	#   conn.drive(0.0, 20.0)
	#
	# Example — simple compass-based heading hold:
	#   if data.get("compass") is not None:
	#       error    = TARGET_HEADING - data["compass"]
	#       steering = max(-30, min(30, error * 0.5))
	#       conn.drive(steering, 25.0)
	pass


def on_system_status(data: dict) -> None:
	"""Called ~1 Hz with queue and car availability info.

	Useful for monitoring your position before a session starts.

	data keys: cars, globalQueuePosition, yourStatus, yourCarID
	"""
	# ── YOUR CODE HERE (optional) ────────────────────────────────────────── #
	pass


def on_params(data: dict) -> None:
	"""Called when the browser sends updated algorithm parameters.

	The browser Algo Params panel lets you tune these live without restarting
	controller.py.  Values arrive pre-converted to cv2 ranges (see _params
	above).
	"""
	global _params
	_params.update(data)

	conn.notice(ub_utils.SEVERITY_DEBUG, f"[DEBUG] params updated: {data}")      
	'''
	print(f"[params] color={data.get('color')}  "
		  f"maxThrottle={data.get('maxThrottle')}  "
		  f"steer/px={data.get('steeringPerPixel')}")
	'''	  


def on_estop(is_driving: bool) -> None:
	"""Called when the browser E-Stop button is toggled.

	is_driving=False  — E-Stop activated; racerlib has already issued drive(0,0).
	is_driving=True   — driving re-enabled.
	"""
	global isDriving
	isDriving = is_driving
	state = "ENABLED" if is_driving else "STOPPED"
	conn.notice(ub_utils.SEVERITY_WARNING if not is_driving else ub_utils.SEVERITY_INFO,
				f"E-Stop: Driving {state}")


def on_confirm_required(data: dict) -> None:
	"""Called when you have reached the front of the queue.

	You must confirm within data["timeoutSec"] seconds or you will be moved
	to the back of the queue.

	The default behaviour (auto-confirm) is active when you pass
	on_confirm_required=None to Racer().  Override it here if you need
	manual or conditional confirmation.
	"""
	print(f"[queue] Confirm required for {data.get('carName')} — auto-confirming.")
	conn.confirm()


# ══════════════════════════════════════════════════════════════════════════════
#  SETUP — create the connection and start
# ══════════════════════════════════════════════════════════════════════════════

conn = Racer(
    on_session_start=on_session_start,
    on_session_end=on_session_end,
    on_telemetry=on_telemetry,
    on_system_status=on_system_status,
    on_confirm_required=on_confirm_required,
    on_params=on_params,
    on_estop=on_estop,
    dev=args.dev,
    port=args.port,
    server=args.server,
)

if __name__ == "__main__":
	# conn.run() blocks until conn.stop() is called or Ctrl-C.
	#
	# To join the queue:
	#   - Use the browser UI (click "Join Queue"), OR
	#   - Call conn.join() from inside on_session_end to re-queue automatically, OR
	#   - In interactive/Jupyter mode, call conn.start() then conn.join().
	#
	# In dev mode, start a session via the browser form (when available) or
	# by calling conn.start_dev_session(camera_url) from a Jupyter cell
	# after conn.start().
	try:
		conn.run()
	finally:
		for c in list(cam.values()):
			try:
				c.stop()
			except Exception:
				pass
