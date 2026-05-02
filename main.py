from multiprocessing import Process, Queue
from camera.capture import camera_loop
from vision.inference import vision_loop
from control.brain import control_loop
from hardware.arduino import hardware_loop
from web.server import run_server


if __name__ == "__main__":
	frame_q = Queue(maxsize=5)
	result_q = Queue()
	command_q = Queue()
	sensor_q = Queue()
	web_cmd_q = Queue()
	status_q = Queue()
	processes = [
		Process(target=camera_loop, args=(frame_q,)),
		Process(target=vision_loop, args=(frame_q, result_q)),
		Process(target=control_loop, args=(result_q, sensor_q, web_cmd_q,
		command_q, status_q)),
		Process(target=hardware_loop, args=(command_q, sensor_q)),
		Process(target=run_server, args=(web_cmd_q, status_q))
		]
	for p in processes:
		p.start()
	for p in processes:
		p.join()




import serial
import subprocess
import time
import re
import os

# ================= SERIAL =================
arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# ================= CAMERA CONTROL =================
def release_camera():
    print("🔓 Releasing camera safely...")
    os.system("sudo killall gst-launch-1.0 2>/dev/null")
    os.system("sudo fuser -k /dev/video0 2>/dev/null")
    time.sleep(2)

def start_camera():
    print("📷 Starting camera...")
    time.sleep(2)
    return subprocess.Popen(
        ["edge-impulse-linux-runner"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

def stop_camera(proc):
    if proc:
        print("🛑 Stopping camera...")
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except:
            proc.kill()
        release_camera()

# ================= START SYSTEM =================
process = start_camera()

# ================= PARAMETERS =================
CENTER_X = 48

LOCK_FRAMES_REQUIRED = 5
POSITION_TOLERANCE = 10
MIN_SIZE = 5   # FIXED (important)

# ================= STATE =================
state = "SEARCH"
vision_active = True

lock_counter = 0
current_x = None
current_y = None

locked_x = None
locked_y = None

nav_dist = 999
bin_full = False

print("🚀 AUTONOMOUS LITTER ROBOT STARTED")

# ================= MAIN LOOP =================
while True:

    # ---------------- READ CAMERA OUTPUT ----------------
    if process and process.poll() is None:
        line = process.stdout.readline()
    else:
        line = ""

    if line:
        print(line.strip())

    # ---------------- ULTRASONIC UPDATE ----------------
    if "DIST:" in line:
        try:
            nav_dist = float(line.split(":")[1])
            if nav_dist < 8:
                arduino.write(b'STOP\n')
        except:
            pass

    # ---------------- BIN FULL ----------------
    if "BIN_FULL" in line:
        bin_full = True

    # ================= RETURN HOME =================
    if bin_full and state != "RETURN_HOME":
        print("📦 BIN FULL → RETURN HOME")

        state = "RETURN_HOME"
        arduino.write(b'STOP\n')
        arduino.write(b'HOME\n')

        vision_active = False

    elif state == "RETURN_HOME":
        if nav_dist < 8:
            arduino.write(b'STOP\n')
        else:
            arduino.write(b'FORWARD\n')

    # ================= SEARCH MODE =================
    elif state == "SEARCH" and vision_active:

        if '"label":"black"' in line or '"label":"blue"' in line:

            x_match = re.search(r'"x":([0-9]+)', line)
            y_match = re.search(r'"y":([0-9]+)', line)
            w_match = re.search(r'"width":([0-9]+)', line)
            h_match = re.search(r'"height":([0-9]+)', line)

            if not (x_match and y_match and w_match and h_match):
                continue

            x = int(x_match.group(1))
            y = int(y_match.group(1))
            w = int(w_match.group(1))
            h = int(h_match.group(1))

            cx = x + w // 2
            cy = y + h // 2

            # STOP robot during detection
            arduino.write(b'STOP\n')

            # ================= FRAME STABILITY =================
            if current_x is None:
                current_x, current_y = cx, cy
                lock_counter = 1
            else:
                dx = abs(cx - current_x)
                dy = abs(cy - current_y)

                if dx < POSITION_TOLERANCE and dy < POSITION_TOLERANCE:
                    lock_counter += 1
                else:
                    current_x, current_y = cx, cy
                    lock_counter = 1

            print(f"Frames stable: {lock_counter}")

            # ================= LOCK =================
            if lock_counter >= LOCK_FRAMES_REQUIRED and w > MIN_SIZE:

                print("🔒 LOCKED → FREEZING CAMERA")

                locked_x = current_x
                locked_y = current_y

                arduino.write(b'STOP\n')
                arduino.write(f"TARGET:{locked_x},{locked_y}\n".encode())

                # 🔥 HARD CAMERA FREEZE
                stop_camera(process)
                process = None

                state = "PICK"

    # ================= PICK MODE =================
    elif state == "PICK":

        print("🤖 EXECUTING PICK")

        arduino.write(b'STOP\n')
        time.sleep(0.5)

        arduino.write(f"TARGET:{locked_x},{locked_y}\n".encode())
        time.sleep(1)

        arduino.write(b'PICK\n')
        time.sleep(2)

        arduino.write(b'DROP\n')
        time.sleep(2)

        arduino.write(b'HOME\n')
        time.sleep(2)

        print("✅ PICK COMPLETE → RESTART SYSTEM")

        # ================= RESTART CAMERA =================
        process = start_camera()

        # ================= RESET =================
        state = "SEARCH"
        vision_active = True
        lock_counter = 0
        current_x = None
        current_y = None
        locked_x = None
        locked_y = None