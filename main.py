import math
import time
import cv2
import numpy as np
import pigpio
from dronekit import connect, VehicleMode, Command
from picamera import PiCamera
from picamera.array import PiRGBArray
from pymavlink import mavutil


# --------------------------------------------------
# -------------- FUNCTIONS
# --------------------------------------------------
# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    """
    Arm the vehicle and takeoff
    """
    while not iha.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    iha.mode = VehicleMode("GUIDED")
    iha.armed = True

    while not iha.armed: time.sleep(1)

    print("Taking Off")
    iha.simple_takeoff(altitude)

    while True:
        v_alt = iha.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


def clear_mission(iha):
    """
    Clear the current mission.
    """
    cmds = iha.commands
    iha.commands.clear()
    iha.flush()

    # After clearing the mission you MUST re-download the mission from the iha
    # before iha.commands can be used again
    # (see https://github.com/dronekit/dronekit-python/issues/230)
    cmds = iha.commands
    cmds.download()
    cmds.wait_ready()


def download_mission(iha):
    """
    Download the current mission from the iha.
    """
    cmds = iha.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.


def get_current_mission(iha):
    """
    Downloads the mission and returns the wp list and number of WP

    Input:
        iha

    Return:
        n_wp, wpList
    """

    print("Downloading mission")
    download_mission(iha)
    missionList = []
    n_WP = 0
    for wp in iha.commands:
        missionList.append(wp)
        n_WP += 1

    return n_WP, missionList


def add_last_waypoint_to_mission(  # --- Adds a last waypoint on the current mission file
        iha,  # --- iha object
        wp_Last_Latitude,  # --- [deg]  Target Latitude
        wp_Last_Longitude,  # --- [deg]  Target Longitude
        wp_Last_Altitude):  # --- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the iha
    cmds = iha.commands
    cmds.download()
    cmds.wait_ready()

    # Save the iha commands to a list
    missionlist = []
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    # Write the modified mission and flush to the iha
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)


def ChangeMode(iha, mode):
    while iha.mode != VehicleMode(mode):
        iha.mode = VehicleMode(mode)
        time.sleep(0.5)
    return True


def velocity(velocity_x, velocity_y, yaw_rate, velocity_z, iha):
    """
    Give velocity command to vehicle
    """
    msg = iha.message_factory.set_position_target_local_ned_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                   0b0000011111000111, 0, 0, 0, velocity_x, velocity_y,
                                                                   velocity_z, 0, 0, 0, 0, math.radians(yaw_rate))
    iha.send_mavlink(msg)
    time.sleep(0.1)


def descend(target_alt):
    """
    Basic descend to target altitude
    """
    while iha.location.global_relative_frame.alt >= target_alt:
        velocity(0, 0, 0, 1, iha)
        time.sleep(0.3)
        print("Current altitude: {}".format(iha.location.global_relative_frame.alt))
    velocity(0, 0, 0, 0, iha)


def descend_lidar(target_alt):
    """
    Basic descend to target altitude with lidar sensor
    """
    velocity(0, 0, 0, 0.2, iha)
    while iha.rangefinder.distance >= target_alt:
        time.sleep(0.2)
        print("lidar: ", iha.rangefinder.distance)
    velocity(0, 0, 0, 0, iha)


def descend_slow(target_alt):
    """
    Descend to target altitude with dynamic sensor switching based on accuracy of sensor
    """
    while True:
        first_value = iha.rangefinder.distance
        if first_value - 2 < iha.location.global_relative_frame.alt < iha.rangefinder.distance + 2:
            old_value = first_value + 1
            current_value = first_value
            break
        else:
            time.sleep(0.5)
            print("bekliyorum")

    while current_value >= target_alt * 1.06:
        velocity(0, 0, 0, 0.2, iha)
        new_value = iha.rangefinder.distance

        upper_limit = current_value + 1
        lower_limit = current_value - 1
        if upper_limit >= new_value >= lower_limit:
            old_value = current_value
            current_value = new_value
            print(f"lidar {current_value:.2f}: ")
            print(f"Relative: {iha.location.global_relative_frame.alt:.2f} ")
            print("------------------------------------------")
            velocity(0, 0, 0, 0.2, iha)
        else:
            print(f"Incorrect value: {new_value:.2f} | range {lower_limit:.2f} - {upper_limit:.2f}")
            velocity(0, 0, 0, -0.05, iha)
        time.sleep(0.1)
    time_counter = 0
    while time_counter < 10:
        print(f"bekle lidar {current_value:.2f}: ")
        if iha.rangefinder.distance > target_alt + 0.20:
            velocity(0, 0, 0, 0.05, iha)
        elif iha.rangefinder.distance < target_alt - 0.20:
            velocity(0, 0, 0, -0.2, iha)
        elif target_alt + 0.2 > iha.rangefinder.distance > target_alt - 0.20:
            velocity(0, 0, 0, 0, iha)
        time_counter = time_counter + 1
        time.sleep(0.1)


def ascend(target_alt):
    """
    Basic ascend to target altitude
    """
    while iha.location.global_relative_frame.alt <= target_alt * 0.94:
        velocity(0, 0, 0, -1, iha)
        print("Current altitude: {}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    velocity(0, 0, 0, 0, iha)


class telemetric:
    def __init__(self, fps, h_velo, v_velo, h_acc, v_acc):
        self.fps = fps
        # self.coord = coord
        self.h_velo = h_velo
        self.v_velo = v_velo
        self.h_acc = h_acc
        self.v_acc = v_acc

pho = telemetric(0, 0, 0, 0, 0)


def color_filter(image, color):
    """
    Filters colors in HSV space
    """
    # Red lower and upper thresholds for HSV color space
    if color == "RED":
        lower = np.array([134, 95, 171])
        upper = np.array([179, 193, 255])
    elif color == "BLUE":
        lower = np.array([80, 95, 20])
        upper = np.array([120, 255, 255])
        # lower = np.array([70, 60, 140])
        # upper = np.array([180, 255, 255])

    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(image, lower, upper)  # Simple thresholding

    return mask


def find_avg(mask, img, threshold=3):
    """
    Finds and returns the mass center of given binary image
    """
    per = mask.mean(axis=0).mean(axis=0) * 100 / 255
    print(f"%{per:.2f} of image is filtered color")
    if per > threshold:
        M = cv2.moments(mask)
        x_avg = int(M["m10"] / M["m00"])
        y_avg = int(M["m01"] / M["m00"])
        print("(", x_avg, ",", y_avg, ")")
        start_point = (int(img.shape[1] / 2), int(img.shape[0] / 2))
        img = cv2.arrowedLine(img, start_point, (x_avg, y_avg), (0, 255, 0), 2, tipLength=0.5)
        return x_avg, y_avg
    else:
        print("Kirmizi alan bulunamadi.")
        return -1, -1


def scaleit(image):
    global scale_percent
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
    return image


def setup():
    # GUI state variables
    global showWindow, showHelp, showFullScreen, scale_percent, gui_width, gui_height, scale_percent
    showWindow = 4
    showHelp = True
    showFullScreen = False
    scale_percent = 10  # res scale of proccessing image
    gui_width, gui_height = 960, 540  # GUI res
    windowName = "Video"
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, gui_width, gui_height)
    cv2.moveWindow(windowName, 0, 0)
    cv2.setWindowTitle(windowName, "GTU AQUA")


def monitoring(img1, img2, img3, pho):
    global vidBuf, displayBuf, showWindow, showHelp, showFullScreen, gui_width, gui_height
    font = cv2.FONT_HERSHEY_PLAIN
    windowName = "Video"
    helpText = "'Esc' to Quit, '1' for Camera Feed, '2' for Circle Detection, '3' for All Stages. '4' to hide help"

    black_back = np.zeros((int(gui_height / 2), int(gui_width / 2), 3), dtype="uint8")
    high_black = np.zeros((gui_height, gui_width, 3), dtype="uint8")

    if showWindow == 4:
        dis_x = int(gui_width / 2) + 20
        dis_y = int(gui_height / 2) + 40
        diff_y = 35
        diff_x = 20
        img1 = cv2.resize(img1, (int(gui_width / 2), int(gui_height / 2)))
        img2 = cv2.resize(img2, (int(gui_width / 2), int(gui_height / 2)))
        vidBuf1 = np.concatenate((img1, img2), axis=1)
        img3 = cv2.resize(img3, (int(gui_width / 2), int(gui_height / 2)))

        vidBuf2 = np.concatenate((img3, black_back), axis=1)
        vidBuf = np.concatenate((vidBuf1, vidBuf2), axis=0)

    if showWindow == 1:  # Show Camera Frame
        displayBuf = img1
    elif showWindow == 2:  # Show Detected Circles
        displayBuf = img2
    elif showWindow == 3:  # Show Telemetry
        displayBuf = high_black
        dis_x = 30
        dis_y = 40
        diff_y = 70
        diff_x = 20


    elif showWindow == 4:  # Show All Stages
        displayBuf = vidBuf

    if showWindow == 4 or showWindow == 3:
        cv2.putText(displayBuf, "Telemetry:", (dis_x, dis_y), font, 2.0, (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(displayBuf, "Telemetry:", (dis_x, dis_y), font, 2.0, (240, 240, 240), 1, cv2.LINE_AA)

        cv2.putText(displayBuf, "FPS: " + str(pho.fps), (diff_x + dis_x, diff_y + dis_y), font, 1.5, (32, 32, 32), 4,
                    cv2.LINE_AA)
        cv2.putText(displayBuf, "FPS: " + str(pho.fps), (diff_x + dis_x, diff_y + dis_y), font, 1.5, (240, 240, 240),
                    1, cv2.LINE_AA)
        # cv2.putText(displayBuf, "Coordinate(x, y, z): " + str(aqua.coord), (700, 430), font, 1.5, (32, 32, 32), 4,
        # cv2.LINE_AA)
        # cv2.putText(displayBuf, "Coordinate(x, y, z): " + str(aqua.coord), font, 1.5, (240, 240, 240), 1, cv2.LINE_AA)

        cv2.putText(displayBuf, "Horizontal Velocity: " + str(pho.h_velo), (diff_x + dis_x, 2 * diff_y + dis_y), font,
                    1.5, (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(displayBuf, "Horizontal Velocity: " + str(pho.h_velo), (diff_x + dis_x, 2 * diff_y + dis_y), font,
                    1.5, (240, 240, 240), 1, cv2.LINE_AA)

        cv2.putText(displayBuf, "Vertical Velocity: " + str(pho.v_velo), (diff_x + dis_x, 3 * diff_y + dis_y), font,
                    1.5, (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(displayBuf, "Vertical Velocity: " + str(pho.v_velo), (diff_x + dis_x, 3 * diff_y + dis_y), font,
                    1.5, (240, 240, 240), 1, cv2.LINE_AA)

        cv2.putText(displayBuf, "Horizontal Acceleration: " + str(pho.h_acc), (diff_x + dis_x, 4 * diff_y + dis_y),
                    font, 1.5, (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(displayBuf, "Horizontal Acceleration: " + str(pho.h_acc), (diff_x + dis_x, 4 * diff_y + dis_y),
                    font, 1.5, (240, 240, 240), 1, cv2.LINE_AA)

        cv2.putText(displayBuf, "Vertical Acceleration: " + str(pho.v_acc), (diff_x + dis_x, 5 * diff_y + dis_y), font,
                    1.5, (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(displayBuf, "Vertical Acceleration: " + str(pho.v_acc), (diff_x + dis_x, 5 * diff_y + dis_y), font,
                    1.5, (240, 240, 240), 1, cv2.LINE_AA)

    if showHelp:
        cv2.putText(displayBuf, helpText, (11, 20), font, 1.0, (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(displayBuf, helpText, (10, 20), font, 1.0, (240, 240, 240), 1, cv2.LINE_AA)

    cv2.imshow(windowName, displayBuf)
    key = cv2.waitKey(5)

    if key == 27:  # Check for ESC key
        cv2.destroyAllWindows()
        exit()
    elif key == 49:  # 1 key, show frame
        cv2.setWindowTitle(windowName, "Camera Feed")
        showWindow = 1
    elif key == 50:  # 2 key, show circles
        cv2.setWindowTitle(windowName, "Canny Edge Detection")
        showWindow = 2
    elif key == 51:  # 3 key, show telemetry
        cv2.setWindowTitle(windowName, "Telemetry")
        showWindow = 3

    elif key == 52:  # 4 key, show Stages
        cv2.setWindowTitle(windowName, "Camera, Detected Circles, Segmented, Telemetry")
        showWindow = 4

    elif key == 53:  # 5 key, toggle help
        showHelp = not showHelp
    elif key == 74:  # Toggle fullscreen; This is the F3 key on this particular keyboard
        # Toggle full screen mode
        if not showFullScreen:
            cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        showFullScreen = not showFullScreen


def find_loc(spill=False, load=False, color="RED", target_sens=5):
    """
    Finds the center of image at given color and commands vehicle to center it.
    """
    target_sensivity = target_sens / 100
    setup()
    global scale_percent
    cam_x = 1280
    cam_y = 720

    res_x = cam_x * scale_percent / 100
    res_y = cam_y * scale_percent / 100
    drone_orta_count = 0

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        start_time = time.time()
        frame_array = frame.array
        image = frame_array.copy()
        start_point = (int(res_x / 2 - res_y * target_sensivity), int(res_y / 2 - res_y * target_sensivity))
        end_point = (int(res_x / 2 + res_y * target_sensivity), int(res_y / 2 + res_y * target_sensivity))
        if scale_percent < 100:
            image = scaleit(image)

        filtered_image = color_filter(image, color)
        output3 = cv2.bitwise_and(image, image, mask=filtered_image)
        target_x, target_y = find_avg(filtered_image, image)
        image = cv2.rectangle(image, start_point, end_point, (255, 0, 0), 2)
        pho.fps = round(1 / (time.time() - start_time), 1)
        monitoring(frame_array, image, output3, pho)

        if target_x == -1 and target_y == -1:
            velocity(1, 0, 0, 0, iha)  # kırmızıyı algılamak için x ekseninde yavaşça ilerler
            print("Target not found")
        else:
            if (
                    res_x / 2 - res_y * target_sensivity) <= target_x <= res_x / 2 + res_y * target_sensivity and res_y / 2 - res_y * target_sensivity <= target_y <= res_y / 2 + res_y * target_sensivity:
                if not spill and color == "RED":
                    velocity(0, 0, 0, 0, iha)
                    rawCapture.truncate(0)
                    return
                elif spill and color == "RED":
                    """
                    After centering process vehicle descends and runs vehicle's servo
                    """
                    velocity(0, 0, 0, 0, iha)
                    print("IHA ortalanmistir")
                    descend_slow(3)
                    time.sleep(5)
                    print("servo calisiyor...")
                    pi.set_servo_pulsewidth(18, 1000)  # safe clockwise
                    time.sleep(rotate_time)
                    pi.set_servo_pulsewidth(18, 1540)  # safe clockwise
                    time.sleep(4)
                    pi.set_servo_pulsewidth(18, 2000)  # safe clockwise
                    time.sleep(rotate_time)
                    pi.set_servo_pulsewidth(18, 1540)  # safe clockwise
                    pi.write(18, 0)
                    rawCapture.truncate(0)
                    return
                elif load and color == "BLUE":
                    velocity(0, 0, 0, 0, iha)
                    print("IHA ortalanmistir")
                    time.sleep(5)
                    print("su alimi icin alcaliniliyor...")
                    descend_slow(2.2)
                    ascend(9)
                    time.sleep(5)
                    rawCapture.truncate(0)
                    return

            else:  # hedefe doğru vektör hızları hesaplar ve bunların toplamı velocity = 0.5 olucak şekilde oranlar
                """
                Calculation of velocity in X and Y axis
                """
                diff_x = target_x - res_x / 2
                diff_y = -(target_y - res_y / 2)
                k = 0.2 / (diff_x ** 2 + diff_y ** 2) ** (1 / 2)
                velocity_y = diff_x * k
                velocity_x = diff_y * k
                print(f"Velo x: {velocity_x:.2f}", f"Velo y: {velocity_y:.2f}")
                velocity(velocity_x, velocity_y, 0, 0, iha)

        rawCapture.truncate(0)


# --------------------------------------------------
# -------------- INITIALIZE
# --------------------------------------------------
# -- Setup
pi = pigpio.pi()
connection_string = "/dev/ttyACM0"
cam_x = 1280
cam_y = 720
camera = PiCamera()
camera.resolution = (cam_x, cam_y)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(cam_x, cam_y))
gnd_speed = 6  # [m/s]
mode = 'GROUND'
current_WP = -1
# --------------------------------------------------
# -------------- CONNECTION
# --------------------------------------------------
# -- Connect to the iha
iha = connect(connection_string, wait_ready=True)

"""
Backups the some parameters to restore after the script
"""
wpnav_speed = iha.parameters['WPNAV_SPEED']
wpnav_accel = iha.parameters['WPNAV_ACCEL']
# --------------------------------------------------
# -------------- MAIN FUNCTION
# --------------------------------------------------
# init value
GET_WATER = 9
SPILL_WATER = 12

rotate_time = 5

flag_get_water = True
flag_spill_Water = True

"""
State Machine to operate through mission
Mission:
    Vehicle has to complete parkour and get water from pool at given approximate location.
    Then vehicle has to find desired red circle pool in the area and will proceed to unload the water on it.
    Vehicle has to operate autonomously without any human intervention.
"""
mode = 'GROUND'
while True:

    if mode == 'GROUND':
        # --- Wait until a valid mission has been uploaded
        n_WP, missionList = get_current_mission(iha)
        time.sleep(1)
        if n_WP > 0:
            print("A valid mission has been uploaded: takeoff!")
            mode = 'TAKEOFF'

    elif mode == 'TAKEOFF':

        # -- Add a fake waypoint at the end of the mission
        add_last_waypoint_to_mission(iha, iha.location.global_relative_frame.lat,
                                     iha.location.global_relative_frame.lon,
                                     iha.location.global_relative_frame.alt)
        print("Home waypoint added to the mission")
        # -- Takeoff
        arm_and_takeoff(10)
        print(iha.gps_0)

        # -- Change mode, set the ground speed
        iha.groundspeed = gnd_speed
        iha.parameters['WPNAV_SPEED'] = 800
        iha.parameters['WPNAV_ACCEL'] = 250
        print("Changing to AUTO")
        ChangeMode(iha, "AUTO")
        mode = 'MISSION'

    elif mode == 'MISSION':
        old_WP = current_WP
        current_WP = iha.commands.next
        if iha.commands.next == GET_WATER and flag_get_water:
            mode = "GET_WATER"
        if iha.commands.next == SPILL_WATER and flag_spill_Water:
            mode = "SPILL_WATER"

        if iha.commands.next == iha.commands.count:
            print("Final waypoint reached <3")
            clear_mission(iha)
            print("Mission deleted")
            mode = "BACK"

        if old_WP != current_WP:
            print("Current WP: %d of %d " % (iha.commands.next, iha.commands.count))
            time.sleep(3)
            print(">> Groundspeed: %s" % round(iha.groundspeed, 1))

    elif mode == "GET_WATER":
        print("State Machine: GET_WATER")
        iha.mode = VehicleMode("GUIDED")
        descend(7)
        find_loc(color="BLUE", load=True)
        iha.parameters['WPNAV_SPEED'] = 400
        iha.parameters['WPNAV_ACCEL'] = 120
        print("Changing to AUTO")
        ChangeMode(iha, "AUTO")
        mode = 'MISSION'
        flag_get_water = False

    elif mode == "SPILL_WATER":
        print("State Machine: SPILL_WATER")
        iha.mode = VehicleMode("GUIDED")
        find_loc(target_sens=8)
        descend(5)
        find_loc(spill=True)
        ascend(9)
        iha.parameters['WPNAV_SPEED'] = 800
        iha.parameters['WPNAV_ACCEL'] = 250
        print("Changing to AUTO")
        ChangeMode(iha, "AUTO")
        mode = 'MISSION'
        flag_spill_Water = False

    elif mode == "BACK":
        iha.parameters['WPNAV_SPEED'] = wpnav_speed
        iha.parameters['WPNAV_ACCEL'] = wpnav_accel
        time.sleep(20)
