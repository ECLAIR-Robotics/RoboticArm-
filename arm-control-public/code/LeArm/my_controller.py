from email.mime import base
from lib2to3.pytree import convert
from math import atan, dist, sin, asin, cos, acos, atan2, degrees, radians, sqrt
import math
from multiprocessing.sharedctypes import Value
import time
from collections.abc import Sequence
from dataclasses import dataclass
from tkinter import E
from numpy import angle
import serial


HEADER = '>>'  # header to start messages to arduino with
SERVO_MOVE_CODE = '$'  # command code to move servo
SERVO_READ_CODE = '#'  # command code to read from servo
VOLT_READ_CODE = '%'  # command code to read voltage
SHOULDER_LENGTH = 4.25;
FOREARM_LENGTH = 3.5;
HAND_LENGTH = 6;


def clamp(x, low, high):
    if high < low:
        low, high = high, low

    return min(max(low, x), high)


@dataclass
class Servo:
    def __init__(self, servo_min: int, servo_max: int, angular_range: int,
                 max_speed: float = 0.09, init_position: float = 0.5, synced: bool = True):
        if servo_min >= servo_max:
            raise Exception('Servo min must be less than servo max!')

        self.min = servo_min
        self.max = servo_max
        self.range = servo_max - servo_min
        self.max_speed = max_speed
        self._delta = 0
        self._synced = synced
        self._angular_range = angular_range
        self._position = init_position

    @property
    def delta(self):
        return self._delta

    @property
    def synced(self):
        return self._synced

    @synced.setter
    def synced(self, new_state):
        self._synced = new_state

    @property
    def position_raw(self):
        return round(self.position * self.range + self.min)

    @position_raw.setter
    def position_raw(self, new_pos_raw):
        self.position = (new_pos_raw - self.min) / self.range

    @property
    def angle(self):
        return (self.position - 0.5) * self._angular_range

    @angle.setter
    def angle(self, new_angle):
        self.position = (new_angle / self._angular_range) + 0.5

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, new_pos):
        new_pos = clamp(new_pos, 0, 1)

        self._delta = new_pos - self.position
        self._position = new_pos
        self._synced = False


class Controller(Sequence):
    BAUDRATE = 9600
    TIMEOUT = .1
    SERVO_NAMES = {  # lol these names are kinda wack but idk what else to call them for now
        'base': 6,
        'shoulder': 5,
        'elbow': 4,
        'wrist': 3,
        'hand': 2,
        'claw': 1,
    }
    HOME_POSITION = 0.5  # position to set every servo to when homed
    HOME_TIME = 5000  # how long it takes to go to home positions when going to home

    def __init__(self, home=False, port='COM3', debug=False, simulation=False):
        if not simulation:
            self._arduino = serial.Serial(port=port, baudrate=self.BAUDRATE, timeout=self.TIMEOUT)
        else:
            self._arduino = None
            self.action_log = list()

        self._servos = {
            1: Servo(servo_min=600, servo_max=1380, angular_range=180, max_speed=2),  # actual max is 1800, be careful
            2: Servo(servo_min=400, servo_max=2500, angular_range=180, max_speed=1),
            3: Servo(servo_min=450, servo_max=2600, angular_range=180, max_speed=1),
            4: Servo(servo_min=400, servo_max=2600, angular_range=180, max_speed=1),
            5: Servo(servo_min=400, servo_max=2500, angular_range=180, max_speed=0.5),
            6: Servo(servo_min=400, servo_max=2500, angular_range=180, max_speed=0.5),
        }
        self._debug = debug
        self.simulation = simulation

        while not simulation:  # wait until arduino gives ready signal
            ready = self._arduino.read()

            if ready:
                break

            time.sleep(self.TIMEOUT)

        if self._debug:
            print ("ready")

        if home:
            self.home()

    def home(self, action_time=HOME_TIME):
        for servo_id, servo in self._servos.items():
            self._setPosition(servo_id, self.HOME_POSITION)

        self._updateArm(action_time)

    def openClamp (self):
        self.setAngle(1, 1000, -90)
    
    def closeClamp (self):
        self.setAngle(1, 1000, 90)

    # use inverse kinematics to set effector position using cartesian coordinates
    # param position is a tuple (x, y, z) where z is height
    # param hand_angle is the angle of the arm relative to the ground
    def setPositionIK(self, position, hand_angle=0, action_time=1000):
        x, y, z = position
        x = -x
        y = -y

        # find base angle
        base_angle =  degrees(atan2(y, x))

        # adjust target position to account for hand angle
        hand_delta_vertical = cos(radians(hand_angle+90)) * HAND_LENGTH
        hand_delta_horizontal = sin(radians(hand_angle+90)) * HAND_LENGTH
        if x != 0:
            x -= cos(radians(base_angle)) * hand_delta_horizontal
        y -= sin(radians(base_angle)) * hand_delta_horizontal
        z -= hand_delta_vertical
        
        distance = sqrt(pow(x, 2) + pow(y,2) + pow(z, 2))
        min_distance = sqrt (pow(FOREARM_LENGTH, 2) + pow(SHOULDER_LENGTH, 2))
        max_distance = SHOULDER_LENGTH + FOREARM_LENGTH
        if distance < min_distance or distance > max_distance:
            print (f"IK WARNING: Wrist target is {distance} inches away from base. " +
                    f"Must be between {min_distance} and {max_distance} inches away. " +
                    "Getting as close as possible.")
            distance = min(max(distance, min_distance), max_distance)

        # find angles of other two joints by solving SSS triangle
        try:
            shoulder_angle = 90 - degrees(asin(z/distance)) - self.solveTri (FOREARM_LENGTH, distance, SHOULDER_LENGTH)
            elbow_angle = 180 - self.solveTri (distance, SHOULDER_LENGTH, FOREARM_LENGTH)
        except Exception:
            print (f"IK COMMAND IGNORED: Position ({x}, {y}, {z}) is unreachable");
            return
        
        # converts hand angle from global to relative (exterior angles add to 360)
        hand_angle = -(hand_angle + 180 - (360 - elbow_angle - (shoulder_angle + 90)))
        if not self.simulation: # weird offset needed for real arm
            hand_angle -= 20
        if hand_angle > 90 or hand_angle < -90:
            print (f"IK COMMAND IGNORED: relative hand angle {hand_angle} deg is outside [-90, 90]");
            return

        base_angle -= 90 # atan2 treats angle 0 as right, shift it back after all trig calculations

        # allow positions behind arm to be reached by flipping angles
        if base_angle > 90 or base_angle < -90:
            base_angle = self.convert_90 (base_angle + 180)
            shoulder_angle = -shoulder_angle
            elbow_angle = -elbow_angle
            hand_angle = -hand_angle
            if not self.simulation: # weird offset needed for real arm
                hand_angle -= 20

        # move servos
        self.setAngles(action_time, base=base_angle, shoulder=shoulder_angle, 
                elbow=elbow_angle, wrist=hand_angle)

    # solves SSS triangle with law of cosines
    # sides are labeled counter-clockwise, returns angle A
    def solveTri (self, sideA, sideB, sideC):
        if sideA >= sideB + sideC or sideB >= sideA + sideC or sideC >= sideA + sideB:
            raise Exception ("Triangle cannot be calculated with given side lenghts. " +
                    "Does not adhere to triange inequality theorem.")
        return degrees(acos((pow(sideB, 2) + pow(sideC, 2) - pow(sideA, 2)) /
                        (2 * sideB * sideC)))

    # converts 360 degree coordinates to 90 degree
    def convert_90 (self, angle_360):
        if angle_360 > 90:
            angle_360 = angle_360 - 360;
        return angle_360

    def getPosition(self, servo_id):
        return self[servo_id]

    def setPosition(self, servo_id, action_time=1000, position=0.5):
        self._setPosition(servo_id, position)  # sets position logically
        self._updateArm(action_time)  # actually moves arm

    def setPositions(self, action_time=1000, **movements):
        for servo_id, position in movements.items():
            self._setPosition(servo_id, position)

        self._updateArm(action_time)

    def setPositionRaw(self, servo_id, action_time=1000, position_raw=1380):
        self._setPositionRaw(servo_id, position_raw)
        self._updateArm(action_time)

    def setPositionsRaw(self, action_time=1000, **movements_raw):
        for servo_id, position_raw in movements_raw.items():
            self._setPositionRaw(servo_id, position_raw)

        self._updateArm(action_time)

    def setAngle(self, servo_id, action_time=1000, angle=0):
        self._setAngle(servo_id, angle)
        self._updateArm(action_time)

    def setAngles(self, action_time=1000, **movements):
        for servo_id, angle in movements.items():
            self._setAngle(servo_id, angle)

        self._updateArm(action_time)

    def readPositions(self, *servo_ids):
        message = f'{len(servo_ids)}'
        for servo_id in servo_ids:
            message += f'{servo_id}'

        message = f'{SERVO_READ_CODE}{message}'
        self._write(message)
        time.sleep(0.05)
        return self._arduino.read_until('Q')

    def readVoltage(self):
        message = f'{VOLT_READ_CODE}'
        self._write(message)
        time.sleep(0.05)
        return self._arduino.read_until('Q')

    def logActions(self):
        if not self.simulation:
            return

        with open ("action_log.txt", "w") as log_file:
            log_file.writelines (self.action_log)

        print ("Actions Logged Successfully")

    def _write(self, x):
        x = HEADER + x
        if not self.simulation:
            if self._debug:
                print(f'sent: {bytes(x, "utf-8")}')
            self._arduino.write(bytes(x, 'utf-8'))
            

    def _read(self):
        return self._arduino.read_until('Q')

    def _updateArm(self, action_time=1000):  # sends command to update all unsynced servos
        message = ''
        sim_message = ''
        unsynced = 0
        for servo_id, servo in self._servos.items():
            if servo.synced:
                continue

            min_time = abs(servo.delta) / (servo.max_speed / 1000)  # max speed is in rot/sec, we want rot/ms
            if min_time > action_time:
                action_time = min_time
                print(f'warning: servo {servo_id} too fast, slowed movement to {action_time} ms')

            message += f'{servo_id}{servo.position_raw:06}'  # [0-6][position padded to 6 characters]
            sim_message += f'{servo_id} {round(servo.angle * 1000) / 1000} '
            unsynced += 1
            servo.synced = True

        message = f'{SERVO_MOVE_CODE}{unsynced}{message}{action_time:016}'  # code, then count, then data, then time
        self._write(message)

        if self.simulation:
            self.action_log.append (f'{SERVO_MOVE_CODE}{unsynced} {sim_message}{action_time}\n')
        else:
            time.sleep(action_time / 1000)  # wait until this command is finished before sending next one

    def _setPosition(self, servo_id, position):
        servo_id = self._validateId(servo_id)

        self._servos[servo_id].position = position

    def _setAngle(self, servo_id, angle):
        servo_id = self._validateId(servo_id)

        self._servos[servo_id].angle = angle

    def _setPositionRaw(self, servo_id, position_raw):
        servo_id = self._validateId(servo_id)

        self._servos[servo_id].position_raw = position_raw

    def _validateId(self, servo_id):
        if isinstance(servo_id, str):
            if servo_id not in self.SERVO_NAMES:
                raise Exception(f'{servo_id} is not a valid servo name')

            servo_id = self.SERVO_NAMES[servo_id]

        if servo_id not in self._servos:
            raise Exception(f'{servo_id} is not a valid servo id')

        return servo_id

    def __getitem__(self, i):
        i = self._validateId(i)

        return self._servos[i].position

    def __setitem__(self, key, value):
        self._setPosition(key, value)
        self._updateArm()

    def __len__(self):
        return len(self._servos)
