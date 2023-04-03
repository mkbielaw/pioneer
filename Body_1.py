"""
    Implementation of the Body class of the Agent
"""

import logging
from zmqRemoteApi import RemoteAPIClient
from typing import Any, List
import time


class Body:
    _my_sensors: set
    _my_actuators: List
    _my_name: str
    _my_perceptions: set

    def __init__(self, name, robot_sens_array, robot_act_array):
        logging.info(f"Agent name: {name} initialized.")
        self._my_name = name
        self._my_sensors = robot_sens_array
        self._my_actuators = robot_act_array

    def __repr__(self):
        out = f"{self._my_name}: I have {len(self._my_sensors)} sensors and {len(self._my_actuators)} list of actuators"
        return out

    def perceive(self, stimuli):
        # transform stimuli from sensors into perceptions
        for s in stimuli:
            self._my_perceptions.add(...)
        pass

    def step(self, t: int):
        # called by the simulator at each simulation step
        logging.info(f"{self._my_name}: agent body step at time {t}")
        stimuli = [s.get_value() for s in self._my_sensors]
        logging.info(f"{self._my_name}: stimuli at time {t} -> {stimuli}")
        self.perceive(stimuli)
        logging.info(f"{self._my_name}: perceptions at time {t} -> {self._my_perceptions}")
        # how about command?
        
    def get_perceptions(self):
        return self._my_perceptions


"""
    Simulated body class to pilot the simulated robot inside
    the simulator (CoppeliaSim)
"""


class SimulatedRobotnikBody(Body):
    _sim: Any
    _cSim_client: Any

    def __init__(self, name: str):
        self._my_name = name
        # zmqRemoteApi connection
        self._cSim_client = RemoteAPIClient()
        self._sim = self._cSim_client.getObject('sim')
        self._my_actuators = []
        # Get handles
        motors = [self._sim.getObject("./front_left_wheel"),
                  self._sim.getObject("./front_right_wheel"),
                  self._sim.getObject("./back_left_wheel"),
                  self._sim.getObject("./back_right_wheel")]
        self._my_actuators.append(motors)
        self._my_sensors = set()
        # self._myu_sensors.add(...)

    def act(self, motor_speeds: List[float]):
        """
        Set the current speed to all motors of the (simulated) robot
        :param motor_speeds: list of values for motors
        :return: True if everything is ok
        """
        try:
            assert len(motor_speeds) == len(self._my_actuators[0])
            for i, speed in enumerate(motor_speeds):
                self._sim.setJointTargetVelocity(self._my_actuators[0][i], motor_speeds[i])
            return True
        except Exception as e:
            print(e)
            logging.exception(e)
            return False

    def start(self):
        self._sim.startSimulation()

    def stop(self):
        self._sim.stopSimulation()

class SimulatedPioneerBody(Body):
    _sim: Any
    _cSim_client: Any
    _my_sensors: List
    _my_actuators: List
    _my_perceptions: dict

    def __init__(self, name: str):
        self._my_name = name
        # zmqRemoteApi connection
        self._cSim_client = RemoteAPIClient()
        self._sim = self._cSim_client.getObject('sim')
        self._my_actuators = []
        # Get handles
        motors = [self._sim.getObject("./leftMotor"),
                  self._sim.getObject("./rightMotor")]
        self._my_actuators.append(motors)
        self._my_sensors = []
        sensors = [self._sim.getObject("./ultrasonicSensor[0]"),
                   self._sim.getObject("./ultrasonicSensor[1]"),
                   self._sim.getObject("./ultrasonicSensor[2]"),
                   self._sim.getObject("./ultrasonicSensor[3]"),
                   self._sim.getObject("./ultrasonicSensor[4]"),
                   self._sim.getObject("./ultrasonicSensor[5]"),
                   self._sim.getObject("./ultrasonicSensor[6]"),
                   self._sim.getObject("./ultrasonicSensor[7]"),
                   self._sim.getObject("./ultrasonicSensor[8]"),
                   self._sim.getObject("./ultrasonicSensor[9]"),
                   self._sim.getObject("./ultrasonicSensor[10]"),
                   self._sim.getObject("./ultrasonicSensor[11]"),
                   self._sim.getObject("./ultrasonicSensor[12]"),
                   self._sim.getObject("./ultrasonicSensor[13]"),
                   self._sim.getObject("./ultrasonicSensor[14]"),
                   self._sim.getObject("./ultrasonicSensor[15]")
        ]
        self._my_sensors.append(sensors)

    def act(self, motor_speeds: List[float]):
        """
        Set the current speed to all motors of the (simulated) robot
        :param motor_speeds: list of values for motors
        :return: True if everything is ok
        """
        try:
            assert len(motor_speeds) == len(self._my_actuators[0])
            for i, speed in enumerate(motor_speeds):
                self._sim.setJointTargetVelocity(self._my_actuators[0][i], motor_speeds[i])
            return True
        except Exception as e:
            print(e)
            logging.exception(e)
            return False

    def _percept(self, sens_values: List) -> List[Any]:
        # Heuristic function
        # For the Pioneer robot, let's find the most FREE-SPACE around
        # sens_values array of 16 floating numbers
        # 0.0 were there is free space
        # calculate the index with the maximum number of surrounding zeros
        """ let's try this idea:
            for each position:
                if is a non-zero position , forget it
                else (is a zero position):
                    sl = sum of zeros on the left
                    sr = sum of zeros on the right
                    save into current position sl+sr
                select the index with the maximum value
        """
        res = len(sens_values)*[0]
        ones = len(sens_values)*[0]
        for i, v in enumerate(sens_values):
            if v == 0.0:
                ones[i] = 1

        for pos in range(len(sens_values)):
            if pos > 0:
                sl = sum(ones[:pos])
            else:
                sl = 0
            if pos < len(sens_values):
                sr = sum(ones[pos+1:])
            else:
                sr = 0
            res[pos] = sl + sr
        print(res)
        return []

    def sense(self):
        """
        Read from (simulated-)hardware sensor devices and store into the internal array
        :return: True if all right, else hardware problem

        readProximitySensor:
        int result,float distance,list detectedPoint,int detectedObjectHandle,list detectedSurfaceNormalVector=sim.readProximitySensor(int sensorHandle)
        """
        try:
            sensors = self._my_sensors[0]
            values = []
            for sens in sensors:
                _, dis, _, _, _ = self._sim.readProximitySensor(sens)
                values.append(dis)
            print(values)
            self._my_perceptions = self._percept(values)
            return True
        except Exception as e:
            print(e)
            logging.exception(e)
            return False

    def plan(self):
        """
        Planning algorithm for the robot to navigate
            1. read perceptions
            2. decide a new direction
            3. output the action
        :return:
        """
        pass

    def start(self):
        self._sim.startSimulation()

    def stop(self):
        self._sim.stopSimulation()


if __name__ == "__main__":
    # my_body = SimulatedRobotnikBody("pilot")
    my_body = SimulatedPioneerBody("pilot")
    print(my_body)
    while True:
        my_body.sense()
        my_body.plan()
        time.sleep(1)
    # my_body.start()
    # my_body.act([2, -0.7, 2, -0.7])
    time.sleep(20)
    # my_body.stop()
    print("Done.")