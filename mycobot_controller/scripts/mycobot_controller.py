#!/usr/bin/env python3.8

from enum import Enum
import math
import sys
import time
import rospy, rosparam
import actionlib

from mycobot_msgs.msg import (
    MyCobotAngles,
    MyCobotCoords,
    MyCobotPumpStatus,
)

from mycobot_msgs.srv import (
    SetAngles, SetAnglesResponse, 
    SetCoords, SetCoordsResponse
)

from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryAction
)

from sensor_msgs.msg import (
    JointState
)

from trajectory_msgs.msg import JointTrajectory

from pymycobot.mycobot import MyCobot

class JointFlag(Enum):
    ANGLES = 0
    COORDS = 1

conf_joint_names = [f'joint{i}' for i in range(1, 7)]
node_name = 'mycobot_controller'

# Extended MyCobot class
class EMyCobot(MyCobot):
    def __init__(
        self, 
        port : str = '/dev/ttyUSB0', 
        baudrate : str = '115200', 
        timeout : float =0.1, 
        debug : bool = False, 
        eps : float = math.pi / 100.
    ):
        self.__eps = eps
        super().__init__(port, baudrate, timeout, debug)

    def is_in_angles(self, data: list) -> int:
        """
        Check if the current angle is close enough to the given angles (within eps).
        
        Args:
            data : destination angles (radian)
        Return:
            1  : True
            0  : False
            -1 : Fail (to get angles)
        """
        mydata = super().get_radians()
        
        if mydata:
            res = [abs(pos - mypos) for pos, mypos in zip(data, mydata)]
            max_res = max(res)
            return 1 if max_res < self.__eps else 0
        else:
            # failed to get angles  
            return -1
        
    def is_in_coords(self, data: list) -> int:
        """
        Check if the current coordinates is close enough to the given coordinates (within eps).
        
        Args:
            data : destination coordinates
        Return:
            1  : True
            0  : False
            -1 : Fail (to get coordinates)
        """
        mydata = super().get_coords()
        
        if mydata:
            res = [abs(pos - mypos) for pos, mypos in zip(data, mydata)]
            max_res = max(res)
            return 1 if max_res < self.__eps else 0
        else:
            # failed to get angles  
            return -1

    def sync_send_angles(self, angles: list, sp: int, timeout: int = 10)-> bool:
        """
        Send angles in synchronous state.
        
        Args:
            angles: target angles (radian)
            sp : speed of motion (0 ~ 100)
            timeout : timeout second
        Return:
            True : success
            False : failure
        """
        super().send_radians(angles, sp)
        
        result = False
        start_time = time.time()      
        while time.time() - start_time < timeout:
            if self.is_in_angles(angles) == 1:
                return True
            time.sleep(0.1)
        
        return False

    def sync_send_coords(self, coords: list, sp: int, timeout: int = 10)-> bool:
        """
        Send coordinates in synchronous state.
        
        Args:
            coords: target coordinates
            sp : speed of motion (0 ~ 100)
            timeout : timeout second
        Return:
            True : success
            False : failure
        """
        super().send_coords(coords, sp)
        
        result = False
        start_time = time.time()      
        while time.time() - start_time < timeout:
            state = self.is_in_coords(coords)
            rospy.loginfo(state)
            if self.is_in_coords(coords) == 1:
                return True
            time.sleep(0.1)
        
        return False
        

    
class MyCobotController:
    __speed = 100
    joint_names = [f'joint{i}' for i in range(1, 7)]

    def __init__(
        self, 
        usb_port = '/dev/ttyUSB0', 
        baud_rate = '115200', 
        timeout = 10, 
        eps = math.pi / 100.,
    ):
        self.__mycobot = EMyCobot(usb_port, baud_rate, eps=eps)
        self.__timeout = timeout
        
        self.__arm_controller = rosparam.get_param(f'{rospy.get_name()}/arm_controller_name') # 'arm_controller'
        self.__control_period = rosparam.get_param(f'{rospy.get_name()}/control_period') # 0.01

        self.__initPublisher()
        self.__initServer()

    def getControlPeriod(self): return self.__control_period

    def __initPublisher(self):
        # self.__mycobot_joint_angles_pub = rospy.Publisher('joint_angles_state', MyCobotAngles, queue_size=5)
        # self.__mycobot_joint_coords_pub = rospy.Publisher('joint_coords_state', MyCobotCoords, queue_size=5)
        
        self.__mycobot_joint_state_seq = 0
        self.__angles_old = self.__mycobot.get_radians()
        self.__mycobot_joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=2)

    def __initServer(self):
        rospy.Service('set_joint_angles', SetAngles, self.__setAngles)
        rospy.Service('set_joint_coords', SetCoords, self.__setCoords)

        self.__mycobot_follow_joint_trajectory_sever = actionlib.SimpleActionServer(
            f'{self.__arm_controller}/follow_joint_trajectory', 
            FollowJointTrajectoryAction,
            self.__executeFollowJointTrajectoryCB,
            False
        )
        self.__mycobot_follow_joint_trajectory_sever.start()

    def __rearrange_joint_trajectory(self, joint_trajectory : JointTrajectory):
        # ---------------------------------------------------------------------------------------
        # Rearranges the point path following the name convention joint1, ... joint6
        # Warning: This function has side effects
        # ---------------------------------------------------------------------------------------

        mapping = [joint_trajectory.joint_names.index(j) for j in conf_joint_names]

        for point in joint_trajectory.points:
            # each array might be empty
            if point.positions:
                point.positions = [point.positions[i] for i in mapping]
            if point.velocities:
                point.velocities = [point.velocities[i] for i in mapping]
            if point.accelerations:
                point.accelerations = [point.accelerations[i] for i in mapping]
            if point.effort:
                point.effort = [point.effort[i] for i in mapping]

        joint_trajectory.joint_names = conf_joint_names

    def __setAngles(self, req):
        angles = [
            req.joint1,
            req.joint2,
            req.joint3,
            req.joint4,
            req.joint5,
            req.joint6,
        ]
        sp = req.speed

        # cannot judge if mycobot reaches the destination or not
        # does not work (origin)
        # self.__mycobot.sync_send_angles(angles, sp, self.__timeout)
        result = self.__mycobot.sync_send_angles(angles, sp, self.__timeout)

        return SetAnglesResponse(result)

    def __setCoords(self, req):
        coords = [
            req.x,
            req.y,
            req.z,
            req.rx,
            req.ry,
            req.rz,
        ]
        sp = req.speed
        mod = req.model

        # does not work
        self.__mycobot.sync_send_coords(coords, sp, mod, self.__timeout)
        result = self.__mycobot.is_in_position(coords, JointFlag.COORDS.value)

        return SetCoordsResponse(result)

    def publishCallback(self, event):
        # self.__publishAngles()
        # self.__publishCoords()
        self.__publishJointState()

    def __publishAngles(self):
        ma = MyCobotAngles()
        angles = self.__mycobot.get_radians()
        if angles:
            ma.joint1 = angles[0]
            ma.joint2 = angles[1]
            ma.joint3 = angles[2]
            ma.joint4 = angles[3]
            ma.joint5 = angles[4]
            ma.joint6 = angles[5]

        self.__mycobot_joint_angles_pub.publish(ma)

    def __publishCoords(self):
        mc = MyCobotCoords()
        coords = self.__mycobot.get_coords()
        if coords:
            mc.x = coords[0]
            mc.y = coords[1]
            mc.z = coords[2]
            mc.rx = coords[3]
            mc.ry = coords[4]
            mc.rz = coords[5]

        self.__mycobot_joint_coords_pub.publish(mc)

    def __publishJointState(self):
        js = JointState()

        self.__mycobot_joint_state_seq += 1
        js.header.seq = self.__mycobot_joint_state_seq
        js.header.stamp = rospy.Time.now()
        
        angles = self.__mycobot.get_radians()
        if angles:
            js.position = angles
            if self.__angles_old:
                js.velocity = [(angle - angle_old) / self.__control_period for angle, angle_old in zip(angles, self.__angles_old)]            
            js.name = MyCobotController.joint_names

        self.__angles_old = angles

        self.__mycobot_joint_state_pub.publish(js)

    def __executeFollowJointTrajectoryCB(self, goal : FollowJointTrajectoryGoal):
        # A trajectory needs at least 2 points        
        if len(goal.trajectory.points) < 2:
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self.__mycobot_follow_joint_trajectory_sever.set_aborted(result, 'A trajectory needs at least 2 points')
            return

        # It is required to rearrange the arrays because MoveIt doesn't guarantee orden preservation
        self.__rearrange_joint_trajectory(goal.trajectory)

        time_start = rospy.Time.from_sec(time.time())

        last_point = goal.trajectory.points[0]
        for point in goal.trajectory.points[1:]:
            # Send and Wait
            self.__mycobot.send_radians(point.positions, MyCobotController.__speed)
            # rospy.sleep(point.time_from_start - last_point.time_from_start)

            # Trajectory abort!
            # To abort the current movement, it is possible to send an empty trajectory
            if self.__mycobot_follow_joint_trajectory_sever.is_preempt_requested():
                result = FollowJointTrajectoryResult()
                result.error_code = 101
                self.__mycobot_follow_joint_trajectory_sever.set_preempted(result, 'FollowJointTrajectory preempted')
                return

            # Feedback
            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = goal.trajectory.joint_names
            feedback.desired = point
            feedback.actual.positions = self.__mycobot.get_radians()
            # feedback.actual.velocities = self.__mycobot.get_velocities()
            feedback.actual.time_from_start = rospy.Time.from_sec(time.time()) - time_start
            self.__mycobot_follow_joint_trajectory_sever.publish_feedback(feedback)

            last_point = point

        # Result
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.__mycobot_follow_joint_trajectory_sever.set_succeeded(result)

if __name__ == '__main__':
    usb_port = '/dev/ttyUSB0'
    # fail unless baud_rate = 115200
    baud_rate = '115200'
    timeout = 10
    eps = math.pi / 100.

    argv = sys.argv
    argc = len(argv)

    if (argc >= 3):
        usb_port = argv[1]
        baud_rate = argv[2]
        if (argc >= 4):
            timeout = argv[3]
        if (argc >= 5):
            eps = argv[4]


    print(f'port_name, baud_rate, timeout, eps are set to {usb_port}, {baud_rate}, {timeout}s, {eps}')

    rospy.init_node(node_name)

    # print(rospy.get_name())

    mc_controller = MyCobotController(usb_port, baud_rate, timeout)

    # publish topic
    rospy.Timer(rospy.Duration(0.01), mc_controller.publishCallback)

    # start service
    rospy.spin()
    # loop_rate = rospy.Rate(100)
    # while (not rospy.is_shutdown()):
    #     rospy.spinOnce()
    #     loop_rate.sleep()
