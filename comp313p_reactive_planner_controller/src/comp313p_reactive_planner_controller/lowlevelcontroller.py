import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp313p_reactive_planner_controller.planned_path import PlannedPath
from comp313p_reactive_planner_controller.controller_base import ControllerBase
from comp313p_mapper.srv import *
import math
import angles
import time

totalDistance = 0
totalAnglesTurned = 0
totalTime = 0

class  LowLevelController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)
        
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)
        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))


        # Flag to toggle the mapper state
        self.enableSettingMapperState = rospy.get_param('enable_change_mapper_state', True)
        rospy.loginfo('enableSettingMapperState=%d', self.enableSettingMapperState)
        self.mappingState = True

        # Get the service to switch the mapper on and off if required
        if self.enableSettingMapperState is True:
            rospy.loginfo('Waiting for change_mapper_state')
            rospy.wait_for_service('change_mapper_state')
            self.changeMapperStateService = rospy.ServiceProxy('change_mapper_state', ChangeMapperState)
            rospy.loginfo('Got the change_mapper_state service')
   
    # Gets distance from goal/waypoint
    def get_distance(self, waypoint):
        distance = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
        return distance

    # Gets the angle 
    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta

    def driveToWaypoint(self, waypoint):
        #print(waypoint[0])
        #print(waypoint[1])
        vel_msg = Twist()
        t0 = time.time()
        distance = self.get_distance(waypoint)
        dx = waypoint[0] - self.pose.x
        dy = waypoint[1] - self.pose.y
        newOrient = atan2(dy, dx)
        angleToBeChanged = self.shortestAngularDistance(self.pose.theta, newOrient)

        anglesTurned = 0
        waypointDist = 0
        currentAngle = self.pose.theta
        currentX = self.pose.x
        currentY = self.pose.y
        while(math.fabs(angleToBeChanged) >= self.driveAngleErrorTolerance and (not rospy.is_shutdown())):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            # Speed which robot turns
            #if(angleToBeChanged > 0):
            #    vel_msg.angular.z = 0.2
            #else:
            #    vel_msg.angular.z = -0.2
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleToBeChanged, 5.0))

            self.velocityPublisher.publish(vel_msg)
            #Calculate angle
            dx = waypoint[0] - self.pose.x
            dy = waypoint[1] - self.pose.y
            newOrient = atan2(dy, dx)
            angleToBeChanged = self.shortestAngularDistance(self.pose.theta, newOrient)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
        #Stop robot rotation 
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)
        angleDiff = (self.pose.theta - currentAngle) * 180 / math.pi
        angleDiff = ((angleDiff + 180) % 360) - 180
        anglesTurned = math.fabs(angleDiff)
        # Rotation complete
        # print("Rotation Complete\n")
        

        while(distance >= self.distanceErrorTolerance):
            #Move robot in a straight line (Fixed speed for now)
            vel_msg.linear.x = 0.1
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.velocityPublisher.publish(vel_msg)
            distance = self.get_distance(waypoint)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
        
        #Stop robot moving
        waypointDist = sqrt(pow((currentX - self.pose.x), 2) + pow((currentY - self.pose.y), 2))
        vel_msg.linear.x = 0
        self.velocityPublisher.publish(vel_msg)
        t1 = time.time()
        # Motion complete
        timeTaken = t1 - t0
        # print("Moving Complete\n")

        global totalDistance
        global totalAnglesTurned
        global totalTime
        totalDistance += waypointDist
        totalAnglesTurned += anglesTurned
        totalTime += timeTaken
        #print("Total angles turned for now:" + str(totalAnglesTurned))
        anglesTurned = 0

    def rotateToGoalOrientation(self, goalOrientation):
        vel_msg = Twist()

        goalOrientation = math.radians(goalOrientation)

        angleToBeChanged = self.shortestAngularDistance(self.pose.theta, goalOrientation)
        t0 = time.time()
        currentAngle = self.pose.theta
        while(math.fabs(angleToBeChanged) >= self.goalAngleErrorTolerance and not rospy.is_shutdown()):
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            #if(angleToBeChanged > 0):
            #    vel_msg.angular.z = 0.1
            #else:
            #    vel_msg.angular.z = -0.1
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleToBeChanged, 5.0))
            #Calculate angle
            self.velocityPublisher.publish(vel_msg)
            angleToBeChanged = self.shortestAngularDistance(self.pose.theta, goalOrientation)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                               
        # Stop movement once finished
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)

        t1 = time.time()
        timeTaken = t1 - t0
        global totalDistance
        global totalAnglesTurned
        global totalTime
        angleDiff = (currentAngle - self.pose.theta) * 180 / math.pi
        angleDiff = ((angleDiff + 180) % 360) - 180
        totalAnglesTurned += math.fabs(angleDiff)
        totalTime += timeTaken
        print("Arrived at waypoint")
        print("Time taken " + str(totalTime))
        print("Distance travelled " + str(totalDistance))
        print("Angles turned " + str(totalAnglesTurned))

        #Reset
        totalDistance = 0
        totalAnglesTurned = 0
        totalTime = 0




        

        
