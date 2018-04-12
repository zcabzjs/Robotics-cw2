#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from std_msgs.msg import String
from math import pow,atan2,sqrt
from comp313p_reactive_planner_controller.planned_path import PlannedPath
from comp313p_reactive_planner_controller.controller_base import ControllerBase
from comp313p_mapper.srv import *
import math
import angles
import time

totalDistance = 0
totalAngleTurned = 0
totalTime = 0
totalWaypoints = 0
# This sample controller works a fairly simple way. It figures out
# where the goal is. It first turns the robot until it's roughly in
# the correct direction and then keeps driving. It monitors the
# angular error and trims it as it goes.

class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)
        
        # Get the proportional gain settings
        self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        self.angleErrorGain = rospy.get_param('angle_error_gain', 4)
        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))

        # DELETE THIS IF NECESSARY ()
        self.detailPublisher = rospy.Publisher('details', String, queue_size=10)
        

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
   
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta
        
    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))
        t0 = time.time()
        currentX = self.pose.x
        currentY = self.pose.y
        currentTheta = self.pose.theta
        waypointDist = 0
        angleTurned = 0
        while (distanceError >= self.distanceErrorTolerance) & (not self.abortCurrentGoal) & (not rospy.is_shutdown()):
            #print("Current Pose: x: {}, y:{} , theta: {}\nGoal: x: {}, y: {}\n".format(self.pose.x, self.pose.y,
            #                                                                           self.pose.theta, waypoint[0],
            #                                                                           waypoint[1]))
            #print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))

            # Proportional Controller
            # linear velocity in the x-axis: only switch on when the angular error is sufficiently small
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                vel_msg.linear.x = max(0.0, min(self.distanceErrorGain * distanceError, 10.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))

		
            #print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))

            # Toggle switching the mapping on and off, depending on
            # how fast the robot is turning. This has to happen first
            # to make sure the mapping is disabled before the new
            # twist message is sent.
            if self.enableSettingMapperState is True:
                if (self.mappingState is True) and (abs(vel_msg.angular.z) > math.radians(0.1)):
                    self.mappingState = False
                    self.changeMapperStateService(False)
                elif (self.mappingState is False) and (abs(vel_msg.angular.z) < math.radians(0.1)):
                    self.mappingState = True
                    self.changeMapperStateService(True)
            
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            # Check if the occupancy grid has changed. If so, monitor if we can still reach the
            # goal or not
            #if self.occupancyGridHasChanged is True:
            #    if self.checkIfWaypointIsReachable(waypoint) is False:
            #        return False
            #    self.occupancyGridHasChanged = False
	    
	        # Recording distances and angles
            waypointDist += sqrt(pow((self.pose.x - currentX), 2) + pow((self.pose.y - currentY), 2))
            currentX = self.pose.x
            currentY = self.pose.y
            newTheta = self.pose.theta
            angleDiff = (newTheta - currentTheta) * 180 / math.pi
            angleDiff = ((angleDiff + 180) % 360) - 180	
            angleTurned += math.fabs(angleDiff)
            currentTheta = newTheta

            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Stopping our robot after the movement is over
        self.stopRobot()
        global totalTime
        t1 = time.time()
        timeTaken = t1 - t0
        totalTime += timeTaken
        global totalDistance
        global totalAngleTurned
        global totalWaypoints
        totalAngleTurned += angleTurned
        angleTurned = 0
        totalDistance += waypointDist
        waypointDist = 0
        totalWaypoints += 1
        return (not self.abortCurrentGoal) & (not rospy.is_shutdown())

    def rotateToGoalOrientation(self, goalOrientation):
        vel_msg = Twist()

        goalOrientation = math.radians(goalOrientation)

        angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)
        t0 = time.time()
        currentAngle = self.pose.theta
        if self.enableSettingMapperState is True:
            self.mappingState = False
            self.changeMapperStateService(False)

        while (math.fabs(angleError) >= self.goalAngleErrorTolerance) & (not self.abortCurrentGoal) \
              & (not rospy.is_shutdown()):
            #print 'Angular Error: ' + str(angleError)

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))

            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()
            angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)

        # Stop movement once finished
        self.stopRobot()
        t1 = time.time()
        timeTaken = t1-t0
        global totalTime
        global totalDistance
        global totalAngleTurned
        global totalWaypoints
        angleDiff = (currentAngle - self.pose.theta) * 180 / math.pi
        angleDiff = ((angleDiff + 180) % 360) - 180
        totalAngleTurned += math.fabs(angleDiff)
        totalTime += timeTaken

        #DELETE THIS IF NECESSARY
        totalDetails = str(totalAngleTurned) + ',' + str(totalDistance) + ',' + str(totalWaypoints)
        self.detailPublisher.publish(totalDetails)

        #print("Time taken " + str(totalTime))
        #print("Distance travelled " + str(totalDistance))
        #print("Angles turned " + str(totalAngleTurned))
        totalTime = 0
        totalDistance = 0
        totalAngleTurned = 0
        totalWaypoints = 0

        

        if self.enableSettingMapperState is True:
            self.mappingState = True
            self.changeMapperStateService(True)

        return (not self.abortCurrentGoal) & (not rospy.is_shutdown())
