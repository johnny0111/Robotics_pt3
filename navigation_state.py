#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from tf import transformations

class NavigationState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.
    ># waypoint     Pose2D      Target waypoint for navigation.
    
    <= arrived                  Navigation to target pose succeeded.
    <= failed                   Navigation to target pose failed.
    """

    def __init__(self):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(NavigationState, self).__init__(outcomes = ['arrived', 'failed'], input_keys = ['waypoint'])
        """
        TODO: set the action topic to /move_base and instantiate the ProxyActionClient
        """
        self._action_topic = '/move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        # The constructor is called when building the state machine, not when actually starting the behavior.
        self._arrived = False
        self._failed = False


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._arrived:
            return 'arrived'
        if self._failed:
            return 'failed'

        #Wait for action result and return outcome accordingly
        if self._client.has_result(self._action_topic):
            """
            TODO: check the result, set the appropriate flag and return the correct outcome
            """
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._arrived = True
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('Navigation failed: %s' % str(status))   
                self._failed = True           
		
    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._arrived = False
        self._failed = False

        # Create and populate action goal
        goal = MoveBaseGoal()
        """
        TODO: Fill in the Point and Quaternion with the userdata waypoint  
        """
        pt = Point(x = userdata.waypoint.x, y = userdata.waypoint.y)
        qt = transformations.quaternion_from_euler(0, 0, userdata.waypoint.theta)

        goal.target_pose.pose = Pose(position = pt, orientation = Quaternion(*qt))
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        """
        TODO: Send the goal for execution to the correct action topic  
        """
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo("Sent goal to action server...")
        except Exception as e:
            Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
            self._failed = True

    
    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.
        self.cancel_active_goals()

    def on_stop(self):
        # This method is called whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.
        self.cancel_active_goals()