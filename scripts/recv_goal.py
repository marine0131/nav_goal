#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
from std_srvs.srv import Empty, EmptyRequest, Trigger, TriggerResponse
from nav_goal.srv import QRcodeAdjust, QRcodeAdjustRequest, GridMove, GridMoveRequest


class NavGoal():
    def __init__(self):
        rospy.Subscriber("nav_goal/goal", MoveBaseGoal, self.goal_callback)
        self.done_pub = rospy.Publisher("nav_goal/done", String, queue_size=1)
        rospy.Service("nav_goal/resume", Trigger, self.handle_resume)
        rospy.Service("nav_goal/cancel", Trigger, self.handle_cancel)
        rospy.Service("nav_goal/get_status", Trigger, self.handle_status)

        self.grid_move_client = rospy.ServiceProxy("nav_goal/grid_move", GridMove)
        self.clear_map_client = rospy.ServiceProxy("move_base/clear_costmaps",
                                                   Empty)
        self.ac = SimpleActionClient('move_base', MoveBaseAction)
        self.current_goal = MoveBaseGoal()

    def goal_callback(self, msg):
        rospy.loginfo('recieve a goal')
        # clear map
        req = EmptyRequest()
        self.clear_map_client.call(req)

        # call goal service
        self.ac.wait_for_server()

        self.ac.send_goal(msg, done_cb=self.done_callback, active_cb=None, feedback_cb=None)

        self.current_goal = msg  # save current goal
        rospy.loginfo('send goal')

    def done_callback(self, status, result):
        rospy.loginfo('goal reached')
        # adjust
        # qr_req = QRcodeAdjustRequest()
        # qr_req.code_message = '2333'
        # res = self.qrcode_adjust_client.call(qr_req)
        target_pose = PoseStamped()
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.0
        target_pose.pose.orientation.w = 0.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0

        # move in
        req = GridMoveRequest()
        req.dir = 1
        req.target_pose = target_pose
        res = self.grid_move_client.call(req)

        if not res.message == 'success':
            rospy.logwarn('get cargo failed')
            return None
        rospy.loginfo('get cargo success')

    def active_callback(self):
        rospy.loginfo('goal actived')

    def feedback_callback(self):
        rospy.loginfo('feedback')

    def handle_resume(self, req):
        rospy.loginfo('resume current goal')
        res = TriggerResponse()
        try:
            self.goal_callback(self.current_goal)  # restart last goal
        except Exception:
            res.message = 'fail'
            return res
        res.success = True
        res.message = "success"
        return res

    def handle_cancel(self, req):
        rospy.loginfo('cancel current goal')
        res = TriggerResponse()
        # self.ac.cancel_goal()
        self.ac.cancel_all_goals()
        res.success = True
        res.message = "success"
        return res

    def handle_status(self, req):
        res = TriggerResponse()
        try:
            state = str(self.ac.get_state())
            res.success = True
            res.message = state
        except Exception:
            res.success = False
            res.message = 'can not get status'
        return res


if __name__ == '__main__':
    rospy.init_node("recv_goal")
    NavGoal()

    while not rospy.is_shutdown():
        rospy.spin()
