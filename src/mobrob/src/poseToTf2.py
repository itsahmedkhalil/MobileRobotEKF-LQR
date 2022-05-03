#! /usr/bin/python3
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import tf_conversions
import tf2_ros
import math
import traceback

from mobrob_util.msg import ME439WheelAngles
lastMsg = ME439WheelAngles()
lastMsg.ang_left = 0
lastMsg.ang_right = 0
isSimulation = False

pathInterval = .02 #how many meters before making another path point (2cm now).
last_point = Pose2D()
path_distance = 0.0
max_path_pts = 100 #adjust this if you bog down on raspberry pi.
markerArray = MarkerArray()


def convertPose2DToMarker(pose):
    marker = Marker()
    marker.header.frame_id="world"
    marker.type=marker.SPHERE
    marker.action=marker.ADD
    marker.scale.x = .01
    marker.scale.y = .01
    marker.scale.z = .01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.b = 1.0
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = 0.0
    return marker


def broadcastTf2(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = 'base_link'
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta + 3.1415926535/2)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    setWheelAngles(lastMsg) #this doesn't work yet.. so it just sets them all to 0 now.
    br.sendTransform(t);
    global path_distance
    global last_point
    global markerPublisher
    path_distance  = path_distance + math.sqrt((msg.x-last_point.x)*(msg.x-last_point.x) + (msg.y-last_point.y)*(msg.y-last_point.y))
    last_point = msg
    if (path_distance > pathInterval):
        markerArray.markers.append(convertPose2DToMarker(msg))
        path_distance = 0
        if (len(markerArray.markers)>max_path_pts):
            markerArray.markers.pop(0)
        id = 0
        for m in markerArray.markers:
            m.id = id
            id = id +1
        markerPublisher.publish(markerArray)
        
        
def saveLastMessage(msg):
    lastMsg = msg;
    
    
def setWheelAngles(msg):
    global toSend, jointPublisher
    vals = [0.0]*3
    vals[0] = 0.0;
    vals[1] = 0.0;
    vals[2] = 0.0;
#    vals[1] = msg.ang_right;
#    vals[2] = msg.ang_left;
    toSend.position=vals;
    toSend.header.stamp = rospy.Time.now();
    jointPublisher.publish(toSend);



def main():
    rospy.init_node('poseToTf2')
    type_of_simulation = rospy.get_param("type_of_launch")
    global markerPublisher
    markerPublisher = rospy.Publisher('/marker_array',MarkerArray, queue_size=1)
    global jointPublisher
    jointPublisher = rospy.Publisher('joint_states',JointState, queue_size=1)
    global toSend
    toSend = JointState()
    toSend.name = ['base_joint','right_wheel_joint','left_wheel_joint'];
    
    if (type_of_simulation == "simulation"):
        isSimulation = True
        rospy.Subscriber('/robot_pose_simulated',Pose2D,broadcastTf2)
    else:
        rospy.Subscriber('/robot_pose_estimated',Pose2D,broadcastTf2)
        rospy.Subscriber('/robot_wheel_angle', ME439WheelAngles,saveLastMessage)
    
    rospy.spin()    


if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
    
    
