import rospy
# Remember to import the pkg="topic_tools" in the launch file!!!!
from topic_tools.srv import MuxSelect, MuxSelectRequest

def CallMux(input_topic):
    rospy.wait_for_service('/mux_cmd_vel/select')
    mux_srv = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect)
    try:
      resp1 = mux_srv(MuxSelectRequest(topic=input_topic))
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

def ActivateFollow():
    CallMux('/follow/cmd_vel')


def ActivateWP():
    CallMux('/move_to/cmd_vel')


def ActivateStop():
    CallMux('/stop/cmd_vel')
