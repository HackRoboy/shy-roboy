from pexpect import pxssh
from std_msgs.msg import Float32
import rospy 


def compare_and_shout(data)    
    if float(data) < 0.7:
        s = pxssh.pxssh()
        s.login("192.168.0.105", "roboy", "Roboy2016")
        s.sendline("docker exec -it hungry_fermi bash")
        s.sendline("source ~/ros2_ws/install/setup.sh")
        s.sendline("ros2 service call /roboy/cognition/speech/synthesis/talk roboy_cognition_msgs/Talk {\"text: please don't touch me I am antisocial\"}")
        s.logout()
    
rospy.init_node('see_and_shout')
rospy.Subscriber("/shy_roboy/nearest_distance", Float32, callback=compare_and_shout)

rospy.spin()
