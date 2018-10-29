
import rclpy
from dialogic import registry
from dialogic.state import state, invocable
from dialogic.property import PropertyBase
from std_msgs.msg import String
from threading import Semaphore
from threading import Lock

rclpy.init()
node = rclpy.create_node("vision_node")

@state(triggers=":startup")
def facerec_run(sess):

    @invocable(sess=sess, write="facerec:face")
    def face_recognition_callback(sess, msg):
        sess["facerec:face"] = msg

    node.create_subscription(String, "/roboy/vision/recognized_faces", face_recognition_callback)
    rclpy.spin(node)


@state(triggers=":shutdown")
def facerec_shutdown():
    node.destroy_node()
    rclpy.shutdown()


registry.register(
    name="facerec",
    props=PropertyBase(name="face", default=""),
    states=(facerec_run, facerec_shutdown))