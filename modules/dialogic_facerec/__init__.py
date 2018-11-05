
import rclpy
from dialogic import registry
from dialogic.state import state
from dialogic.receptor import receptor
from dialogic.property import PropertyBase
from std_msgs.msg import String


rclpy.init()
node = rclpy.create_node("vision_node")

@state(triggers=":startup")
def facerec_run(ctx):

    @receptor(ctx_wrap=ctx, write="facerec:face")
    def face_recognition_callback(ctx, msg):
        ctx["facerec:face"] = msg

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