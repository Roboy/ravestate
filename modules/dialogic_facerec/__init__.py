
import rclpy
from dialogic import registry
from dialogic.state import state
from dialogic.property import PropertyBase
from std_msgs.msg import String
from threading import Semaphore
from threading import Lock

rclpy.init()
node = rclpy.create_node("vision_node")

recognition_queue = []
recognition_queue_lock = Lock()
recognition_queue_size = Semaphore(0)



@state(triggers=":startup")
def rcl_spin(sess):
    def face_recognition_callback(msg):
        with recognition_queue_lock:
            recognition_queue.append(msg.data)
            recognition_queue_size.release()
    node.create_subscription(String, "/roboy/vision/recognized_faces", face_recognition_callback)
    rclpy.spin(node)



@state(triggers=((":startup",),("facerec:value-handover",)), signal="value-handover")
def wait_for_recognition(sess):
    recognition_queue_size.acquire()
    with recognition_queue_lock:
        msg = recognition_queue.pop(0)

    # Use a small handover state to actually write the property,
    # such that facerec:face is not blocked by wait_for_recognition.
    @state(triggers="facerec:value-handover", write="facerec:face")
    def face_value_handover(sess):
        sess["facerec:face"] = msg
        return sess.DeleteMe

    sess.add_state(face_value_handover)
    return True


@state(triggers=":shutdown")
def shutdown():
    node.destroy_node()
    rclpy.shutdown()


registry.register(
    name="facerec",
    props=PropertyBase(name="face", default=""),
    states=(rcl_spin, wait_for_recognition, shutdown))