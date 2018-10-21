import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from sensor_msgs.msg import Image

from convertions.convertions import image_to_numpy
from face_recognition.face_recognition import load_image_file, face_encodings, face_locations, compare_faces

DIR = os.path.dirname(os.path.realpath(__file__))

# This is a demo of running face recognition on live video from your webcam. It's a little more complicated than the
# other example, but it includes some basic performance tweaks to make things run a lot faster:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# OpenCV is *not* required to use the face_recognition library. It's only required if you want to run this
# specific demo. If you have trouble installing it, try any of the other demos that don't require it instead.

# Load a sample picture and learn how to recognize it.
obama_image = load_image_file(os.path.join(DIR, "obama.jpg"))
obama_face_encoding = face_encodings(obama_image)[0]

# Load a second sample picture and learn how to recognize it.
biden_image = load_image_file(os.path.join(DIR, "biden.jpg"))
biden_face_encoding = face_encodings(biden_image)[0]

# Load a second sample picture and learn how to recognize it.
emilka_image = load_image_file(os.path.join(DIR, "emilka.jpg"))
emilka_face_encoding = face_encodings(emilka_image)[0]

# Load a second sample picture and learn how to recognize it.
wagram_image = load_image_file(os.path.join(DIR, "wagram.jpg"))
wagram_face_encoding = face_encodings(wagram_image)[0]


# Create arrays of known face encodings and their names
known_face_encodings = [
    obama_face_encoding,
    biden_face_encoding,
    emilka_face_encoding,
    wagram_face_encoding
]
known_face_names = [
    "Barack Obama",
    "Joe Biden",
    "Emilka",
    "Wagram"
]

# Initialize some variables
_face_locations = []
_face_encodings = []
face_names = []
process_frame = True


class FaceRecognizerNode(Node):

    def __init__(self):
        super().__init__('face_recognition_webcam')

        qos_profile = qos_profile_default
        qos_profile.depth = 1

        self.subscription = self.create_subscription(
            Image,
            'image',
            self.recognize_faces)

        self.process_frame = True
        self.get_logger().info('NODE STARTED')

    def recognize_faces(self, msg):
        # Grab a single frame of video
        frame = image_to_numpy(msg)

        if self.process_frame:
            # Resize frame of video to 1/4 size for faster face recognition processing
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = small_frame[:, :, ::-1]

            # Find all the faces and face encodings in the current frame of video
            _face_locations = face_locations(rgb_small_frame)
            _face_encodings = face_encodings(rgb_small_frame, _face_locations)

            face_names = []
            for face_encoding in _face_encodings:
                # See if the face is a match for the known face(s)
                matches = compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = known_face_names[first_match_index]

                face_names.append(name)

            # Display the results
            for (top, right, bottom, left), name in zip(_face_locations, face_names):
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Face recognition', frame)
        cv2.waitKey(1)

        self.process_frame = not self.process_frame

        # return face_names

    def stop(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    face_recognizer_node = FaceRecognizerNode()

    rclpy.spin(face_recognizer_node)

    face_recognizer_node.get_logger().info('NODE TERMINATED')
    face_recognizer_node.stop()
    face_recognizer_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

