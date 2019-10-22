from collections import defaultdict, namedtuple
import numpy as np
from numpy import ndarray
from typing import Dict, Tuple, List, Set

from reggol import get_logger
logger = get_logger(__name__)

from pyroboy.face_recognition import FaceRec
try:
    from roboy_cognition_msgs.msg import Faces, FacialFeatures
except ImportError as e:
    logger.error(f"""
--------
An exception occurred during `from roboy_cognition_msgs.msg import Faces`: {e}
Roboy will not comment on recognized faces!
--------
""")


Person = namedtuple('Person', ['is_known', 'id', 'face_vector'])
Face = namedtuple('Face', ['id', 'confidence', 'face_encodings'])

# Keep the last 10 messages
FACE_MESSAGE_WINDOW_SIZE = 10
CONFIDENCE_THRESHOLD = 0.8


class FaceOracleFilter:

    def __init__(self, window_size=FACE_MESSAGE_WINDOW_SIZE, confidence_threshold=CONFIDENCE_THRESHOLD):
        """
        Constructor.
        """

        """
        Count messages per person. A person is identified by ...
          1. ... a bool indicating whether the person is known or not.
          2. ... a Scientio primary key for known people, OR an unknown person index otherwise.
        """
        self.messages_per_person: Dict[int, List[Face]] = defaultdict(list)

        """
        We need to keep track of person objects.
        """
        self.people: Dict[int, Person] = dict()

        """
        We need IDs for anonymous people.
        """
        self.next_unknown_index = 0

        """
        Keep a history of previous faces messages
        """
        self.message_buffer: List[Face] = []

        """
        Tuple of known-flag, primary key/anonymous-index, face vector.
        """
        self.current_best_guess: Person = None

    def push_message(self, msg: Faces) -> bool:
        """
        Add a new FaceOracle Faces message to the filter. recalculate_best_guess() is called
         automatically, and a flag indicating whether the best guess changed is returned.
         Old messages are forgotten automatically.
        :param msg: The new message to add.
        :return: True if the current best interlocutor match changed, false otherwise.
        """

        for index in range(len(msg.ids)):
            face: Face = Face(msg.ids[index], msg.confidence[index], msg.face_encodings[index])

            # Remember the new message
            self.message_buffer.append(face)

            # Remove old messages if necessary
            if len(self.message_buffer) > FACE_MESSAGE_WINDOW_SIZE:

                message_to_forget = self.message_buffer[0]
                self.message_buffer = self.message_buffer[1:]

                # Remove message from each person that might reference it
                for _, messages in self.messages_per_person.items():
                    if message_to_forget in messages:
                        messages.remove(message_to_forget)

                # Filter people that are out of messages
                for person_id in list(self.messages_per_person.keys()):
                    if len(self.messages_per_person[person_id]) == 0:
                        del self.messages_per_person[person_id]

            # Determine who gets the new message ...
            person: Person = None

            # focus = None

            # if self.current_best_guess and self.current_best_guess.id < 0:
            #     current_face_vec = np.array(self.people[self.current_best_guess.id].face_vector)
            #     incoming_face_vecs = [np.array(encoding.ff) for encoding in msg.face_encodings]
            #     idx, conf = FaceRec.match_face(current_face_vec, incoming_face_vecs)
            #     focus = idx
            # elif self.current_best_guess and self.current_best_guess.id in msg.ids:
            #     focus = msg.ids.index(self.current_best_guess.id)
            # else:
            #     focus = msg.confidence.index(max(msg.confidence))

            if face.confidence > CONFIDENCE_THRESHOLD:

                # There is a good database match
                person = Person(True, face.id, face.face_encodings.ff)

            else:

                # There is no good database match -> match or create anonymous interloc.
                ids, face_vecs = self.unknown_people_face_vecs_and_ids()
                if ids:
                    assert len(ids) == len(face_vecs)
                    # Match face vector among current strangers
                    idx, conf = FaceRec.match_face(np.array(face.face_encodings.ff), face_vecs)
                    if conf > CONFIDENCE_THRESHOLD:
                        # Stranger matched
                        person = Person(False, ids[idx], face.face_encodings.ff)

                if not person:
                    # Create new stranger
                    self.next_unknown_index -= 1
                    person = Person(False, self.next_unknown_index, face.face_encodings.ff)

            assert person is not None
            self.messages_per_person[person.id].append(face)
            self.people[person.id] = person

        return self.recalculate_best_guess()

    def unknown_people_face_vecs_and_ids(self) -> Tuple[List[int], List[ndarray]]:
        """
        Get same-length lists of unknown person ids and respective face vectors.
        """
        ids = list()
        face_vectors = list()

        for person_id, person in self.people.items():
            if not person.is_known:
                ids.append(person.id)
                face_vectors.append(np.array(person.face_vector))
        return ids, face_vectors

    def recalculate_best_guess(self) -> bool:
        """
        Determine which interlocutor has the best score right now,
         according to the messages currently buffered.
        :return: True if self.current_best_guess has changed. Note: current_best_guess
         might also change to None if no interlocutor with sufficient messages was found.
        """
        best_guess = None
        best_guess_count = 0
        for person_id, msgs in self.messages_per_person.items():
            assert len(msgs) > 0
            if len(msgs) > best_guess_count:
                best_guess = self.people[person_id]
                best_guess_count = len(msgs)
        if not self.current_best_guess or best_guess.id != self.current_best_guess.id:
            self.current_best_guess = best_guess
            logger.info(f'Current best guess changed. Person id: {best_guess.id}')
            return True
        return False

    def convert_current_anonymous_to_known(self, registered_primary_key):
        """
        If the current best guess is an anonymous interlocutor, convert it to
         a known interlocutor with the given primary key.
        """
        assert registered_primary_key >= 0
        unregistered_primary_key = self.current_best_guess.id

        assert not (registered_primary_key in self.people.keys())
        person_to_save = self.people[unregistered_primary_key]._replace(is_known=True, id=registered_primary_key)
        del self.people[unregistered_primary_key]

        self.messages_per_person[registered_primary_key] = self.messages_per_person[unregistered_primary_key]
        del self.messages_per_person[unregistered_primary_key]

        self.people[registered_primary_key] = person_to_save
        self.current_best_guess = person_to_save

    def print_people(self):
        print_str = ''
        for person in self.people:
            print_str += str(person)
