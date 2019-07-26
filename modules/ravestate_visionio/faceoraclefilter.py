from collections import defaultdict, namedtuple
from numpy import ndarray
from typing import Dict, Tuple, List, Set
from roboy_cognition_msgs.msg import Faces
from pyroboy.face_recognition import FaceRec


PersonId = namedtuple('PersonId', ['is_known', 'id', 'face_vector'])


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
        self.messages_per_person: Dict[PersonId, Set[Faces]] = defaultdict(set)

        """
        We need IDs for anonymous people.
        """
        self.next_unknown_index = 0

        """
        Keep a history of previous faces messages
        """
        self.message_buffer: List[Faces] = []

        """
        Tuple of known-flag, primary key/anonymous-index, face vector.
        """
        self.current_best_guess: PersonId = None


    def push_message(self, msg: Faces) -> bool:
        """
        Add a new FaceOracle Faces message to the filter. recalculate_best_guess() is called
         automatically, and a flag indicating whether the best guess changed is returned.
         Old messages are forgotten automatically.
        :param msg: The new message to add.
        :return: True if the current best interlocutor match changed, false otherwise.
        """

        # Remember the new message
        self.message_buffer.append(msg)

        # Remove old messages if necessary
        if len(self.message_buffer) > FACE_MESSAGE_WINDOW_SIZE:

            message_to_forget = self.message_buffer[0]
            self.message_buffer = self.message_buffer[1:]

            # Remove message from each person that might reference it
            for _, messages in self.messages_per_person.items():
                messages.discard(message_to_forget)

            # Filter people that are out of messages
            for person_id in list(self.messages_per_person.keys()):
                if len(self.messages_per_person[person_id]) == 0:
                    del self.messages_per_person[person_id]

        # Determine who gets the new message ...
        person_id: PersonId = None
        if msg.confidence[0] > CONFIDENCE_THRESHOLD:

            # There is a good database match
            person_id = PersonId(True, msg.ids[0], msg.face_encodings)

        else:

            # There is no good database match -> match or create anonymous interloc.
            ids, face_vecs = self.unknown_people_face_vecs_and_ids()
            if ids:
                assert len(ids) == len(face_vecs)
                # Match face vector among current strangers
                idx, conf = FaceRec.match_face(msg.face_encodings, face_vecs)
                if conf > CONFIDENCE_THRESHOLD:
                    # Stranger matched
                    person_id = PersonId(False, ids[idx], msg.face_encodings)

            if not person_id:
                # Create new stranger
                person_id = PersonId(False, self.next_unknown_index, msg.face_encodings)
                self.next_unknown_index += 1

        assert person_id is not None
        self.messages_per_person[person_id].add(msg)

        return self.recalculate_best_guess()


    def unknown_people_face_vecs_and_ids(self) -> Tuple[Tuple[ndarray], Tuple[int]]:
        """
        Get same-length lists of unknown person ids and respective face vectors.
        """
        strangers = [person_id for person_id in self.messages_per_person if not person_id.is_known]
        if strangers:
            _, ids, face_vecs = zip(*)
            return ids, face_vecs
        return tuple(), tuple()


    def recalculate_best_guess(self) -> bool:
        """
        Determine which interlocutor has the best score right now,
         according to the messages currently buffered.
        :return: True if self.current_best_guess has changed. Note: current_best_guess
         might also change to None if no interlocutor with sufficient messages was found.
        """
        pass


    def convert_current_anonymous_to_known(self, primary_key):
        """
        If the current best guess is an anonymous interlocutor, convert it to
         a known interlocutor with the given primary key.
        """
        pass
