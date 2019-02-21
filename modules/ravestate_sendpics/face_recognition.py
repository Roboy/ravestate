from typing import Optional
from reggol import get_logger
from numpy import ndarray
logger = get_logger(__name__)

PYROBOY_AVAILABLE = False
try:
    from pyroboy.face_recognition import FaceRec
    PYROBOY_AVAILABLE = True
except ImportError as e:
    import face_recognition as fr


def recognize_face_from_image_file(image_file: str) -> Optional[ndarray]:

    if PYROBOY_AVAILABLE:
        return FaceRec.get_biggest_face_encoding(image_file)
    else:
        logger.warning("Falling back to basic Face Recognition functions, since Pyroboy is unavailable!")
        image = fr.load_image_file(image_file)
        faces = fr.face_encodings(image)
        if faces:
            return faces[0]
        return None
