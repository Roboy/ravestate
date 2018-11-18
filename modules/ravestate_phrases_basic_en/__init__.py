
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)), "en"))
