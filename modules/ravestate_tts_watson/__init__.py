
from ravestate import registry
from ravestate_tts_watson import states

registry.register(name="tts_watson", states=(states.audio_talk,))
