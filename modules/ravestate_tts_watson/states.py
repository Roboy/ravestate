import ravestate_rawio
import pyaudio
import wave
import yaml

from ravestate.state import state
from ravestate_tts_watson.text_to_speech import Text2Speech

with open("resources/api.key") as keyfile:
    keys = yaml.load(keyfile)
t2s = Text2Speech(**keys)

CHUNK = 1024

audio = pyaudio.PyAudio()
def play_audio(waveform):
    # Open stream
    stream = audio.open(format=audio.get_format_from_width(waveform.getsampwidth()),
                        channels=waveform.getnchannels(),
                        rate=waveform.getframerate(),
                        output=True)

    # Read data
    data = waveform.readframes(CHUNK)

    # Play stream
    while len(data) > 0:
        stream.write(data)
        data = waveform.readframes(CHUNK)

    # stop stream
    stream.stop_stream()
    stream.close()


# Say the utterance
@state(read="rawio:out")
def audio_talk(ctx):
    raw_out = ctx["rawio:out"]
    t2s.synthesize(raw_out, "tts.wav")
    with wave.open("tts.wav", 'rb') as waveform:
        play_audio(waveform)


if __name__ == "__main__":
    t2s.synthesize("this is a long test", "tts.wav")
    with wave.open("tts.wav", 'rb') as waveform:
        play_audio(waveform)
