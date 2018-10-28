import dialogic_rawio
import pyaudio
import wave
import yaml

from dialogic import registry
from dialogic.state import state
from dialogic_speech.speech_to_text import Speech2Text
from dialogic_speech.text_to_speech import Text2Speech
from dialogic_verbaliser.verbaliser import Verbaliser


with open("../../api.key") as keyfile:
    keys = yaml.load(keyfile)

s2t = Speech2Text(keys['s2t_username'], keys['s2t_password'], keys['s2t_url'])
t2s = Text2Speech(keys['t2s_username'], keys['t2s_password'], keys['t2s_url'])

audio = pyaudio.PyAudio()
verbaliser = Verbaliser()
CHUNK = 1024
RECORD_SECONDS = 7
WAVE_OUTPUT_FILENAME = "file.wav"
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100


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


def record_audio():
    # Open stream
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=RATE, input=True,
                        frames_per_buffer=CHUNK)

    # Read data
    frames = []
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    # stop stream
    stream.stop_stream()
    stream.close()
    audio.terminate()

    with open(WAVE_OUTPUT_FILENAME, 'wb') as waveform:
        waveform = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        waveform.setnchannels(CHANNELS)
        waveform.setsampwidth(audio.get_sample_size(FORMAT))
        waveform.setframerate(RATE)
        waveform.writeframes(b''.join(frames))
        return waveform


@state(read="rawio:in")
def audio_shutdown(sess):
    raw_in = sess["rawio:in"]
    if "bye" in raw_in:
        # Say bye
        waveform = wave.open(t2s.synthesize(verbaliser.bye()), 'rb')
        play_audio(waveform)
        sess.shutdown()


@state(read="rawio:out")
def audio_talk(sess):
    raw_out = sess["rawio:out"]
    # Say the utterance
    waveform = wave.open(t2s.synthesize(raw_out), 'rb')
    play_audio(waveform)


@state(write="rawio:in", triggers="rawio:out:changed")
def audio_listen(sess):
    # Recognize audio
    waveform = record_audio()
    text = s2t.recognise(waveform)
    sess["rawio:in"] = text


registry.register(
    name="audioio",
    states=(audio_shutdown, audio_listen, audio_talk)
)
