FROM missxa/melodic-dashing-roboy

# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
ADD requirements-dev.txt /tmp/requirements-dev.txt
RUN pip3 install -r /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements-dev.txt

# install speech recognition requirements and download speech recognition
RUN apt install -y libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 python3-pyaudio
RUN pip3 install webrtcvad monotonic SpeechRecognition pyaudio
RUN cd /root/ros2_ws/src && git clone https://github.com/Roboy/ros2_speech_recognition.git
