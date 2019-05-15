FROM missxa/melodic-crystal-roboy

# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
ADD requirements-dev.txt /tmp/requirements-dev.txt
RUN pip3 install -r /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements-dev.txt

# install speech recognition requirements
ADD requirements-stt.txt /tmp/stt_requirements.txt
RUN pip3 install -r /tmp/stt_requirements.txt
RUN apt install -y python3-pyaudio
