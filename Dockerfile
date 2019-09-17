FROM missxa/melodic-dashing-roboy

# PLEASE INSERT ADDITIONAL LAYERS AT THE END OF THE FILE

# ------------------------------------------------------
# install neo4j
RUN wget -O - https://debian.neo4j.org/neotechnology.gpg.key | apt-key add -
RUN echo 'deb https://debian.neo4j.org/repo stable/' | tee /etc/apt/sources.list.d/neo4j.list
RUN apt-get update && apt-get install -y neo4j
RUN neo4j-admin set-initial-password test

# ------------------------------------------------------
# install redis
RUN apt-get install -y redis

# ------------------------------------------------------
# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
ADD requirements-dev.txt /tmp/requirements-dev.txt
RUN pip3 install -r /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements-dev.txt

# ------------------------------------------------------
# download wildtalk and spacy models
RUN python3 -c "from roboy_parlai import wildtalk"
RUN python3 -c "from pytorch_pretrained_bert import cached_path; \
    cached_path('https://s3.amazonaws.com/models.huggingface.co/transfer-learning-chatbot/finetuned_chatbot_gpt.tar.gz')"
RUN python3 -c "from pytorch_transformers import GPT2Tokenizer, GPT2LMHeadModel; \
    GPT2Tokenizer.from_pretrained('gpt2-medium'); GPT2LMHeadModel.from_pretrained('gpt2-medium')"
RUN python3 -c "from spacy.cli import download as spacy_download; spacy_download('en_core_web_sm')"

# ------------------------------------------------------
# install pyroboy with melodic
# add github repo metadata to bust cache when repo is updated
# => bad idea due to github API rate limit, fails Travis builds now and then.
# ADD https://api.github.com/repos/roboy/pyroboy/git/refs/heads/melodic pyroboy_version.json
RUN cd ~/melodic_ws/src && git clone https://github.com/Roboy/pyroboy.git && \
    cd ~/melodic_ws/src/pyroboy && git checkout melodic && \
    cd ~/melodic_ws/src/roboy_communication && git pull && \
    cd ~/melodic_ws && . /opt/ros/melodic/setup.sh && catkin_make && . /opt/ros/melodic/setup.sh

# ------------------------------------------------------
# install face_oracle
RUN pip3 install ecdsa
RUN pip install -U face_recognition websocket_client pillow opencv-python numpy
# add github repo metadata to bust cache when repo is updated
# => bad idea due to github API rate limit, fails Travis builds now and then.
# ADD https://api.github.com/repos/roboy/face_oracle/git/refs/heads/visionio_messages faceoracle_version.json
RUN cd ~/melodic_ws/src && git clone https://github.com/Roboy/face_oracle.git && \
    cd ~/melodic_ws/src/face_oracle && git checkout visionio_messages && git pull && cd ~/melodic_ws && \
    . /opt/ros/melodic/setup.sh && catkin_make && . /opt/ros/melodic/setup.sh

# install speech recognition requirements and download speech recognition
# RUN apt install -y libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 python3-pyaudio
# RUN pip3 install webrtcvad monotonic SpeechRecognition pyaudio
# RUN cd /root/ros2_ws/src && git clone https://github.com/Roboy/ros2_speech_recognition.git
