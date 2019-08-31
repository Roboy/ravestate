FROM missxa/melodic-dashing-roboy

# install pyroboy with melodic
RUN cd ~/melodic_ws/src && git clone https://github.com/Roboy/pyroboy.git && \
    cd ~/melodic_ws/src/pyroboy && git checkout melodic && cd ~/melodic_ws && \
    . /opt/ros/melodic/setup.sh && catkin_make && . /opt/ros/melodic/setup.sh

# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
ADD requirements2.txt /tmp/requirements2.txt
ADD requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install -r /tmp/requirements2.txt
RUN pip3 install -r /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements-dev.txt

# install speech recognition requirements and download speech recognition
# RUN apt install -y libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 python3-pyaudio
# RUN pip3 install webrtcvad monotonic SpeechRecognition pyaudio
# RUN cd /root/ros2_ws/src && git clone https://github.com/Roboy/ros2_speech_recognition.git

# download wildtalk and spacy models here instead of during container creation
RUN python3 -c "from roboy_parlai import wildtalk"
RUN python3 -c "from pytorch_pretrained_bert import cached_path; \
    cached_path('https://s3.amazonaws.com/models.huggingface.co/transfer-learning-chatbot/finetuned_chatbot_gpt.tar.gz')"
RUN python3 -c "from pytorch_transformers import GPT2Tokenizer, GPT2LMHeadModel; \
    GPT2Tokenizer.from_pretrained('gpt2-medium'); GPT2LMHeadModel.from_pretrained('gpt2-medium')"
RUN python3 -c "from spacy.cli import download as spacy_download; spacy_download('en_core_web_sm')"
