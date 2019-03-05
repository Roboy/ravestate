FROM missxa/bouncy-roboy

# install ravestate dependencies
ADD requirements.txt /tmp/requirements.txt
ADD requirements-dev.txt /tmp/requirements-dev.txt
RUN pip3 install -r /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements-dev.txt