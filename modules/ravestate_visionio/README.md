```
       _      _             _____ ____  
      (_)    (_)           |_   _/ __ \ 
 _   _ _  ___ _  ___  _____  | || |  | |
| | | | |/___) |/ _ \|  _  \ | || |  | |
 \ V /| |___ | ( (_) ) | | |_| || |__| |
  \_/ |_(___/|_|\___/|_| |_)_____\____/ 
                                                                                                                      
```

## VisionIO

The VisionIO module enables ravestate to connect to a Face Recognition
ROS topic provided by `face_oracle`, based on which conversations can be initiated.

## Architecture

```
OpenCV Video Stream
----> (     FaceOracle Client   )
      (webcam_video_processor.py) ---> ravestate_visionio
        |                     A        /roboy/cognition/vision/visible_face_names
        |                     |        (ROS1)
        V                     |
  face_recognition            |
        |              name-confidence pairs
        V                     |
  facial feature vector       |
        |                     |
        V                     |
     (FaceOracle Websocket Server)
        |                     A
        V                     |
  Load FaceVector-PrimKey     |
  pairs from Redis            |
        |                     |
        V                     |
  Match with Request          |
  Vector via Pyroboy          |
  FaceRec.match_face          |
        |                     |
        V                     |
   [ Name for PrimKey from Scientio ]
```

## How to run

Add the `ravestate_visionio` module to your ravestate session. For recognition, `ravestate_visionio` requires a running FaceOracle websocket client.

The FaceOracle websocket client is installed inside the ravestate docker container.
The client processes camera frames with OpenCV and `face_recogniton`, and
streams the results to the `/roboy/cognition/vision/visible_face_names` ROS1 topic.

When running the `ravestate` image via Docker compose (see main README), you can
navigate to `~/melodic_ws/src/face_oracle` and start the FaceOracle client with a parameter
indicating the camera you want to process. The parameter value depends on your platform:

### Linux

On Linux, on both Audio and Video devices are easily mapped into the container
via the provided docker-compose configuration. Inside the container, just execute...

```bash
# 1/3) In Docker: Make sure that `roscore` is running

roscore &

# 2/3) In Docker: Launch `webcam_video_processor.py` on video device `0` (Change number for different cam)

python webcam_video_processor.py -i 0 &

# 3/3) On Host: Connect to `localhost:5000` in your browser to start video streaming.
```

### MacOS

On macOS, devices (such as webcams) cannot be easily mapped into Docker. Therefore, we use `ffmpeg` and `RTMP` to stream video into the ravestate docker container, which can then be picked up by the face oracle client component.

**Requirements:**

- `ffmpeg` (`brew install ffmpeg`)
- `Local RTMP Server` ([See Releases on GitHub](https://github.com/sallar/mac-local-rtmp-server/releases))

**Running:**

```bash
# 1/5) On Host: Launch `Local RTMP Server`
# 2/5) On Host: run `ffmpeg` to stream video from your Facetime camera to Docker 

ffmpeg -f avfoundation -framerate 30 -i 0 -filter:v fps=fps=2 -f flv rtmp://127.0.0.1/live/facetime   
#Note: `"0"` refers to device with index 0. Change to stream from a different camera.`

# 3/5) In Docker: Make sure that roscore is running

roscore &

# 4/5) In Docker: Launch `webcam_video_processor.py` listening to `rtmp://127.0.0.1/live/facetime`

python webcam_video_processor.py -i "rtmp://host.docker.internal/live/facetime" &

# 5/5) On Host: Connect to `localhost:5000` in your browser to start video streaming.
```

### Windows

Currently unexplored.
