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

## Dependencies

VisionIO requires the following components to be running:

* `ravestate` or `raveboard` with `ravestate_visionio` module
* `Neo4j` backend for Scientio
* `redis` for persisting facial feature vectors
* `face_oracle` client and server 

## Configuration

VisionIO provides the following config keys:

Key | Default | Description
----|---------|--------------------------------------------
redis_host | Host for Redis database. | localhost
redis_port | Port for Redis database. | 6379
redis_pass | Password for Redis database | Empty
ros1-node | Topic for Faces messages. | /roboy/cognition/vision/visible_face_names
min-confidence | Minimum confidence below which someone will be a stranger. | 0.85

## How to run

We recommend running VisionIO through one of the ravestate
docker-compose profiles, which will start `Neo4j`, `redis`,
and the `face_oracle` client and server automatically.

Start the profile and visionio in docker as follows:

```bash
> docker-compose up -d {profile}
> docker exec -it rs bash
> python3 -m ravestate ...
```

__To start face recognition, open [localhost:8088/index.html](localhost:8088/index.html) in a browser (Chrome works best).__
This will give a visualisation of recognised faces, and simultaneously keep recognition running.
__Face recognition will only work as long as you can see it doing so!__

The docker-compose profiles differ per operating system: 

### Profile rs-linux

On Linux, a Webcam for VisionIO can simply be mapped into
docker as a device. Per default, this will be `video0`.

If you want to change the device, map a new device in
`docker-compose.yml`, and don't forget to change the
`FACEORACLE_VIDEO_DEVICE` variable.

### Profile rs-macos

On Mac, Docker can not natively access USB devices. Instead,
live video can be streamed into the container via RTMP:

1. Install/start [`Local RTMP Server`](https://github.com/sallar/mac-local-rtmp-server/releases)
2. Install `ffmpeg` via `brew install ffmpeg`
3. Stream webcam via RTMP by starting `ravestate/run_ffmpeg_stream.sh`

You can now start `docker-compose up -d rs-macos`.

### Using a video instead of a webcam feed

If you don't have a webcam, you can use a video instead for
debugging. Just set `FACEORACLE_VIDEO_DEVICE` for your
particular platform profile to `/ravestate/resources/obama.mp4`.
Note, that after changing `docker-compose.yml`, you have to run
`docker-compose up -d` with the `--force-recreate` flag.
