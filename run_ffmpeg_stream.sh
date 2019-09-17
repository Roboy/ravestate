ffmpeg -f avfoundation -framerate 25 -i 0 -filter:v fps=fps=2 -f flv rtmp://127.0.0.1/live/faceoracle

