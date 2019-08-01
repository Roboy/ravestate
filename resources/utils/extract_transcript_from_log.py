from typing import List
import sys
import os

# This script will convert all *.log files within the given directory to easy-to-read transcripts

if len(sys.argv) != 2:
    print("Pass directory containing logs as command line parameter")
    exit(2)

logdir = sys.argv[1]
if not os.path.isdir(logdir):
    print(str(logdir) + " is not a directory")
    exit(3)

INPUT_TRIGGERS = ('INPUT: ', 'listen() -> ')
OUTPUT_TRIGGERS = ('OUTPUT: ', '.say(')

transcript_ctr = 0
for logfile_name in filter(lambda f: os.path.isfile(os.path.join(logdir, f)) and f.endswith(".log"), os.listdir(logdir)):
    with open(os.path.join(logdir, logfile_name), 'r') as logfile:
        log_arr: List[str] = list(filter(lambda l: any(t in l for t in INPUT_TRIGGERS + OUTPUT_TRIGGERS), logfile.read().split(sep='\n')))

    transcript: List[str] = []
    for line in log_arr:
        for inp_trig in INPUT_TRIGGERS:
            if inp_trig in line:
                inp = line.split(inp_trig)[1].strip()
                transcript.append("Interlocutor: " + inp)
        for out_trig in OUTPUT_TRIGGERS:
            if out_trig in line:
                # split(')') for .say(...) logs
                out = line.split(out_trig)[1].split(')')[0].strip()
                transcript.append("Ravestate: " + out)

    with open(os.path.join(logdir, 'transcript_' + logfile_name), 'w+') as outfile:
        outfile.write('\n'.join(transcript))
        transcript_ctr += 1

print("Created " + str(transcript_ctr) + " Transcripts")
