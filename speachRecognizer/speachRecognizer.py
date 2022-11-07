#
#  Implement a speach recognition frontend that places recognized commands in a file
#
#  Because python3 does not support threading this is run in separate process
#  and appends to a speah output file which some other app may read for commands.
#
#  This uses vosk speach recognition from https://alphacephei.com/vosk
#  Requires: pip3 install vosk    and   sudo apt install python3-pyaudio
#  Developed on 5-15-2022 Ubiquity Robotics Pi4 image that uses Ubuntu 20.04
#
#  You will need to define a path to the speach output text file
#  and will need to download a vosk speach model and define path to that below
#
#  20221106     mjstn2011@gmail.com    Formed 
#
#  Speach recognizer basics thanks to Brandon Jacobsons https://www.youtube.com/watch?v=3Mga7_8bYpw
#

import sys
import os
import time

# speach stuff
from vosk import Model, KaldiRecognizer
import pyaudio

# Define output for recognized speach. Use empty string for Path if no output required
# You MUST create this file even if empty as this program does not auto-create it
# which may destroy prior file so best to do that 'above' this program
createSpeachFile = 1      # when set to 1 we will create the file if the path exists
speachOutputPath = "/home/ubuntu/audioprocessing/"
speachOutputFile = speachOutputPath + "speachOutFile.txt"
outputFileExists = os.path.isfile(speachOutputFile)
outputPathExists = os.path.exists(speachOutputPath)
if outputPathExists:
    print("Speach path exists: ",outputPathExists)
    if outputFileExists == True:
        print("output file exists") 
    else:
        print("output file does not exist") 
        if createSpeachFile == 1:
            outFile = open(speachOutputFile, "w")
            outFile.write("\n")
            outFile.close()
            outputFileExists = os.path.isfile(speachOutputFile)


exit

#  Define the speach model vosk will use to process audio input
model = Model(r"/home/ubuntu/vosk-models/vosk-model-small-en-us-0.15")
recognizer = KaldiRecognizer(model, 44100)

# Setup microphone for sampling with pyaudio
micSampleRate = 44100
micDataChunkSize  = 8192
mic = pyaudio.PyAudio()
micInputInfo = mic.get_default_input_device_info()
print("Mic input info: ",micInputInfo)
stream = mic.open(format=pyaudio.paInt16, channels=1, rate=micSampleRate, input=True, frames_per_buffer=micDataChunkSize)

stream.start_stream()

# Next we start a loop to continuously listen and interprit
# Each time we get some text we output it to a command file for other process to 'hear'
while True:
    try:
        # get data from mic stream. We use exception_on_overflow = False)
        # because as root this read failed. 
        # See stackexchange.com/question/10733903/pyaudio-input-overflowed
        # where post 33 says to add this option to stream.read()
        data = stream.read(micDataChunkSize, exception_on_overflow = False)

        # if len(data) == 0:
        #     break           # optional break out of this for no data error
        if recognizer.AcceptWaveform(data):
            recognizerRecord = recognizer.Result()

            # The recognizer outputs a currly brace struct so we need to peel off front and back
            # This is a bit 'hacky' however to properly parse this would be 'nasty' 
            text = recognizerRecord[14:-3]

            print(text)
            if len(text) >= 1 and text.find("huh") == -1:     # suppress unrecognized words
                print("Heard: ",text)
                if outputFileExists == True:
                    try:
                        # Shown are 2 ways to output to a file but file.write() is cleanest
                        outLine = text + "\n"
                        outFile = open(speachOutputFile, "a")
                        outFile.write(outLine)
                        outFile.close()

                        # cmdLine = "echo " + text + " >> " + speachOutputFile
                        # print(cmdLine)
                        # os.system(cmdLine)
                    except:
                        print("Cannot write to speach output fild!")
    except KeyboardInterrupt:
        print ("\nKeyboard Abort!")
        break
