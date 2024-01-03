'''
AI Chat Module voice-to-text class
Randy Mackay, December 2023

AP_FLAKE8_CLEAN
'''

import time

try:
    import pyaudio  # install using, "sudo apt-get install python3-pyaudio"
    import wave     # install with "pip3 install wave"
    from openai import OpenAI
except Exception:
    print("chat: failed to import pyaudio, wave or openai.  See https://ardupilot.org/mavproxy/docs/modules/chat.html")
    exit()


class chat_voice_to_text():
    def __init__(self):
        # initialise OpenAI connection
        self.client = None
        self.assistant = None

    # set the OpenAI API key
    def set_api_key(self, api_key_str):
        self.client = OpenAI(api_key=api_key_str)

    # check connection to OpenAI assistant and connect if necessary
    # returns True if connection is good, False if not
    def check_connection(self):
        # create connection object
        if self.client is None:
            try:
                self.client = OpenAI()
            except Exception:
                print("chat: failed to connect to OpenAI")
                return False

        # return True if connected
        return self.client is not None

    # record audio from microphone
    # returns filename of recording or None if failed
    def record_audio(self):
        # Initialize PyAudio
        p = pyaudio.PyAudio()

        # Open stream
        try:
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=1024)
        except Exception:
            print("chat: failed to connect to microphone")
            return None

        # calculate time recording should stop
        curr_time = time.time()
        time_stop = curr_time + 5

        # record until specified time
        frames = []
        while curr_time < time_stop:
            data = stream.read(1024)
            frames.append(data)
            curr_time = time.time()

        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save audio file
        wf = wave.open("recording.wav", "wb")
        wf.setnchannels(1)
        wf.setsampwidth(pyaudio.PyAudio().get_sample_size(pyaudio.paInt16))
        wf.setframerate(44100)
        wf.writeframes(b''.join(frames))
        wf.close()
        return "recording.wav"

    # convert audio to text
    # returns transcribed text on success or None if failed
    def convert_audio_to_text(self, audio_filename):
        # check connection
        if not self.check_connection():
            return None

        # Process with Whisper
        audio_file = open(audio_filename, "rb")
        transcript = self.client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file,
            response_format="text")
        return transcript
