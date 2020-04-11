import pyaudio
import wave
import threading


class StreamRecorder(object):
    """Class that accepts a audio stream generator (RobotAudioStream, MicrophoneStream...) and records the input
    to a '.wav' file.

    """

    CHUNK = 1024
    FORMAT = pyaudio.paInt16

    def __init__(self, audio_stream_generator, output_file_path):
        self.audio_stream_generator = audio_stream_generator
        self.output_file_path = output_file_path
        self.frames = []
        self.py_audio = pyaudio.PyAudio()
        self.recording = False

    def __handle_input_data(self):
        input_frames = (frame for frame in self.audio_stream_generator.generator())
        for input_frame in input_frames:
            self.frames.append(str(input_frame))


    def start(self):
        self.recording = True
        thread = threading.Thread(target=self.__handle_input_data)
        thread.start()

    def stop(self):
        # This code is adapted to only get 431 frames for the sound recognition
        while len(self.frames) < 431:
            x = 1
        self.recording = False
        wave_file = wave.open(self.output_file_path, 'wb')
        wave_file.setnchannels(self.audio_stream_generator.channels)
        wave_file.setsampwidth(self.py_audio.get_sample_size(pyaudio.paInt16))
        wave_file.setframerate(self.audio_stream_generator.rate)
        # Only send the first 431 frames
        wave_file.writeframesraw(b''.join(self.frames[:431]))
        wave_file.close()
        self.frames = []  # empty frames again.

def main():
    """Main function showing example usage.

    This function sets up a StreamRecorder so it can be tested in isolation. This code is not used
    in the dialogue framework itself.

    Note: the following error output is expected and ok

    ALSA lib pcm_dsnoop.c:606:(snd_pcm_dsnoop_open) unable to open slave
    ALSA lib pcm_dmix.c:1029:(snd_pcm_dmix_open) unable to open slave
    ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
    ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.center_lfe
    ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.side
    ALSA lib pcm_dmix.c:1029:(snd_pcm_dmix_open) unable to open slave

    Note the hard coded IP address. This needs to be changed to fit your needs.

    You can either chose the 'nao_example' function or 'microphone_example' function (at the end of this main function)
    to chose your input source for the transcriber (your local microphone or a robot)

    :return: Nothing
    """

    # Example with Nao stream
    def nao_example():
        import qi
        import time
        from streamgenerators.RobotAudioStream import RobotAudioStream
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + "tcp://10.10.131.15:9559"])
        app.carb_count_start()
        session = app.session
        al_audio_device = session.service("ALAudioDevice")
        with RobotAudioStream(session, al_audio_device) as stream:
            recorder = StreamRecorder(stream, "output_nao.wav")
            recorder.start()
            time.sleep(3)
            recorder.stop()

    # Example with mic stream:

    def microphone_example():
        from streamgenerators.MircrophoneStream import MicrophoneStream
        with MicrophoneStream(7, 1024) as stream:  # input device index can be determined with the list_input_devices method.
            recorder = StreamRecorder(stream, "output_mic.wav")
            recorder.start()
            raw_input("press enter to stop recording")
            recorder.stop()

    # Choose one:
    microphone_example()
    # nao_example()


if __name__ == "__main__":
    main()
