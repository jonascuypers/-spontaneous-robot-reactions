import threading

import pyaudio


class StreamPlayer(object):
    """
    Class that accepts an audio stream (MicrophoneStream, RobotAudioStream...) and plays it back through
    a chosen output device. The output device is chosen via its device index. The available output
    devices can be listed through the list_output_devices method.

    """

    FORMAT = pyaudio.paInt16
    CHUNK = 1024
    CHANNELS = 1

    def __init__(self, input_generator, output_device_index):
        self.output_device_index = output_device_index
        self.output_stream = None
        self.input_generator = input_generator
        self.py_audio = None
        self.thread = None
        self.play = False

    def __enter__(self):
        self.py_audio = pyaudio.PyAudio()
        self.output_stream = self.py_audio.open(
            format=StreamPlayer.FORMAT,
            channels=StreamPlayer.CHANNELS,
            rate=self.input_generator.rate,
            output=True,
            frames_per_buffer=StreamPlayer.CHUNK,
            output_device_index=self.output_device_index)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.output_stream.close()
        self.py_audio.terminate()

    def __handle_data(self):
        """Handling method that pipes samples from the input to the output.

        This method should be executed in a separate thread so playback can be stopped.

        :return: Nothing
        """
        audio_generator = self.input_generator.generator()
        generated_data = (content for content in audio_generator)
        for frame in generated_data:
            if self.play:
                self.output_stream.write(frame)
            else:
                break

    def start(self, keep_alive=False):
        """Starts playback of the input source.

        If the keep_alive argument is set to true, the program will be kept alive, continuing to play the input
        even after the main thread has finished.

        :param keep_alive: Whether the background thread should be kept alive.
        :return: Nothing.
        """
        self.play = True
        self.thread = threading.Thread(target=self.__handle_data, args=())
        self.thread.daemon = not keep_alive
        self.thread.start()

    def stop(self):
        """Stops playback of the input source.

        Stopping the playback will also abort the playback thread so the program can exit cleanly (no matter
        what the value was of the 'keep_alive' flag passed to 'start'.

        :return: Nothing
        """
        self.play = False
    
    def list_output_devices(self):
        print("Output devices:")
        audio_lib = pyaudio.PyAudio()
        for i in range(audio_lib.get_device_count()):
            device_info = audio_lib.get_device_info_by_index(i)
            if device_info["maxOutputChannels"] > 0L:
                print("\tOutput device [{index}]\n"
                      "\t\tName: {name}\n"
                      "\t\tmaxOutputChannels: {max_output_channels}\n"
                      "\t\tdefaultSampleRate: {default_sample_rate}"
                      .format(name=device_info["name"], index=device_info["index"],
                              max_output_channels=device_info["maxOutputChannels"],
                              default_sample_rate=device_info["defaultSampleRate"]))
    
    def list_input_devices(self):
        print("Input devices:")
        for i in range(self.py_audio.get_device_count()):
            device_info = self.py_audio.get_device_info_by_index(i)
            if device_info["maxInputChannels"] > 0L:
                print("\tInput device [{index}]\n"
                      "\t\tName: {name}\n"
                      "\t\tmaxInputChannels: {max_input_channels}\n"
                      "\t\tdefaultSampleRate: {default_sample_rate}"
                      .format(name=device_info["name"], index=device_info["index"],
                              max_input_channels=device_info["maxInputChannels"],
                              default_sample_rate=device_info["defaultSampleRate"]))


def main():
    """Testing function

    This function sets up a StreamPlayer so it can be tested in isolation. This code is not used
    in the dialogue framework itself.

    Note the hard coded IP address. This needs to be changed to fit your needs.

    You can either chose the 'nao_example' function or 'microphone_example' function (at the end of this main function)
    to chose your input source for the transcriber (your local microphone or a robot)

    :return: Nothing
    """
    
    # Example with Nao stream
    def nao_example():
        import qi
        from streamgenerators.RobotAudioStream import RobotAudioStream
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + "tcp://10.10.131.15:9559"])
        app.carb_count_start()
        session = app.session
        al_audio_device = session.service("ALAudioDevice")
        with RobotAudioStream(session, al_audio_device) as stream:
            with StreamPlayer(stream, 6) as handler:
                handler.start()

    # Example with mic stream:
    def microphone_example():
        from streamgenerators.MircrophoneStream import MicrophoneStream
        with MicrophoneStream(5, 1024) as stream:   # input device index can be determined with the list_input_devices method.
            with StreamPlayer(stream, 6) as handler:
                handler.start()

    # Choose one of the examples below:
    # nao_example()

if __name__ == '__main__':
    main()


