import numpy as np

import pyaudio
from six.moves import queue


class MicrophoneStream(object):
    """Object that presents the audio from one of your laptops input devices as an audio stream..

    This class is a resource and is hence supposed to be used with a 'with' statement.
    Furthermore it is implemented as a generator.

    See the main() function in this class for usage example.
    """

    def __init__(self, device_index, chunk, channels=1):
        """

        :param device_index: Device index. Use list_input_devices() in main to list your input devices
        :param chunk: Number of frames to buffer. Typical value is 1024.
        """

        self._audio_interface = pyaudio.PyAudio()
        self.device_index = device_index
        device_info = self._audio_interface.get_device_info_by_index(device_index)

        self.rate = int(device_info["defaultSampleRate"])
        self.chunk = chunk
        self.channels = channels

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        """Resource allocation function. Registers service and starts handling audio.

        This method is called when using this object in a 'with resource' statement.
        """
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=self.channels, rate=self.rate,
            input=True, frames_per_buffer=self.chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
            input_device_index=self.device_index
        )

        self.closed = False

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Resource de-allocation function. Unregister service and stop receiving audio.

        This method is called upon exiting a 'with resource' statement."""
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return in_data, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        print("Chunk none")
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


def main():
    """Testing function

    This function sets up a MicrophoneStream so it can be tested in isolation. This code is not used
    in the dialogue framework itself.

    Prints the list of available input devices followed by the RMS value received on the bound microphone.

    Note the hard coded input source. You will need to update this on your own machine.
    Note that the following warnings are expected and ok:

    ALSA lib pcm_dsnoop.c:606:(snd_pcm_dsnoop_open) unable to open slave
    ALSA lib pcm_dmix.c:1029:(snd_pcm_dmix_open) unable to open slave
    ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
    ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.center_lfe
    ALSA lib pcm.c:2266:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.side
    ALSA lib pcm_dmix.c:1029:(snd_pcm_dmix_open) unable to open slave

    :return: Nothing
    """

    def list_input_devices():
        print("Input devices:")
        py_audio = pyaudio.PyAudio()
        for i in range(py_audio.get_device_count()):
            device_info = py_audio.get_device_info_by_index(i)
            if device_info["maxInputChannels"] > 0L:
                print("\tInput device [{index}]\n"
                      "\t\tName: {name}\n"
                      "\t\tmaxInputChannels: {max_input_channels}\n"
                      "\t\tdefaultSampleRate: {default_sample_rate}"
                      .format(name=device_info["name"], index=device_info["index"],
                              max_input_channels=device_info["maxInputChannels"],
                              default_sample_rate=device_info["defaultSampleRate"]))

    def calc_rms_level(data):
        """
        Calculate RMS level
        """
        b = bytearray()
        b.extend(data)
        converted = convert_to_signed_int(b)
        rms = 20 * np.log10(np.sqrt(np.sum(np.power(converted, 2) / len(converted))))
        return rms

    def convert_to_signed_int(data):
        """
        This function takes a byte array containing 16 bits little endian sound
        samples as input and returns a vector containing the 16 bits sound
        samples values converted between -1 and 1.
        """
        signed_data = []
        ind = 0
        for i in range(0, len(data) / 2):
            signed_data.append(data[ind] + data[ind + 1] * 256)
            ind = ind + 2

        for i in range(0, len(signed_data)):
            if signed_data[i] >= 32768:
                signed_data[i] = signed_data[i] - 65536

        for i in range(0, len(signed_data)):
            signed_data[i] = signed_data[i] / 32768.0

        return signed_data

    list_input_devices()

    # --------------------------------------- #
    #             Example usage               #
    # --------------------------------------- #
    # Prints the RMS level of the microphone. Adjust your mic gain and see it work!
    input_source = 5    # As gotten from output of 'list_input_devices'
    with MicrophoneStream(input_source, 1024) as stream:
        for sample in stream.generator():
            # compute the rms level on front mic
            rms_mic_front = calc_rms_level(sample)
            print("rms level mic = " + str(rms_mic_front))

    # ---------- END example usage ---------- #


if __name__ == "__main__":
    main()
