import pyaudio
import qi
from six.moves import queue
import numpy as np


# noinspection PyPep8Naming
class RobotAudioStream(object):
    """Object that presents the audio from one of Nao or Peppers microphones as an audio stream.

    This class is a resource and is hence supposed to be used with a 'with' statement.
    Furthermore it is implemented as a generator.

    See the main() function in this class for usage example.
    """

    def __init__(self, session, audio_device):
        """Constructor

        :param session: Naoqi session
        :param audio_device: ALAudioDevice proxy
        """
        super(RobotAudioStream, self).__init__()

        self.session = session

        self.al_audio_device = audio_device
        self.module_name = "RobotAudioStream"

        self.rate = 16000
        self.chunk = 1024
        self.channels = 1

        self.frames = queue.Queue()
        self.py_audio = pyaudio.PyAudio()
        self.__serviceId = -1

    def __enter__(self):
        """Resource allocation function. Registers service and starts handling audio.

        This method is called when using this object in a 'with resource' statement.
        """
        self.__serviceId = self.session.registerService(self.module_name, self)
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Resource de-allocation function. Unregister service and stop receiving audio.

        This method is called upon exiting a 'with resource' statement."""
        self.stop()
        self.session.unregisterService(self.__serviceId)

    def start(self):
        """
        Subscribe to audio data.
        From now on the processRemote method will be periodically called asynchronously when new audio data is available.
        """
        # ask for the front microphone signal sampled at 16kHz
        # if you want the 4 channels call setClientPreferences(self.module_name, 48000, 0, 0)
        self.al_audio_device.setClientPreferences(self.module_name, self.rate, 3, 0)
        self.al_audio_device.subscribe(self.module_name)

    def stop(self):
        """Unsubscribes from the audio data.
        From now on the processRemote method will no longer be called.
        """
        self.al_audio_device.unsubscribe(self.module_name)

        # Signal end of stream to consumers. This will make the generator return.
        self.frames.put(None)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        """Callback method that gets called by the robot when new audio data is available.
        This method is only called after subscription. See start method.
        """
        self.frames.put(str(inputBuffer))

        # Prevent queue overflow. Dump samples if they are not consumed fast enough
        while self.frames.qsize() > 100:
            self.frames.get_nowait()

    def generator(self):
        """Generator function that yields audio samples that are asynchronously added to the queue by the
        processRemote callback.

        If a chunk is None, this indicates the end of the stream and so the generator returns.
        """
        while True:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self.frames.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self.frames.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)


def main():
    """Testing function

    This function sets up a RobotAudioStream so it can be tested in isolation. This code is not used
    in the dialogue framework itself.

    Note the hard coded ip address. You will need to update this.

    Prints the RMS level received on the chosen microphone

    :return: Nothing
    """

    def calcRMSLevel(data):
        """
        Calculate RMS level
        """
        b = bytearray()
        b.extend(data)
        converted = convertStr2SignedInt(b)
        rms = 20 * np.log10(np.sqrt(np.sum(np.power(converted, 2) / len(converted))))
        return rms

    def convertStr2SignedInt(data):
        """
        This function takes a byte array containing 16 bits little endian sound
        samples as input and returns a vector containing the 16 bits sound
        samples values converted between -1 and 1.
        """
        signedData = []
        ind = 0
        for i in range(0, len(data) / 2):
            signedData.append(data[ind] + data[ind + 1] * 256)
            ind = ind + 2

        for i in range(0, len(signedData)):
            if signedData[i] >= 32768:
                signedData[i] = signedData[i] - 65536

        for i in range(0, len(signedData)):
            signedData[i] = signedData[i] / 32768.0

        return signedData

    # --------------------------------------- #
    #             Example usage               #
    # --------------------------------------- #
    connection_url = "tcp://10.10.131.10:9559"
    app = qi.Application(["RobotAudioStream", "--qi-url=" + connection_url])
    app.carb_count_start()
    session = app.session

    al_audio_device = session.service("ALAudioDevice")

    with RobotAudioStream(session, al_audio_device) as nao_stream:
        i = 0
        for sample in nao_stream.generator():
            # compute the rms level on front mic
            rms_mic_front = calcRMSLevel(sample)
            i += 1
            print("rms level mic front = " + str(rms_mic_front))
            if i > 10:
                break

    # ---------- END example usage ---------- #


if __name__ == "__main__":
    main()
