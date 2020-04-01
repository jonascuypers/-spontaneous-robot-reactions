import re

import threading

import time

import datetime
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.oauth2 import service_account

from src.analysis.language import Language


class StreamTranscriber(object):
    """Stream handlers that uses Google's speech to text to transcribe the audio stream."""

    # noinspection PyUnresolvedReferences
    def __init__(self, audio_stream_generator, credentials, language):

        self.audio_stream_generator = audio_stream_generator
        self.client = speech.SpeechClient(credentials=credentials)

        self.language = language

        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=audio_stream_generator.rate,
            language_code=self.language.get_language_code())

        self.streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=True
        )

    @staticmethod
    def create_credentials_from_api_key_file(path_to_api_key):
        return service_account.Credentials.from_service_account_file(path_to_api_key)

    def start(self):
        thread = threading.Thread(target=self.__handle_input_data())
        thread.start()

    def stop(self):
        pass

    def do_one_transcription(self):
        """Runs the streaming TTS service for a single sentence.

        The end of the sentence is detected by Google and signaled by the 'is_final' flag.

        Meta_info is a dict containing:
        {'start_time': time when transcription starts,
         'final_time' : time when the 'final' response is in
         'confidence' : confidence in the transcript (0..1)
        }

        :return: transcript, meta_info
        """
        start_time = time.time()
        # noinspection PyUnresolvedReferences
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in self.audio_stream_generator.generator())

        responses = self.client.streaming_recognize(self.streaming_config, requests)

        for response in responses:
            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            if result.is_final:
                print("final detected")
                final_time = time.time()
                return transcript, { "start_time": start_time,
                                     "final_time": final_time,
                                     "confidence": result.alternatives[0].confidence}

    def __handle_input_data(self):
        # noinspection PyUnresolvedReferences
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in self.audio_stream_generator.generator())

        responses = self.client.streaming_recognize(self.streaming_config, requests)

        # Now, put the transcription responses to use.
        self.listen_print_loop(responses)

    @staticmethod
    def listen_print_loop(responses):
        """Iterates through server responses and prints them.
        The responses passed is a generator that will block until a response
        is provided by the server.
        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.
        In this case, responses are provided for interim results as well. If the
        response is an interim one, print a line feed at the end of it, to allow
        the next result to overwrite it, until the response is a final one. For the
        final one, print a newline to preserve the finalized transcription.
        """
        num_chars_printed = 0
        for response in responses:
            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Display interim results, but with a carriage return at the end of the
            # line, so subsequent lines will overwrite them.
            #
            # If the previous result was longer than this one, we need to print
            # some extra spaces to overwrite the previous result
            overwrite_chars = ' ' * (num_chars_printed - len(transcript))

            if not result.is_final:
                print("[temp:] " + transcript)
                # sys.stdout.write(transcript + overwrite_chars + '\r')
                # sys.stdout.flush()

                num_chars_printed = len(transcript)

            else:
                print("[final:]" + transcript + overwrite_chars)

                # Exit recognition if any of the transcribed phrases could be
                # one of our keywords.
                if re.search(r'\b(exit|quit|stop)\b', transcript, re.I):
                    print('Exiting..')
                    break

                num_chars_printed = 0


def main():
    """Testing function

    This function sets up a StreamTranscriber so it can be tested in isolation. This code is not used
    in the dialogue framework itself.

    Note the hard coded IP address. This needs to be changed to fit your needs.

    You can either chose the 'nao_example' function or 'microphone_example' function (at the end of this main function)
    to chose your input source for the transcriber (your local microphone or a robot)

    :return: Nothing
    """

    path_to_api_key = "/home/christof/gitrepos/robot_dialogue/data/api_key.json"
    credential = StreamTranscriber.create_credentials_from_api_key_file(path_to_api_key)

    # Example with Nao stream
    def nao_example():
        import qi
        from streamgenerators.RobotAudioStream import RobotAudioStream
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + "tcp://10.10.131.10:9559"])
        app.start()
        session = app.session
        al_audio_device = session.service("ALAudioDevice")
        print("Start: " + str(datetime.datetime.now()))
        with RobotAudioStream(session, al_audio_device) as stream:
            handler = StreamTranscriber(stream, credential, Language.ENGLISH)
            print("Handler created:" + str(datetime.datetime.now()))
            handler.start()

    # Example with mic stream:
    def microphone_example():
        from streamgenerators.MircrophoneStream import MicrophoneStream
        with MicrophoneStream(5,
                              1024) as stream:  # input device index can be determined with the list_input_devices method.
            handler = StreamTranscriber(stream, credential, Language.ENGLISH)
            handler.start()

    # Choose one of the examples below:
    nao_example()
    # microphone_example()


if __name__ == "__main__":
    main()
