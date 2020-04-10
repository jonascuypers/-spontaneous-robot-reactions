# voice_emotion_recognition.py
# public-domain sample code by Vokaturi, 2019-05-31
#
# A sample script that uses the VokaturiPlus library to extract the emotions from
# a wav file on disk. The file has to contain a mono recording.
#
# Call syntax:
#   python3 voice_emotion_recognition.py path_to_sound_file.wav
#
# For the sound file hello.wav that comes with OpenVokaturi, the result should be:
#	Neutral: 0.760
#	Happy: 0.000
#	Sad: 0.238
#	Angry: 0.001
#	Fear: 0.000

import scipy.io.wavfile
import Vokaturi


class VoiceEmotionRecognition:
	def __init__(self):
		print ("Loading library...")
		Vokaturi.load("/home/jonas/master_ws/src/voice_emotion_detection/src/lib/open/linux/OpenVokaturi-3-3-linux64.so")
		print ("Analyzed by: %s" % Vokaturi.versionAndLicense())

	def emotions_from_file(self, file_name):
		print ("Reading sound file...")
		(sample_rate, samples) = scipy.io.wavfile.read(file_name)
		print ("sample rate %.3f Hz" % sample_rate)

		print ("Allocating Vokaturi sample array...")
		buffer_length = len(samples)
		print ("   %d samples, %d channels" % (buffer_length, samples.ndim))
		c_buffer = Vokaturi.SampleArrayC(buffer_length)
		if samples.ndim == 1:  # mono
			c_buffer[:] = samples[:] / 32768.0
		else:  # stereo
			c_buffer[:] = 0.5*(samples[:,0]+0.0+samples[:,1]) / 32768.0

		print ("Creating VokaturiVoice...")
		voice = Vokaturi.Voice (sample_rate, buffer_length)

		print ("Filling VokaturiVoice with samples...")
		voice.fill(buffer_length, c_buffer)

		print ("Extracting emotions from VokaturiVoice...")
		quality = Vokaturi.Quality()
		emotion_probabilities = Vokaturi.EmotionProbabilities()
		voice.extract(quality, emotion_probabilities)
		voice.destroy()
		if quality.valid:
			print ("Neutral: %.3f" % emotion_probabilities.neutrality)
			print ("Happy: %.3f" % emotion_probabilities.happiness)
			print ("Sad: %.3f" % emotion_probabilities.sadness)
			print ("Angry: %.3f" % emotion_probabilities.anger)
			print ("Fear: %.3f" % emotion_probabilities.fear)
			emotions = [
				["Neutral", emotion_probabilities.neutrality],
				["Happy", emotion_probabilities.happiness],
				["Sad", emotion_probabilities.sadness],
				["Angry", emotion_probabilities.anger],
				["Fear", emotion_probabilities.fear]
			]
			return emotions
		else:
			print ("Not enough sonorancy to determine emotions")