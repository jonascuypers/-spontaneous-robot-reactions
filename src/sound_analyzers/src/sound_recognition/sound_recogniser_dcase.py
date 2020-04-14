import numpy as np

import librosa

from keras.models import Model, Sequential
from keras.layers import Input, Dense, GlobalAveragePooling2D, GlobalMaxPooling2D, Lambda, \
    Flatten, Dropout, Conv2D, MaxPooling2D, BatchNormalization, Activation, TimeDistributed, \
    GRU, Reshape, Bidirectional, GlobalAveragePooling1D, GlobalMaxPooling1D, SpatialDropout2D, \
    Concatenate, Multiply
from keras import backend as K
import keras.backend.tensorflow_backend as tb

class SoundRecogniser:
    def __init__(self):
        self.load_sound_recognition()
        self.class_list = ['an Alarm bell ringing', 'Speech', 'a Dog', 'a Cat', 'a Vacuum_cleaner', 'Dishes', 'Frying', 'an Electric_shaver or a toothbrush', 'a Blender', 'Running_water']

    def define_model(self, at_layer_name='at_output', loc_layer_name='loc_output'):
        time_pooling_factor = 1

        input_shape = (64, 431, 1)

        melInput = Input(input_shape)

        # ---- mel convolution part ----
        mBlock1 = Conv2D(filters=64, kernel_size=(3, 3), padding="same")(melInput)
        mBlock1 = BatchNormalization()(mBlock1)
        mBlock1 = Activation(activation="relu")(mBlock1)
        mBlock1 = MaxPooling2D(pool_size=(4, 1))(mBlock1)
        # mBlock1 = Dropout(0.1)(mBlock1)
        mBlock1 = SpatialDropout2D(0.3, data_format=K.image_data_format())(mBlock1)

        mBlock2 = Conv2D(filters=64, kernel_size=(3, 3), padding="same")(mBlock1)
        mBlock2 = BatchNormalization()(mBlock2)
        mBlock2 = Activation(activation="relu")(mBlock2)
        mBlock2 = MaxPooling2D(pool_size=(4, time_pooling_factor))(mBlock2)
        mBlock2 = SpatialDropout2D(0.3, data_format=K.image_data_format())(mBlock2)
        # mBlock2 = Dropout(0.1)(mBlock2)

        mBlock3 = Conv2D(filters=64, kernel_size=(3, 3), padding="same")(mBlock2)
        mBlock3 = BatchNormalization()(mBlock3)
        mBlock3 = Activation(activation="relu")(mBlock3)
        mBlock3 = MaxPooling2D(pool_size=(4, time_pooling_factor))(mBlock3)
        mBlock3 = SpatialDropout2D(0.3, data_format=K.image_data_format())(mBlock3)
        # mBlock3 = Dropout(0.1)(mBlock3)

        targetShape = int(mBlock3.shape[1] * mBlock3.shape[2])
        mReshape = Reshape(target_shape=(targetShape, 64))(mBlock3)

        gru = Bidirectional(
            GRU(kernel_initializer='glorot_uniform', activation='tanh', recurrent_dropout=0.1, \
                dropout=0.1, units=64, return_sequences=True)
        )(mReshape)

        gru = Dropout(0.1)(gru)

        output = TimeDistributed(
            Dense(64, activation="relu"),
        )(gru)

        output = Dropout(0.1)(output)

        loc_output = TimeDistributed(
            Dense(10, activation="sigmoid"), name=loc_layer_name,
        )(output)

        # output = TimeDistributed(
        #  Lambda(lambda x: (x - K.min(x, axis=1, keepdims=True))/(K.max(x, axis=1, keepdims=True)- K.min(x, axis=1, keepdims=True)) ),
        # )(output)

        ### output = GlobalAveragePooling1D()(output)
        gap = GlobalAveragePooling1D()(loc_output)
        gmp = GlobalMaxPooling1D()(loc_output)
        # flat_gap = Flatten()(gap)
        # flat_gmp = Flatten()(gmp)

        concat = Concatenate()([gap, gmp])

        d = Dense(1024, activation="relu")(concat)
        d = Dropout(rate=0.5)(d)

        at_output = Dense(10, activation="sigmoid", name=at_layer_name)(d)

        self.model = Model(inputs=[melInput], outputs=[loc_output, at_output])

    def load_sound_recognition(self):
        weight_path = "/home/jonas/dcase19.90-0.1658-0.3292.h5"
        self.define_model(at_layer_name='at_output1', loc_layer_name='loc_output1')
        self.model.load_weights(weight_path)

    def recognise_sound(self, waf_file, threshold):
        print(waf_file)
        fpath = waf_file
        signal, sr = librosa.load(fpath, res_type='kaiser_fast')
        hop_length = 512
        #             # multiply by random factor for data aug
        #             if self.fact_amp > 0:
        #                 print('amp')
        #                 signal *= rand_amp_arr[i]

        power = librosa.feature.melspectrogram(y=signal,
                                               sr=sr,
                                               n_fft=2048,
                                               n_mels=64,
                                               fmin=0.0,
                                               fmax=sr / 2.0,
                                               htk=False,
                                               hop_length=hop_length,
                                               power=2.0,
                                               norm=1)
        power = librosa.core.power_to_db(power, ref=np.max)
        endpoint_time = np.min([power.shape[1], 431])
        x_test = power[:, :endpoint_time]
        x_test = x_test[np.newaxis, :, :, np.newaxis]
        # x_test.shape # (1, 64, 431, 1)
        tb._SYMBOLIC_SCOPE.value = True
        loc_probs, at_probs = self.model.predict(x_test)
        loc_probs.shape, at_probs.shape

        sounds = []
        for i, percentage in enumerate(at_probs[0]):
            if percentage > threshold:
                sounds.append((self.class_list[i], percentage))
        return sounds
