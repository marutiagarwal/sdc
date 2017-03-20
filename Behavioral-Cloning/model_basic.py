from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers import Cropping2D

def BasicNet(input_shape):
    # build a regression network to predict steering angle
    model = Sequential()
    # 50 rows pixels from the top, 20 rows pixels from the bottom, 
    # 0 columns of pixels from the left, 0 columns of pixels from the right
    model.add(Cropping2D(cropping=((50,20), (0,0)), input_shape=input_shape))
    model.add(Lambda(lambda x: (x / 255.0) - 0.5))
    model.add(Flatten())
    model.add(Dense(1))
    return model

