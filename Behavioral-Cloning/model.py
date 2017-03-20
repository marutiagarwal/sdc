from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten, Lambda
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Cropping2D

def LeNet(input_shape):
    # build a regression network to predict steering angle
    model = Sequential()
    # 50 rows pixels from the top, 20 rows pixels from the bottom, 
    # 0 columns of pixels from the left, 0 columns of pixels from the right
    model.add(Cropping2D(cropping=((50,20), (0,0)), input_shape=input_shape))
    model.add(Lambda(lambda x: (x / 255.0) - 0.5))
    
    #conv1
    model.add(Convolution2D(24, 5, 5, subsample=(2,2)))    
    model.add(Activation('relu'))

    #conv2
    model.add(Convolution2D(36, 5, 5, subsample=(2,2)))    
    model.add(Activation('relu'))

    #conv3
    model.add(Convolution2D(48, 5, 5, subsample=(2,2)))    
    model.add(Activation('relu'))

    #conv4
    model.add(Convolution2D(64, 3, 3))    
    model.add(Activation('relu'))

    #conv5
    model.add(Convolution2D(64, 3, 3))    
    model.add(Activation('relu'))

    # FC Layers
    model.add(Flatten())
    
    # FC1
    model.add(Dense(100))
    model.add(Activation('relu'))

    # FC2
    model.add(Dense(50))
    model.add(Activation('relu'))

    # FC3
    model.add(Dense(10))
    model.add(Activation('relu'))
            
    model.add(Dense(1))
    return model

