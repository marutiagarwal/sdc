import os, sys
import cv2
import csv
import numpy as np
import skimage
import skimage.io

from keras.optimizers import Adam, Adadelta

# verious network architectures
from model_basic import BasicNet
from model_lenet import LeNet
from model_nvidianet import NvidiaNet
# from model_nvidia_lstmnet import NvidiaLSTMNet

def augment_brightness_camera_images(image):
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)
    random_bright = 0.5+np.random.uniform()
    image1[:,:,2] = image1[:,:,2]*random_bright
    image1[:,:,2][image1[:,:,2]>255]  = 255
    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1

def add_random_shadow(image):
    top_y = 320*np.random.uniform()
    top_x = 0
    bot_x = 160
    bot_y = 320*np.random.uniform()
    image_hls = cv2.cvtColor(image,cv2.COLOR_RGB2HLS)
    shadow_mask = 0*image_hls[:,:,1]
    X_m = np.mgrid[0:image.shape[0],0:image.shape[1]][0]
    Y_m = np.mgrid[0:image.shape[0],0:image.shape[1]][1]

    shadow_mask[((X_m-top_x)*(bot_y-top_y) -(bot_x - top_x)*(Y_m-top_y) >=0)]=1
    #random_bright = .25+.7*np.random.uniform()
    if np.random.randint(2)==1:
        random_bright = .5
        cond1 = shadow_mask==1
        cond0 = shadow_mask==0
        if np.random.randint(2)==1:
            image_hls[:,:,1][cond1] = image_hls[:,:,1][cond1]*random_bright
        else:
            image_hls[:,:,1][cond0] = image_hls[:,:,1][cond0]*random_bright    
    image = cv2.cvtColor(image_hls,cv2.COLOR_HLS2RGB)
    return image

images = []
steering_angles = []
with open('data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for idx,line in enumerate(reader):
        # skip the header
        if idx==0:
            continue

        # store images
        img_center = skimage.io.imread(os.path.join('data/IMG', os.path.basename(line[0])))
        img_left   = skimage.io.imread(os.path.join('data/IMG', os.path.basename(line[1])))
        img_right  = skimage.io.imread(os.path.join('data/IMG', os.path.basename(line[2])))
        images.extend((img_center, img_left, img_right))

        # store steering_angles
        steering_center = float(line[3])
        
        # create adjusted steering measurements for the side camera images
        correction = 0.25 # this is a parameter to tune
        steering_left = steering_center + correction
        steering_right = steering_center - correction       
        steering_angles.extend((steering_center, steering_left, steering_right))

        # add the flipped images and angles as well
        images.extend((np.fliplr(img_center), np.fliplr(img_left), np.fliplr(img_right)))
        steering_angles.extend((-steering_center, -steering_left, -steering_right))


        # randomly select one of left/center/right image for adding brightness 
        ix = np.random.randint(3)
        if ix==0:
            image1 = augment_brightness_camera_images(img_left)
            images.append(image1)
            steering_angles.append(steering_left)
        elif ix==1:
            image2 = augment_brightness_camera_images(img_center)
            images.append(image2)
            steering_angles.append(steering_center)
        else:# ix==2:
            image3 = augment_brightness_camera_images(img_right)
            images.append(image3)
            steering_angles.append(steering_right)

        # randomly select one of left/center/right image for adding random shadow 
        ix = np.random.randint(3)
        if ix==0:
            image1 = add_random_shadow(img_left)
            images.append(image1)
            steering_angles.append(steering_left)
        elif ix==1:
            image2 = add_random_shadow(img_center)
            images.append(image2)
            steering_angles.append(steering_center)
        else:# ix==2:
            image3 = add_random_shadow(img_right)
            images.append(image3)
            steering_angles.append(steering_right)

# transform data into numpy array
X_train = np.array(images)
y_train = np.array(steering_angles)

# build a regression network to predict steering angle
model = NvidiaNet(input_shape=(160,320,3))

# Default, Keras trains for 10 epochs
learning_rate=0.001
#opt = Adam(lr=learning_rate)
opt = Adadelta(lr=learning_rate)
model.compile(loss='mse', optimizer=opt) # rmsprop
model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=50, verbose=2)

model.save('model.h5')
