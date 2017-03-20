Here is a link to the video file obtained in "autonomous" model on using the trained model committed above.

https://youtu.be/xrfQ1odZzJw


**Training Dataset**

A center, left, and right images used in the training dataset:

![alt tag](https://github.com/marutiagarwal/sdc/blob/master/Behavioral-Cloning/images/training_images/center_2016_12_01_13_30_48_287.jpg)
![alt tag](https://github.com/marutiagarwal/sdc/blob/master/Behavioral-Cloning/images/training_images/left_2016_12_01_13_30_48_287.jpg)
![alt tag](https://github.com/marutiagarwal/sdc/blob/master/Behavioral-Cloning/images/training_images/right_2016_12_01_13_30_48_287.jpg)

**Steering Angles**

![alt tag](https://github.com/marutiagarwal/sdc/blob/master/Behavioral-Cloning/images/histograms/steering_angles_original.png)

As we can see above, the closer we are to 0 degree steering angle, the more samples we have in the training set. It makes sense as well since the car steering does not need to be turned fully to one side often.

Since the data is recorded from a real run in simulator, it did not require extreme angles as much. Looking at the above graph it feels as if we might have problem with unbalanced dataset and prediction might be biased towards center. But we observed experimentally, that the data works just fine and we are able to get a ncie and smooth ride in the car with this data.

Although, we augmented the data with bunch on preprocessing tricks. But the distribution of the steering angles didn't really change much.



