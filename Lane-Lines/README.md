Reflection on the current algorithm:

The current algorithm does not make use of any temporal information. Using that can greatly help in getting a better performance.

The cases when lane lines are missed because of shadow or illumination, the knowledge from previous frames can be used to set up a higher detection probability for the pixels where lane lines are expected to be for the current frame.

Also, the current algorithm makes use of Hough lines. This is a bit computational expensive. We can also use simply any edge detection
scheme (e.g. sobel operator) and work our way through that. 