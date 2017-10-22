
# Self-Driving Car Engineer Nanodegree

## Term 2 Project 1: Extended Kalman Filters

The goal for this project was to implement an extended Kalman filter using sensor fusion of RADAR and LIDAR. The project specification was given [here](https://review.udacity.com/#!/rubrics/748/view).

### Approach

I first needed to install the uWebSocketIO which was not that easy with the given install-mac.sh. But some research in the rubric forum led me to an updated bash procedure, which worked just fine.

Implementing the Kalman Filter was pretty straight forward using the starter code and the code given in the courses. So I didn't create any other pipeline but just filled in the missing code.

Compiling the code was sucessful and the simulation in the app ran with the first attempt.

### Challenges

Running dataset 1 I had no problems during the first counter-clockwise curve but the predictions took a 90 degree turn and moved away from the measurement path. I did some research and found [this discussion](https://discussions.udacity.com/t/rmse-value-to-high-for-new-data-file/241643/2). Therefore I changed atan to atan2 and included the normalising of the angles in the kalman_filter.cpp.

### Conclusion

This was my first project in C++ and I got used to it quite well. The project had two challenges for me installing the packages and having control over the angles. 
