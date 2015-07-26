Nicholas Robinson

Yosef Hoffman 

Zachary Bienenfeld

##AI for Robotics: Final Project

#Introduction
We had two approaches to solving the final project. The first approach was by using a Kalman Filter to track the trajectory of the robot over from the time that it leaves the frame and calculate the next 60 frames. The second approach that we took was that of a Particle Filter. The application of Kalman Filter to our scenario at hand is intuitive. Applying a Particle Filter requires a little bit of imagination but the results were significant. 

We tested our results by trimming the inputs by 60 frames (1739 frames instead of 1799) and testing the result against the expected 60 ending frames. We created two gauges for our accuracy. The first was an accuracy test as specified by the instructions for calculating the best score (Pythagorean theorem). This allowed us to gauge an objective score of how our algorithms were performing. However, the score itself was insufficient in gaining an understanding of where our robot was actually going and where it was deviating from its expected path. For that, we implemented a GUI which would pop up after a run through an input and would show the expected path versus the actual path plotted on a graph.  
#Overview 
###Kalman Filter
The Kalman Filter approach is the more classical approach to the question. Being that we have the previous movements, we can use a Kalman Filter to find a most likely next move as a Gaussian. There are two challenges which the Kalman Filter can be applied to resolve. The first is the 1 dimensional velocity of the robot. As the robot moves across the board it accelerates. When it crashes into the wall it is then slowed down and will once again gain momentum as it is free to roam. The second challenge is accounting for the 2 dimentional curving of the robot. 

As we studied in the class lectures, the Kalman Filter works with combining a measurement update (Bayes Rule) and a motion update (Total Probability). Where we are somewhat limited in the question at hand is that we never get a measurement feedback after each of our steps. We must make all of our 60 predictions without a single measurement update. In order to mitigate this problem and continue using the Kalman Filter, we must assume that each motion is the correct motion and continue with our next motion as if the measurement update retrieved was the same as the motion update. A somewhat different way of looking at it would be that the robot takes steps of 60 frames at a time and then gets feedback of its motion with a measurement. We are given the measurements up to N and must then calculate with the Kalman filter the next 60 frames. 

The Gaussian that is formed by the Kalman Filter will be a 2 dimensional oval. 1 of its axis signifies is the likelihood of how far it will go based on its velocity and acceleration, and the other axis signifies the likelihood of how much it will turn. The center will give us the highest probable location of the robot at its next frame.

In our research on Kalman Filters we came across a second form of the Kalman Filter known as the [Unscented Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter). This type of Kalman filter is useful in the case of non-linear problems. Since our robot problem fit this description this is the Kalman Filter that we decided to pursue.

###Particle Filter
The Particle Filter approach is an extension of the particle filter we studied in class. In the class lectures, we learned about how to find the location of a robot by placing particles around the entire area where the robot could possibly lie facing in assorted directions. As the robot moves around, we apply the same motion to the particles. When the robot senses its surroundings, we are given a better understanding of which particles are likely candidates for being in the same location as the robot. 

How would we apply such a concept to the question in the final project? We are given about 30 minutes of total training videos which give us real past movements of the robot. If we use our imagination, we can picture all of the previous frames in all of the previous trainings as particles. There are enough training frames given that we have a good sampling of robot motion in every location on the board and in every direction. We can then apply that sampling against the current state of our robot. As our robot moves, we can track which previous frames are likely candidates that represent our current scenario and let the others fall off. 

In the class lectures there was a re-sampling stage in which filters that became less likely to be correct robot positions joined teams with likely robot positions and formed a heavier weighted gathering in likely positions. So too in our use of the particle filter, the unlikely “particles” or frames get dropped off and the most likely portion of training video is selected as the closest match. Once we have the closest match, following the behavior is trivial. We can apply the exact movements that we have from the training and apply it to the robot. 

In some ways this use of the particle filter is the opposite of the particle filter studied in class. For in class, the particles were following the robot so that we can assess where the likely position of the robot is. In our case, the robot (the current state) is following the particles (the previous frames). And in our case, we know the exact location of the robot at the current state, we simply wish to derive from it which is the most likely particle filter (past frame) to trace. 

As discussed in class, switching between approaches mid-run can cause a robot to quickly jerk out of position. Averaging the approaches would remove the jerkiness but would be computationally heavier and would not offer a better result, rather would supply a result that is imprecise according to both of the the approaches. 

Each approach has its strength and weaknesses. The obvious strength of Kalman Filter over the Particle Filter is that the Particle Filter will only work if it finds a sufficient match in the training; if however there is no "particle" from which to base the current move, the Particle Filter will have trouble locating itself. For example, in the 7th input the robot turns over and for a period of 25 seconds is practically stationary. As we explain below, our algorithm includes the current video as training and uses them as particles. If however we chose not to use that implementation, rather to base solely on the training video, we would have no particle to base the current stationary movement on. The Kalman Filter on the other hand requires no such training. The Kalman Filter takes into account only its recent moves in ascertaining the velocity and turning direction of the robot. 
 
#Implementation
###Kalman Filter
Initially our approach to solving the problem via Kalman Filters was limited to trying to keep up with the velocity of the robot as it moved along. We began by keeping track of the position of the x,y coordinates as well as the velocity in the x and y direction. This method proved to be helpful but did not produce very good results. The issue we had when solving the problem this way was that when we got to the end of our measurement list and had to start predicting data points we could only predict a straight line as all we had to go on was velocity of the robot.

###Unscented Kalman Filter
After not getting the results we wanted from a basic Kalman Filter with a Constant Velocity model we decided to explore Unscented Kalman Filters. In our reading we found that UKF were particularly well suited for nonlinear problems and were relativley simple to implements. Our readings also explained that UKF can often have better results than Extended Kalman Filters. 

We decided to pursue using UKF with a Constant Acceleration model. We believed that if we could keep track of acceleration in the x and y direction that we could keep up with the strange curving patterns that we observed happening with the robot. The function below shows our state transition function. 

	def f_cv(x, dt):
    """ state transition function for a constant velocity aircraft"""

    F = np.array([[1, dt, (0.5 * (dt**2)),  0., 0., 0.],
                  [0, 1, dt,  0., 0., 0.],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0,  1, dt, (0.5 * (dt**2))],
                  [0, 0, 0, 0, 1, dt],
                  [0, 0, 0, 0, 0, 1]])
    return np.dot(F, x)
    
 We looked around on the internet to try to find additional models that might be helpful as well. We kept arriving back at this model due to the simplicity as well as the time limits we had on our program runtime. 
 
 We chose to use the [filterpy](https://filterpy.readthedocs.org/en/latest/) library as a basis for our Filter implementation. The library offers basic Kalman Filters as well as UKF and EKF. 

###Particle Filter

#Results
###Kalman Filter

Below is an example of a resulting prediction using the velocity/position Kalman Filter:

![Linear KF](https://s3.amazonaws.com/cs8803/linear_kf_02.png)

As you can see from the image our predictions look very good until we get to the point that our predictor goes blind and stops getting measurements. At that point we drift away from the desired result quite quickly. Tuning the filter helps to some extent, but for the most part this type of approach does not produce great results.

###Unscented Kalman Filter

The following image shows the results for test case 02. This image shows 10 frames before measurements stop and 20 frames after. As you can see the graph is turning in the direction of the last few measurements. The problem we ran into at this point is that no amount of tuning seemed to correct these wide swings in our predictions. The problem with the constant acceleration model is that our robot is very jittery and the acceleration is unfortunatley not very consistent. Due to this our UKF did not perform as well as expected. 

![UKF](https://s3.amazonaws.com/cs8803/ukf_02_small.png)

###Particle Filter
#Opportunities

We mentioned above that there is a weakness in the Particle Filter approach in that if there is no particle that matches the current robot, we are bound to get poor results. This was demonstrated in the class lectures when sometimes, the randomly generated particles simply did not land in the correct location/direction matching the robot; in those cases the percentage of error was very high. In our implementation, as mentioned, the same weakness occurs in that if the robot does not match up to any of the previous locations and facing directions of the videos, we will not get a good estimate for the future location of the robot.

This weakness of the Particle Filter could however be mitigated with a slightly smarter algorithm. If the algorithm would map every particle to the center of an x-y plane, and track the subsequent movements compared to the robot as if it too were positioned at the center of an x-y plane, then we would be effectively drawing from a much larger pool of possible particles. Previously we were looking to for the closest match of 1. location 2. velocity 3. acceleration and 4. turning direction. With the improved algorithm we would no longer search for a match of location (only the other 3), and would therefore find a much better match for velocity, acceleration and turning direction. Once we found a match we would simply apply the difference in location to the the subsequent moves of the found particle by adding those distances to the robot. 

For example, the current approach, as stated, will take the last 6 moves of the current robot and compare them against 6 sequential moves in the training. We may be faced with a robot whose last 6 moves are `((20,30), (21,30), (22,30), (23,30), (24,30), (25,30))`. We see that this robot is going straight at a rate of 1 unit per frame. Consider comparing that robot with the following two particles:

- Particle a: `((20,30), (22,30), (24,30), (26,30), (28,30), (30,30))`
- Particle b: `((100,50), (101,50), (102,50), (103,50), (104,50), (105,50))`

Particle 1 is clearly a closer match but if you look closer, that match is deceiving since only the locations are closer. If we map the starting point to 0, then you would get:

- Particle a: `((0,0), (2,0), (4,0), (6,0), 8,0), (10,0))`
- Particle b: `((0,0), (1,0), (2,0), (3,0), (4,0), (5,0))`

And now Particle b shows an exact match to our robot. All we have to do is subtract (80,20) from the particles.

The drawback to this approach is that the world we are given has walls (unlike a lot of the samples in class in which you fall off one side and appear on the other). That being the case, we would have to devise a system for accounting for the case where either the particle that we are copying hits a wall, or the robot approaches the bounds of the world. Since this adds a lot of complexity to the code and would likely provide minimal improvements in accuracy, we decided to not implement this approach.  

#Conclusion
