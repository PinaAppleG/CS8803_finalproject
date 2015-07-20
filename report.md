#Introduction
We had two approaches to solving the final project. The first approach was by using a Kalman Filter to track the trajectory of the robot over from the time that it leaves the frame and calculate the next 60 frames. The second approach that we took was that of a Particle Filter. The application of Kalman Filter to our scenario at hand is intuitive. Applying a Particle Filter requires a little bit of imagination but the results were significant. 
#Overview 
###Kalman Filter
The Kalman Filter approach is the more classical approach to the question. Being that we have the previous movements, we can use a Kalman Filter to find a most likely next move as a Gaussian. There are two challenges which the Kalman Filter can be applied to resolve. The first is the 1 dimensional velocity of the robot. As the robot moves across the board it accelerates. When it crashes into the wall it is then slowed down and will once again gain momentum as it is free to roam. The second challenge is accounting for the 2 dimentional curving of the robot. 

As we studied in the class lectures, the Kalman Filter works with combining a measurement update (Bayes Rule) and a motion update (Total Probability). Where we are somewhat limited in the question at hand is that we never get a measurement feedback after each of our steps. We must make all of our 60 predictions without a single measurement update. In order to mitigate this problem and continue using the Kalman Filter, we must assume that each motion is the correct motion and continue with our next motion as if the measurement update retrieved was the same as the motion update. A somewhat different way of looking at it would be that the robot takes steps of 60 frames at a time and then gets feedback of its motion with a measurement. We are given the measurements up to N and must then calculate with the Kalman filter the next 60 frames. 

The Gaussian that is formed by the Kalman Filter will be a 2 dimensional oval. 1 of its axis signifies is the likelihood of how far it will go based on its velocity and acceleration, and the other axis signifies the likelihood of how much it will turn. The center will give us the highest probable location of the robot at its next frame.
###Particle Filter
The Particle Filter approach is an extension of the particle filter we studied in class. In the class lectures, we learned about how to find the location of a robot by placing particles around the entire area where the robot could possibly lie facing in assorted directions. As the robot moves around, we apply the same motion to the particles. When the robot senses its surroundings, we are given a better understanding of which particles are likely candidates for being in the same location as the robot. 

How would we apply such a concept to the question in the final project? We are given about 30 minutes of total training videos which give us real past movements of the robot. If we use our imagination, we can picture all of the previous frames in all of the previous trainings as particles. There are enough training frames given that we have a good sampling of robot motion in every location on the board and in every direction. We can then apply that sampling against the current state of our robot. As our robot moves, we can track which previous frames are likely candidates that represent our current scenario and let the others fall off. 

In the class lectures there was a resampling stage in which filters that became less likely to be correct robot positions joined teams with likely robot positions and formed a heavier weighted gathering in likely positions. So too in our use of the particle filter, the unlikely “particles” or frames get dropped off and the most likely portion of training video is selected as the closest match. Once we have the closest match, following the behavior is trivial. We can apply the exact movements that we have from the training and apply it to the robot. 

In some ways this use of the particle filter is the opposite of the particle filter studied in class. For in class, the particles were following the robot so that we can assess where the likely position of the robot is. In our case, the robot (the current state) is following the particles (the previous frames). And in our case, we know the exact location of the robot at the current state, we simply wish to derive from it which is the most likely particle filter (past frame) to trace. 
---------- More to write here in terms of which one we select and how we chose one over other----------
#Implementation
###Kalman Filter

###Particle Filter

#Results
###Kalman Filter

###Particle Filter
#Opportunities

#Conclusion
