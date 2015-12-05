Date: 2013-05-03
Title: Self collision detection with Microsoft Kinect: Final Update

*by Michael Kosler*

## Project Summary

*Self collision detection with Microsoft Kinect* attempts to provide accurate
collision detection between a user's extremities and their own body. The overall
algorithm is broken down into three distinct steps:

1. Cull the user's depth data from the depth image provided by the Kinect;
2. Distinguish the extremities and the body into distinct objects;
3. Perform an edge detection algorithm on the pixels around the extremity 
to determine if there is collision or not.

The original idea came from a sketch idea in another class where we would turn
a person into a musical instrument by creating noise based on smacking different
sections of their body. After some discussion, we realized given the time we had
we would be unable to complete the project, but I really enjoyed the idea
and figured this final project might be a good place to flesh out the
necessary algoritms.

## Previous Work

Much of the previous work that I have found on the subject has been related
to collision detection with virtual objects [1](http://youtu.be/gD-O5Dm4hMs)
[2](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6398480).

## Dependencies

- [OpenNI](http://www.openni.org/);
- [NITE](http://www.openni.org/files/nite/#.UYPPRopDveU)
- [SensorKinect](https://github.com/avin2/SensorKinect);
- [SimpleOpenNI](https://code.google.com/p/simple-openni/);
- [Processing](http://processing.org/);

## Description of Work

In order to find self collision, we must first distinguish the user's depth data
from the surrounding space. OpenNI provides a user map after calibration, which
is an array of user ID's corresponding to the pixels in both the depth
and RGB images the Kinect provides. Combining the two, I can cull away the
user depth data from the remaining space. To calibrate, OpenNI requires the user
to stand in front of the camera and perform the "Psi" pose, which looks like the
pose you'd make if you were held at gunpoint. In addition to creating the user
map, this calibration also creates a user skeleton, composed of 20 coordinates
located at major joints on the body.

![Joint Map](http://3.bp.blogspot.com/-RKzkHXNxCQE/TgBuuTm_o5I/AAAAAAAAAuw/m4KiprVdCtE/s1600/7.png)

Even after calibration, however, we are still left with a mass of pixels, with
little distinction between the extremities and the main body. So, I added in
another calibration step. The user stands with the hand stretch out in front of
them, as if they were trying to stop something, and spreads their hand wide. Once
they have reached this pose, they wave at the camera. I could have done this
via a keypress, but this allows the user themselves to tell the program when they
are ready. After the wave is recognized, I create a bounding box around the hand,
which stays tracked for the remainer of the program.

The reason I have the user extend their hand is because I need to temporarily cull
away the rest of the body from the image in order to get the dimensions of their
hand. I do this second culling based on the depth value given from the hand's
joint position, provided from the user skeleton created during the first
calibration step. I then add in a threshold, which removes pixels from the image
if their depth data is not within the bounds of the threshold surrounding the hand
joint. By sticking the hand out, this effectively removes most of the noise around
the hand, allowing me to create the bounding box.

Concurrently, after the first calibration, I create a bounding box surrounding the
user's body - in this case extending shoulder to shoulder and head to foot.

Once both calibration steps are complete, the collision detection can begin. I
first do a broad phase collision detection based simply on the two bounding boxes
colliding. If they are in fact colliding, I then switch to a very simple edge
detection algorithm which checks the area of the image within the hand's bounding
box.

Why edge detection? Since I am using a depth map, I can find the contours of my 
hand by checking the changes in depth value from one pixel to another. If that
change is greater than some threshold, then I can flag that pixel as an "edge
pixel." However, when the user's hand is resting on their body, there is never
enough of a significant change to detect edge pixels. Given that, we can
say if there are no edge pixels (or very few of them), the we effectively have a
collision.


## Results and Analysis

http://www.youtube.com/embed/HC7yB-by0CI

The algorithm works fairly well. The above video does not show it, but fingers,
fists, and other arrangements also provide collision detection. It is also
decently fast. After calibration, the collision detection algorithm is quadratic
in runtime due to the edge detection algorithm needing to scan every pixel in
the hand's bounding box; for most practical purposes, and due to the current 
Kinect images producing at maximum 640 x 480 resolution images, the scan
area is rather small.

There are some issues with the work, however. The broad phase collision detection
is actually a necessity, rather than an attempt at improving the runtime. Since
I only do edge detection if the pixels are inside the user map, if the hand is not
near other user pixels, then it finds no edges, and thus signals a collision. If
I do not cull away the user depth pixels from the rest of the scene, this fixes
that issue, but if I try do collision detection near the edge of my body, then
I get edge pixels from my body and/or arm to the background. To me, this issue
was less desirable than the issue it fixed, so I added in the bounding box 
collision detection to simply turn on the edge detection only when I am clearly
close enough the user.

If I, or another person, were to continue with this work, I think the overall
algorithm would be greatly improved by adding better extremity/body distinction.
The bounding boxes provide too much area where false positives or false negatives
happen. If there was better differentiation, then they could simply throw
out the edge pixels if the two compared pixels are not one body pixel and one
extremity pixel.

## Source Code

You can find the source code on its [Github Project page](https://github.com/mkosler/KinectCollision).
