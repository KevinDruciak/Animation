#### Assignment 4: Keyframe Animation

###### Kevin Druciak

kdrucia1

Late Days: 0

MSVC / Visual Studio Community 2017 /  Windows 10



#### Summary:

- Overall, I am happy about the outcome of this assignment. Unfortunately, I felt quite limited by my inability to efficiently create .ray and .key files to play around with. 

#### Implemented Features:

- all --parameter and --interpolant arguments
- video recording using snapshots and ray traced renders

#### Unimplemented Features:

- camera path spline
- key file merging
- composite motions

#### Parameterizations and Interpolation:

Here I have generated a 100 frame, 20 fps gif for each of the possible combinations and parameterizations and interpolations. The row labels are as follows:

- `1`: Rotations are represented by 3x3 matrices, the matrices are blended, and the blended transformation (not necessarily rotation) is returned.
- `2`: Rotations are represented by 3x3 matrices, the matrices are blended, and the closest rotation to the blended transformation is returned.
- `3`: Rotations are represented by Euler angles, the Euler angles are blended, and the rotation described by the blended Euler angles is returned.
- `4`: Rotations are represented by skew-symmetric matrices, the skew-symmetric matrices are blended, and the exponential of the blended skew-symmetric matrix is returned.
- `5`: Rotations are represented by quaternions, the quaternions are blended, and the rotation described by the normalized blended quaternion is returned.

|       |                 Nearest Point Interpolation                  |                     Linear Interpolation                     |                  Catmull-Rom Interpolation                   | Uniform Cubic B-spline Approx.                               |
| :---: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: | ------------------------------------------------------------ |
| **1** | <img src="kdrucia1_HTML\act\11.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\12.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\13.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\14.gif" alt="0" style="zoom:50%;" /> |
| **2** | <img src="kdrucia1_HTML\act\21.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\22.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\23.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\24.gif" alt="0" style="zoom:50%;" /> |
| **3** | <img src="kdrucia1_HTML\act\31.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\32.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\33.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\34.gif" alt="0" style="zoom:50%;" /> |
| **4** | <img src="kdrucia1_HTML\act\41.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\42.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\43.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\44.gif" alt="0" style="zoom:50%;" /> |
| **5** | <img src="kdrucia1_HTML\act\51.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\52.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\53.gif" alt="0" style="zoom:50%;" /> | <img src="kdrucia1_HTML\act\54.gif" alt="0" style="zoom:50%;" /> |

#### Modifications to original code base:

- I have kept the my code modifications from assignment 3. The ones listed here are in addition to those.

- bmp.cpp --> void BMPWriteImage(const Image32& img, FILE *fp)
  - Whenever I generated a ray-traced video render, the ouput gif was flipped vertically. To fix this, I made I modified this method to draw the Image32 to a bmp flipped if the Image32's _flip field was true.
- image.h
  - In order to check if an image should be flipped I added the boolean field, _flip  and its getter and setter to the Image32 class. 
- cylinder.h / cylinder.todo.cpp
  - I suppose this isn't the original code base, but it's implementation that should have been done in assignment 2. I implemented cylinder intersection since the stick figure model uses a lot of cylinders. Keep in mind that the implementation is pretty rough as its only purpose in this assignment was to make the ray-traced renders look better.
  - As such, I changed *bool implemented() { return false; }* to *bool implemented() { return true; }*. I also added a helper function, *double Cylinder::intersectBase(Ray3D ray, Point3D c) const*, to check for intersection with the two bases of the cylinder. 
- Video Recording:
  - Both Screenshot and Rendered recording is done entirely within window.cpp and window.h. Note that I only support the .bmp file format for outputting video.
  - I added the following variables to window.h:
    - static bool Record;
      - tracks if program should be recording screenshots
    - static Util::Timer RecordingTime;
      - the timer for the recording
    - static int RecordingFrames;
      - the amount of frames to record
    - static double RecordingSeconds;
      - the amount of time to record
    - static double RecordingFrameTime;
      - the time between frames in the recording
    - static int RecordingCount;
      - the amount of frames recorded
    - static std::string RecordingFile;
      - the file to output the recording
    - static bool RecordRayTraced;
      - tracks if program should be recording ray traced renders
    - static int RecordingrLimit;
      - the recursion limit for the ray traced renders
    - static double RecordingcutOff;
      - the cut off for the ray traced renders
  - Beginning the recordings is done through user input in void *Window::KeyboardFunction(unsigned char c, int x, int y)*
    - Capital G begins screenshot recording
    - Capital R begins ray traced recording
  - Capturing the video is done in *void Window::IdleFunction(void)*
    - The basic algorithm for recording a video within the IdleFunction is to pause the animation, capture a frame, resume the animation, wait for the next frame to capture, and repeat until all of the frames are captured. The difference between screenshot recording and ray traced recording is that one uses the already implemented TakeSnapshot function to capture a frames while the other ray traces the scene. After all the frames have been captured, I turn them into a gif and delete the individual frames.
    - gif.h
      - Since I would be creating a lot of videos to demonstrate output, I decided to use an external library (credit to Charlie Tangora, ctangora@gmail.com) to generate gifs from images.

#### Art Submissions:

- As I mentioned in the summary, I felt quite limited by what I could do in terms of animation since it's hard to create key files without proper software. Keeping that in mind, I decided to generate the transformations randomly using a small script. 

  `import random`

  `import math`

  `f = open("out.txt", "a")`

  `for i in range(60):`

    `f.write("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 ")`

    `for j in range(99 * 16):`

  ​    `f.write(str(random.uniform(-10, 10)))`

  ​    `f.write(" ")`

    `f.write("\n")`

  `f.close()`

  #### Video Submission

  - This is a ray traced video of ball100.ray.

  ![](kdrucia1_HTML\kdrucia1.art\kdrucia1.art.4.1.gif)

  #### Image Submission

  - This is a ray traced closeup of dog100.ray.

  ![](kdrucia1_HTML\kdrucia1.art\kdrucia1.art.4.2.bmp)

  

#### .ray and .key File Submissions

- I submit ball100.key and ball100.ray for this section.
