import java.util.Vector;
import processing.core.PApplet;
import processing.core.PImage;
import processing.core.PVector;
import SimpleOpenNI.SimpleOpenNI;

@SuppressWarnings("serial")
public class Main extends PApplet
{
  private SimpleOpenNI _kinect;
  private int _userID;

  public void setup()
  {
    _kinect = new SimpleOpenNI(this);
    size(640, 480);

    _kinect.enableDepth();
    _kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  }

  public void draw()
  {
    background(0);
    _kinect.update();

    if (_kinect.getNumberOfUsers() > 0) {
      int[] userMap = _kinect.getUsersPixels(SimpleOpenNI.USERS_ALL);
      float[] relativeDepthData = getRelativeDepthData(
          userMap,
          _kinect.depthImage().pixels,
          width * height);

      loadPixels();
      for (int i = 0; i < width * height; i++) {
        if (userMap[i] != 0) {
          pixels[i] = color(
              0,
              255 * relativeDepthData[i],
              255 * (1.0f - relativeDepthData[i]));
        }
      }
      updatePixels();

      if (_kinect.isTrackingSkeleton(1)) {
        stroke(0);
        strokeWeight(5);

        _kinect.drawLimb(
            1,
            SimpleOpenNI.SKEL_LEFT_ELBOW,
            SimpleOpenNI.SKEL_LEFT_HAND);

        noStroke();
        fill(255, 0, 0);

        drawJoint(1, SimpleOpenNI.SKEL_LEFT_ELBOW);
        drawJoint(1, SimpleOpenNI.SKEL_LEFT_HAND);
      }
    }
  }

  public float[] getRelativeDepthData(int[] userMap, int[] depthMap, int size)
  {
    float minimumDepth = Float.MAX_VALUE,
          maximumDepth = Float.MIN_VALUE;
    for (int i = 0; i < size; i++) {
      if (userMap[i] != 0) {
        float v = depthMap[i] & 0xFF;
        if (v < minimumDepth) {
          minimumDepth = v;
        }
        if(v > maximumDepth) {
          maximumDepth = v;
        }
      }
    }

    float[] data = new float[size];

    for (int i = 0; i < size; i++) {
      if (userMap[i] != 0) {
        float v = depthMap[i] & 0xFF;
        data[i] = (v - minimumDepth) / (maximumDepth - minimumDepth);
      } else {
        data[i] = 0.0f;
      }
    }

    return data;
  }

  public void drawJoint(int uid, int jid)
  {
    PVector joint = new PVector();
    float confidence = _kinect.getJointPositionSkeleton(uid, jid, joint);
    if (confidence > 0.5) {
      PVector convertedJoint = new PVector();
      _kinect.convertRealWorldToProjective(joint, convertedJoint);
      ellipse(convertedJoint.x, convertedJoint.y, 5, 5);
    }
  }

  public void onNewUser(int uid)
  {
    System.out.println("Starting pose detection...");
    _kinect.startPoseDetection("Psi", uid);
  }

  public void onEndCalibration(int uid, boolean successful)
  {
    if (successful) {
      System.out.println("User calibrated! " + uid);
      _kinect.startTrackingSkeleton(uid);
    } else {
      System.out.println("Failed to calibrate user!");
      _kinect.startPoseDetection("Psi", uid);
    }
  }

  public void onStartPose(String pose, int uid)
  {
    System.out.println("Started pose for user");
    _kinect.stopPoseDetection(uid);
    _kinect.requestCalibrationSkeleton(uid, true);
  }

  public static void main(String args[])
  {
    PApplet.main(new String[] { "Main" });
  }
}
