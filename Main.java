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
  private int[] _thresholdMap;
  private int[] _boundingBox;

  public void setup()
  {
    _kinect = new SimpleOpenNI(this);
    size(640, 480);
    _thresholdMap = new int[width * height];

    _kinect.enableDepth();
    _kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
    _kinect.enableGesture();

    _kinect.addGesture("Wave");
  }

  public void draw()
  {
    background(0);
    _kinect.update();

    if (_kinect.getNumberOfUsers() > 0) {
      int[] userMap = _kinect.getUsersPixels(SimpleOpenNI.USERS_ALL);
      int[] depthMap = _kinect.depthMap();

      float[] relativeDepthData = getRelativeDepthData(
          userMap,
          depthMap,
          width * height);

      if (_kinect.isTrackingSkeleton(1)) {
        PVector rightHand = new PVector();
        _kinect.getJointPositionSkeleton(
            1,
            SimpleOpenNI.SKEL_LEFT_HAND,
            rightHand);

        PVector leftHand = new PVector();
        _kinect.getJointPositionSkeleton(
            1,
            SimpleOpenNI.SKEL_RIGHT_HAND,
            leftHand);

        int bound = 75;

        float[] rightThreshold = {
          rightHand.z - bound,
          rightHand.z + bound,
        };

        for (int i = 0; i < userMap.length; i++) {
          if (userMap[i] != 0 &&
              rightThreshold[0] < depthMap[i] && depthMap[i] < rightThreshold[1]) {
            _thresholdMap[i] = 1;
          } else {
            _thresholdMap[i] = 0;
          }
        }

        loadPixels();
        for (int i = 0; i < pixels.length; i++) {
          if (_thresholdMap[i] == 1) {
            pixels[i] = color(0, 255 * relativeDepthData[i], 255 * (1.0f - relativeDepthData[i]));
          }
        }
        updatePixels();

        if (_boundingBox != null) {
          PVector convertedRightHand = new PVector();
          _kinect.convertRealWorldToProjective(rightHand, convertedRightHand);

          noFill();
          stroke(255, 0, 0);
          int bbWidth = _boundingBox[1] - _boundingBox[0],
              bbHeight = _boundingBox[3] - _boundingBox[2];
          rect(convertedRightHand.x - (bbWidth / 2), convertedRightHand.y - (bbHeight / 2), bbWidth, bbHeight);
        }
      }
    }
  }

  public float[] getRelativeDepthData(int[] userMap, int[] depthMap, int size)
  {
    float minimumDepth = Float.MAX_VALUE,
          maximumDepth = Float.MIN_VALUE;
    for (int i = 0; i < size; i++) {
      if (userMap[i] != 0) {
        float v = depthMap[i];
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
        float v = depthMap[i];
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
      System.out.printf("Z value: %.2f\n", joint.z);

      PVector convertedJoint = new PVector();
      _kinect.convertRealWorldToProjective(joint, convertedJoint);
      ellipse(convertedJoint.x, convertedJoint.y, 10, 10);
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

  public void onRecognizeGesture(String gesture, PVector idPosition, PVector endPosition)
  {
    System.out.printf("Recognized %s: [ID] %s [END] %s\n", gesture, idPosition, endPosition);
    _kinect.removeGesture("Wave");

    _boundingBox = new int[4];
    _boundingBox[0] = width;
    _boundingBox[1] = 0;
    _boundingBox[2] = height;
    _boundingBox[3] = 0;

    for (int i = 0; i < _thresholdMap.length; i++) {
      int row = i / width;
      int column = i % width;

      if (_thresholdMap[i] == 1) {
        if (column < _boundingBox[0]) _boundingBox[0] = column;
        if (column > _boundingBox[1]) _boundingBox[1] = column;
        if (row < _boundingBox[2]) _boundingBox[2] = row;
        if (row > _boundingBox[3]) _boundingBox[3] = row;
      }
    }
  }

  public static void main(String args[])
  {
    PApplet.main(new String[] { "Main" });
  }
}
