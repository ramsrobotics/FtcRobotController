package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Implements the common algorithms used both by BoKAutoBlue* and BoKAutoRed*.
 * Its primary responsibilities include:
 * initSoftware() method which
 * 1. initialize OpenCV
 * 2. initialize Vuforia
 * moveForward() method
 * <p>
 * <p>
 * Flow of Auto: Goal: Reach soft cap auto points
 * detect block 1 sec
 * drive forward 2 sec
 * align and grab 4 sec
 * back up 2 sec
 * turn towards the foundation 1 sec
 * drive against the wall 4 sec
 * drive by dist
 * align to foundation 4 sec
 * place 2 sec
 * pull with intake 3 sec
 * turn 1 sec
 * park 1 sec
 */
public abstract class CCAutoCommon implements CCAuto {
    protected static final boolean DEBUG_OPEN_CV = false;
    // CONSTANTS
    private static final double DT_RAMP_SPEED_INIT = 0.2;
    private static final double P_TURN_COEFF = 0.5;
    // Sampling locations in the image; phone's image is 1280x720s
    private static final int HOUGH_CIRCLE_MIN_RAD = 20;
    private static final int HOUGH_CIRCLE_MAX_RAD = 105;
    private static final int SPHERE_LOC_Y_MIN = 275;
    private static final int SPHERE_LOC_Y_MAX = 600;
    private static final int CUBE_LOC_LEFT_X_MIN = 350;
    private static final int CUBE_LOC_RIGHT_X_MAX = 850;
    private static final int ROI_WIDTH = 50;
    private static final int ROI_HEIGHT = 50;
    private static final String VUFORIA_CUBE_IMG = "vuImage.png";
    private static final String VUFORIA_ROI_IMG = "vuImageROI.png";
    private static final double SAMPLE_RATE_SEC = 0.05;
    private static final int RS_DIFF_THRESHOLD_CM = 5;
    private static final int RS_DIFF_THRESHOLD_CM_LOW = 1;
    private static final double DETECT_BUMP_THRESHOLD = 1.5;
    // Constants for runAuto
    private static final double MOVE_POWER_LOW = 0.3;
    private static final double MOVE_POWER_HIGH = 0.5;
    private static final boolean PARK = true;
    private static final boolean MOVE_FOUNDATION = true;
    private static final boolean TWO_STONES = false;
    private static final boolean WAIT = false;
    private static final int WAIT_SECONDS = 0;
    protected CCAuto.BoKAllianceColor allianceColor;
    protected ElapsedTime runTime = new ElapsedTime();
    protected CCAutoOpMode opMode;  // save a copy of the current opMode and robot
    protected CCHardwareBot robot;
    protected Orientation angles;
    private int YELLOW_PERCENT = 95;
    private AppUtil appUtil = AppUtil.getInstance();
    private VuforiaLocalizer vuforiaFTC;
    protected boolean noStone = true;
    boolean park = true;
    // OpenCV Manager callback when we connect to the OpenCV manager
    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    // All BoKAuto*OpModes must be derived from CCAutoCommon. They must override runSoftware
    // method in order to run specific methods from CCAutoCommon based on the missions.

    /**
     * writeFile
     * Helper method to save a copy of the image seen by the robot. This image can be opened in
     * Paint for further analysis as the image is saved in PNG format.
     *
     * @param fname
     * @param img
     * @param always
     */
    protected static void writeFile(String fname, Mat img, boolean always) {
        if (always || DEBUG_OPEN_CV) {
            String filePath = "/sdcard/FIRST/" + fname;
            //Log.v("BOK", "Saving image" + filePath);
            Imgcodecs.imwrite(filePath, img);
        }
    }

    @Override
    public void runSoftware() {
        runAuto(true, true, false);
    }

    /**
     * initSoftware
     * Initialize OpenCV, Vuforia. Setup the frame queue in Vuforia to 1 so that we can
     * grab a frame from Vuforia and run OpenCV algorithm on the frame.
     * Wait for the drive team to press X on Gamepad1 before initializing IMU.
     * Parameters:
     * 1. opMode: caller's opMode
     * 2. robot: hardware used by the opMode
     * Returns BOK_AUTO_SUCCESS
     */

    //@Override
    public CCAuto.BoKAutoStatus initSoftware(CCAutoOpMode opMode,
                                             CCHardwareBot robot) {
        loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            Log.v("BOK", "initDebug False");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION,
                    appUtil.getActivity(), loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }


        Log.v("BOK", "Initializing Vuforia");
        // Initialize Vuforia
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the
         * RC phone); If no camera monitor is desired, use the parameterless constructor instead.
         */
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Vuforia License Key
        parameters.vuforiaLicenseKey = "ARRy24H/////AAABmWUQmKdoZknqg/9YdrBoyz1A6PA84DX24hMeuG/wA60YzwbhQoJHjFvdO0dHMALr1N9D3tIFlQNREybNz8TlycHSnb5bFqTlt3iBHwlgjz0ZRKwXVVIeX531cNltqKzaja0/WjfjU5baqWN2TdrivXUqwwd/+mjTyo2v/70pHQZ+mUuNO6Lnbw1xdRU1t8Jjf1zYZ9zp3eWAXl7ozKcI8VkBqnzhuU3EMOULrvQYC99dYp6G684cQc7jbXcvimQ2kBUdghB6IzmavVCUBDn4FUE99WzH7HxiW4wWRnSxxkcmDP32PeEtSlggRZDTIk1pKlFtMUo4739NQMc3ANaapZHhGWJYhV1KUVtkNjhB2S15";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaFTC = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        Log.v("BOK", "Done initializing software");
        this.opMode = opMode;
        this.robot = robot;

        while (!opMode.gamepad1.x) {
            opMode.telemetry.addData("Status", "Press \"X\" to start gyro init");
            opMode.telemetry.update();
        }

        opMode.telemetry.addData("Status", "Initializing gyro");
        opMode.telemetry.update();
        setupRobot();
        opMode.telemetry.addData("Status", "Done initializing gyro!");
        opMode.telemetry.update();
        return CCAuto.BoKAutoStatus.BOK_AUTO_SUCCESS;


    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    /**
     * setupRobot
     * Helper method called by initSoftware to complete any software setup tasks.
     */
    private void setupRobot() {
        // now initialize the IMU
        robot.initializeImu();
    }

    /**
     * setupOpenCVImg
     * Helper  method to setup OpenCV mat object from a rgb image in Vuforia.
     *
     * @param rgb
     * @param fileName
     * @param always
     * @return
     */
    private Mat setupOpenCVImg(Image rgb, String fileName, boolean always) {
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(),
                rgb.getHeight(),
                Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        Mat img = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, img);

        // OpenCV only deals with BGR
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        writeFile(fileName, img, always);

        // First convert from BGR to HSV; separate the color components from intensity.
        // Increase robustness to lighting changes.
        // Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        return img;
    }

    /**
     * move
     * Algorithm to move forward or back using encoder sensor on the DC motors on the drive train
     * Parameters
     * leftPower, rightPower, inches to move, forward or back, waitForSec before timing out
     *
     * @param leftPower
     * @param rightPower
     * @param inches
     * @param forward
     * @param waitForSec
     */
    protected void move(double leftPower,
                        double rightPower,
                        double inches,
                        boolean forward,
                        double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.resetDTEncoders();
            robot.startMove(leftPower, rightPower, inches, forward);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&*/
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "move timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }
            }

            robot.stopMove();
        }
    }


    /**
     * moveRamp
     * Algorithm to move forward or back using encoder sensor on the DC motors on the drive train
     * using a ramp up and ramp down period to prevent the robot from getting jerks.
     * Parameters
     * maxPower, inches to move, forward or back, waitForSec before timing out
     *
     * @param maxPower
     * @param inches
     * @param forward
     * @param waitForSec
     */
    protected void moveRamp(double maxPower,
                            double inches,
                            boolean forward,
                            double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            boolean steady = false;
            robot.resetDTEncoders();
            int targetEncCount = robot.startMove(DT_RAMP_SPEED_INIT,
                    DT_RAMP_SPEED_INIT,
                    inches,
                    forward);
            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount / 4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT) / rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "moveRamp timed out!" +
                            String.format(" %.1f", waitForSec));
                    break;
                }
                int lfEncCount = Math.abs(robot.getLFEncCount());
                if (lfEncCount < rampupEncCount) {
                    double power = DT_RAMP_SPEED_INIT + ratePower * lfEncCount;
                    robot.setPowerToDTMotors(power, forward);

                } else if (lfEncCount < rampdnEncCount) {
                    if (!steady) {
                        robot.setPowerToDTMotors(maxPower, forward);
                        steady = true;
                    }
                } else {
                    double power = DT_RAMP_SPEED_INIT -
                            ratePower * (lfEncCount - targetEncCount);
                    robot.setPowerToDTMotors(power, forward);
                }
            }
            // finally stop moving
            robot.stopMove();
        }
    }

    /**
     * strafe
     * Allows robot to move left and right by amount of motor rotations
     *
     * @param maxPower
     * @param rotations
     * @param right
     * @param waitForSec
     */
    protected void strafe(double maxPower,
                          double rotations,
                          boolean right,
                          double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            int targetEncCount = -1;
            try {
                targetEncCount = robot.startStrafeWEnc(maxPower, rotations, right);
            } catch (UnsupportedOperationException e) {
                return;
            }
            Log.v("BOK", "strafeRamp: " + targetEncCount);

            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount / 4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT) / rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    (robot.getAvgEncCount() < targetEncCount)) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "strafeRamp timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }


            }
            robot.stopMove();
        }
    }

    /**
     * isCubePresent
     * Helper method that checks for yellow pixels to confirm the presence of the cube mineral. The
     * input image is in HSV format to prevent the effect of ambient light conditions.
     * This method should always return false because this is called by findCube only for spheres.
     *
     * @param imgHSV
     * @param roi
     * @return
     */
    private boolean areRingsThere(Mat imgHSV, Rect roi) {
        Mat hist = new Mat();
        MatOfInt histSize = new MatOfInt(180);
        MatOfFloat ranges = new MatOfFloat(0f, 180f);
        Mat mask = new Mat(imgHSV.rows(), imgHSV.cols(),
                CvType.CV_8UC1, new Scalar(0));
        float[] resFloat = new float[180];
        boolean foundYellow = false;

        Imgproc.rectangle(imgHSV, new Point(roi.x, roi.y),
                new Point(roi.x + roi.width,
                        roi.y + roi.height),
                new Scalar(0, 255, 0), 10);

        Mat subMask;
        try {
            subMask = mask.submat(roi);
        } catch (CvException cvE) {
            Log.v("BOK", "Caught CvException " + cvE.toString());
            try {
                Rect newRoi = new Rect(roi.x, roi.y, roi.width / 2, roi.height / 2);
                roi = newRoi;
                subMask = mask.submat(roi);
            } catch (CvException e) {
                Log.v("BOK", "Caught another CvException!" + cvE.toString());
                return false;
            }
        }
        subMask.setTo(new Scalar(255));

        Imgproc.calcHist(Arrays.asList(imgHSV), new MatOfInt(0),
                mask, hist, histSize, ranges);
        //writeFile(HSV_IMG, img, DEBUG_OPEN_CV);
        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
        hist.get(0, 0, resFloat);

        int p, nYellowPixels = 0;
        int numPixels = roi.width * roi.height;
        // Red is 0 (in HSV),
        // but we need to check between 10 and 35
        for (p = 0; p < 15; p++) {
            nYellowPixels += (int) resFloat[p];
        }

        if (Math.abs(nYellowPixels) >= Math.abs(((numPixels * YELLOW_PERCENT) / 100))) {
            foundYellow = true;
            Log.v("BOK", "Yellow is true");
        }

        Log.v("BOK", "num Yellow pixels: " + nYellowPixels + " out of " + numPixels);
        Log.v("BOK", "nYellow: " + Math.abs(nYellowPixels) + " Percent: " + Math.abs(((numPixels * YELLOW_PERCENT) / 100)));
        hist.release();
        histSize.release();
        ranges.release();
        mask.release();

        return foundYellow;
    }

    /**
     * findCube
     * Finds the location of the cube (sampling mission).
     * It takes a frame from Vuforia frame queue. It converts the frame from rgb to gray. It uses
     * the HoughCircles algorithm to detect the spheres in the band of the picture where the
     * spheres are most likely to be found.
     *
     * @return
     */

    protected CCAutoRingsLocation findRings() {
        int numYellow = 0;
        CCAutoRingsLocation ret = CCAutoRingsLocation.CC_RING_FRONT;
        VuforiaLocalizer.CloseableFrame frame;

        // takes the frame at the head of the queue
        try {
            frame = vuforiaFTC.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.v("BOK", "Exception while finding cube!!");
            return ret;
        }

        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                // rgb is now the Image object that we’ve used in the video
                if (rgb != null) {
                    Mat src = setupOpenCVImg(rgb, VUFORIA_CUBE_IMG, true);
                    Mat srcHSV = new Mat();

                    Mat srcGray = new Mat(); // Convert image to gray scale
                    Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
                    Imgproc.cvtColor(src, srcHSV, Imgproc.COLOR_BGR2HSV);
                    Size s = new Size(50, 50);

                    if (allianceColor == BoKAllianceColor.BOK_ALLIANCE_RED) {


                        Rect TopRoi = new Rect(new Point(530, 450), new Point(510, 400));
                        Rect BotRoi = new Rect(new Point(630, 450), new Point(650, 400));
                        boolean top = areRingsThere(srcHSV, TopRoi);
                        boolean bot = areRingsThere(srcHSV, BotRoi);

                        if (top && bot) {
                            ret = CCAutoRingsLocation.CC_RING_BACK;
                        }
                        if (!top && bot) {
                            ret = CCAutoRingsLocation.CC_RING_MID;
                        }
                        if (!top && !bot) {
                            ret = CCAutoRingsLocation.CC_RING_FRONT;
                        }

                        /*
                        if (center && left) {
                            ret = CCAutoStoneLocation.CC_CUBE_RIGHT;
                        }
                        if (!center && left) {
                            ret = CCAutoStoneLocation.CC_CUBE_CENTER;
                        }
                        if (!left && center) {
                            ret = CCAutoStoneLocation.CC_CUBE_LEFT;
                        }
                        */
                        writeFile(VUFORIA_ROI_IMG, srcHSV, true);
                        Log.v("BOK", "Top: " + top +
                                " Bot: " + bot);
                    }


                }

            }
        }
        frame.close();
        Log.v("BOK", "Cube loc:" + ret.toString());
        return ret;
    }

    /**
     * takePicture
     * Helper method to take picture from Vuforia and save it to a file.
     *
     * @param sFileName
     */
    protected void takePicture(String sFileName) {
        VuforiaLocalizer.CloseableFrame frame;
        // takes the frame at the head of the queue
        try {
            frame = vuforiaFTC.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.v("BOK", "Exception while taking picture!!");
            return;
        }
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                // rgb is now the Image object that we’ve used in the video
                if (rgb != null) {
                    Mat img = setupOpenCVImg(rgb, sFileName, true);
                    img.release();
                }
                break;
            } // PIXEL_FORMAT.RGB565
        }
        frame.close();
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */

    public double gyroTurn(double speed,
                           double init_angle,
                           double angle,
                           int threshold,
                           boolean tank,
                           boolean leftTank,
                           double waitForSeconds) {
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() &&
                !onHeading(speed, init_angle, angle, threshold, tank, leftTank, P_TURN_COEFF)) {
            if (runTime.seconds() >= waitForSeconds) {
                Log.v("BOK", "gyroTurn timed out!" + String.format(" %.1f", waitForSeconds));
                break;
            }
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(CCHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        robot.setPowerToDTMotors(0);
        Log.v("BOK", "turnF: " + angles.thirdAngle);
        return angles.thirdAngle;
    }

    // Code copied from the sample PushbotAutoDriveByGyro_Linear

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    protected boolean onHeading(double speed,
                                double init_angle,
                                double angle,
                                int threshold,
                                boolean tank,
                                boolean leftTank,
                                double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= (Math.abs(angle - init_angle) * 0.25)) {
            // slow down as we are withing 1/4th of the target
            if (speed > DT_TURN_SPEED_LOW) {
                speed = DT_TURN_SPEED_LOW;
            }
            //speed /= 2;
        }

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);

            rightSpeed = speed * steer;
            if (rightSpeed > 0) {
                rightSpeed = Range.clip(rightSpeed,
                        DT_TURN_SPEED_LOW,
                        DT_TURN_SPEED_HIGH);
            } else {
                rightSpeed = Range.clip(rightSpeed,
                        -DT_TURN_SPEED_HIGH,
                        -DT_TURN_SPEED_LOW);
            }

            if (!tank) {
                leftSpeed = rightSpeed;
            } else if (leftTank) {
                leftSpeed = rightSpeed;
                rightSpeed = 0;
            } else {
                leftSpeed = 0;
            }
        }

        // Send desired speeds to motors.
        robot.setOnHeading(leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of
     * reference; +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        robotError = targetAngle - angles.thirdAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    protected double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * followHeadingPID
     * Parameters:
     * heading: gyro heading (z axis)
     * power: input power (steady state)
     * dist: distance to move in inches
     * detectBump: true if the robot should stop after detecting bump
     * waitForSec: seconds before timing out
     *
     * @param heading
     * @param power
     * @param dist
     * @param detectBump
     * @param waitForSec
     */
    protected void followHeadingPID(double heading,
                                    double power,
                                    double dist,
                                    boolean detectBump,
                                    double waitForSec, boolean forward) {
        double angle, error, diffError, turn, speedL, speedR,
                sumError = 0, lastError = 0, lastTime = 0;
        double Kp = 0.01, Ki = 0, Kd = 0; // Ki = 0.165; Kd = 0.093;
        double targetEnc = robot.getTargetEncCount(dist); // convert inches to target enc count
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        Log.v("BOK", "followHeadingPID: " + heading + ", dist: " + dist);
        robot.resetDTEncoders();
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec) &&
                (robot.getAvgEncCount() < targetEnc)) {
            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES);
                angle = angles.thirdAngle;
                error = angle - heading;
                Log.v("BOK", "angle: " + angle + " error: " + error);
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                Log.v("BOK", "angle: " + angle + " error: " + error + " turn: " + turn);


                if (forward) {
                    speedL = (power) + turn;
                    speedR = (power) - turn;
                    speedR *= -1;
                    speedL = Range.clip(speedL, -Math.abs(power), Math.abs(power));
                    speedR = Range.clip(speedR, -Math.abs(power), Math.abs(power));
                } else {
                    speedL = (power) - turn;
                    speedR = (power) + turn;
                    speedL *= -1;
                    speedR = Range.clip(speedR, -Math.abs(power), Math.abs(power));
                    speedL = Range.clip(speedL, -Math.abs(power), Math.abs(power));
                }


                speedL = Range.clip(speedL, -0.9, 0.9);
                speedR = Range.clip(speedR, -0.9, 0.9);
                robot.setPowerToDTMotors(speedL, speedR);
                Log.v("BOK", "Follow Heading PID Speed Left: " + speedL + " Speed Right: " + speedR);
                lastError = error;
                lastTime = currTime;
                if (detectBump) {

                    if (Math.abs(angles.secondAngle) > DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta y in if " + angles.secondAngle);
                        break;
                    }
                }
            }
        }
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "followHeadingPID timed out!");
        }

    }

    /**
     * moveWithRangeSensor
     * Parameters:
     * power: input power (steady state)
     * targetDistanceCm: distance to the wall before stopping the robot
     * capDistCm: max distance that can possibly be reported by the distance sensor
     * waitForSec: seconds before timing out
     *
     * @param power
     * @param targetDistanceCm
     * @param capDistCm
     * @param waitForSec
     */
    public void moveWithRangeSensor(double power,
                                    int targetDistanceCm,
                                    int capDistCm,
                                    double waitForSec) {
        double cmCurrent, diffFromTarget, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);
        Log.v("BOK", "moveWithRangeSensor: " + cmCurrent + ", target: " + targetDistanceCm);
        //if (!Double.isNaN(cmCurrent))
        diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget / 15;

            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor

            if (diffFromTarget < 0) {// we are still far away!
                robot.setPowerToDTMotors(Math.abs(wheelPower), false /* going back*/);
            } else {
                // if diffFromTarget > 0 then wheelPower is +ve
                // robot.setPowerToDTMotors(wheelPower, true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensor timed out!");
        }
        Log.v("BOK", "moveWithRangeSensor: " + cmCurrent);
    }

    /**
     * moveWithRangeSensorBack
     * Parameters:
     * power: input power (steady state)
     * targetDistanceCm: distance to the wall before stopping the robot
     * capDistCm: max distance that can possibly be reported by the distance sensor
     * waitForSec: seconds before timing out
     *
     * @param power
     * @param targetDistanceCm
     * @param capDistCm
     * @param waitForSec
     * @param rangeSensor
     * @param forward
     */
    public void moveWithRangeSensorBack(double power,
                                        double targetDistanceCm,
                                        int capDistCm,
                                        double waitForSec, AnalogInput rangeSensor, boolean forward, int threshold) {
        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //   AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);//0.25
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent + ", target: " + targetDistanceCm);

        //if (!Double.isNaN(cmCurrent))
        if (!forward) {
            diffFromTarget = targetDistanceCm - cmCurrent;
        } else {
            diffFromTarget = cmCurrent - targetDistanceCm;
        }
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= threshold)) {
            Log.v("BOK", "CmCurrent: " + cmCurrent);
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRSBack timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget / 15;
            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor
            if (diffFromTarget < 0 || forward) { // we are still far away!
                robot.setPowerToDTMotors(Math.abs(wheelPower), false /* going back*/);
            } else if (!forward) {
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(Math.abs(wheelPower), true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensorBack timed out!");
        }
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent);
    }


    /**
     * dumpMarker
     * Dumps the marker by moving the marker servo.
     */
    protected void dumpMarker() {
        //  robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL);
        opMode.sleep(500);
        //robot.markerServo.setPosition(robot.MARKER_SERVO_INIT);
    }

    protected void arcTurn(double leadingPower, double laggingPower, double radius, double initAngle, double targetAngle, double waitForSec) {
        runTime.reset();

        boolean isLeftLead = ((targetAngle - initAngle) < 0) && (leadingPower > 0);
        if (isLeftLead) {
            robot.setPowerToDTMotors(leadingPower, laggingPower);
        } else {
            robot.setPowerToDTMotors(laggingPower, leadingPower);
        }
        while (Math.abs(robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle) < Math.abs(targetAngle) && runTime.seconds() < waitForSec && opMode.opModeIsActive()) {
            Log.v("BOK", "IMU: " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES).thirdAngle);
        }
        robot.stopMove();
    }

    protected void strafeWithRange(double power, int targetDistance, int capDistance, AnalogInput rangeSensor, int waitForSec, int threshold, boolean right) {
        double cmCurrent, diffFromTarget = targetDistance, pCoeff, wheelPower;
        boolean Right = right;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        //   AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistance, 0.25, opMode);//0.25
        while (cmCurrent < 20 && opMode.opModeIsActive() && waitForSec < runTime.seconds()) {
            cmCurrent = robot.getDistanceCM(rangeSensor, capDistance, 2, opMode);//0.25

        }
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent + ", target: " + targetDistance);

        //if (!Double.isNaN(cmCurrent))
        if (targetDistance > cmCurrent) {
            diffFromTarget = targetDistance - cmCurrent;
            Right = !right;
        } else {
            diffFromTarget = cmCurrent - targetDistance;

        }

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= threshold)) {
            Log.v("BOK", "CmCurrent: " + cmCurrent);
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRSBack timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistance, 0.25, opMode);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistance - cmCurrent;
            pCoeff = diffFromTarget / 15;
            wheelPower = Range.clip(power * pCoeff, 0, Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.7)
                wheelPower = 0.7; // min power to move
            if (wheelPower < 0 || wheelPower > -0.7)
                wheelPower = 0.7;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor
            robot.setPowerToDTMotorsStrafe(wheelPower, Right);
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensorBack timed out!");
        }
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent);

    }
    protected void shootRings(int numRings, int waitForSec){
        int ringCount = 0;
        runTime.reset();
        while(opMode.opModeIsActive() && ringCount < numRings && runTime.seconds() < waitForSec){
           if(ringCount == 1){
               robot.frontIntake.setPower(-1);
           }
          // robot.gateServo.setPosition(robot.GATE_DOWN);
            if(ringCount == 0) {
                robot.shooterServo.setPosition(robot.getShooterAngle(robot.getBatteryVoltage()) + 0.028);
            }
            else {
                robot.shooterServo.setPosition(robot.getShooterAngle(robot.getBatteryVoltage()));
            }
           robot.gateServo.setPosition(robot.GATE_UP);
           robot.verticalIntake.setPower(-1);
           opMode.sleep(750);
           ringCount++;
        //   robot.shooter.setPower(.95);
        }
    }
    protected void followPath(int numPoints, Point[] points, double lastx, double lasty, double lasttheta, double turnSpeed, double straightsSpeed, int waitForSec){
        runTime.reset();
        double currentX = lastx;
        double currentY = lasty;

        double lastX = lastx;
        double lastY = lasty;

        double lastTheta = lasttheta;
        double theta = lastTheta;
       for(int i = 0; i <= numPoints;i++){
           if(runTime.seconds() >= waitForSec){
               break;
           }
           if(i != 0){
               lastX = currentX;
               lastY = currentY;
           }

           currentX = currentX + (robot.imu.getLinearAcceleration().xAccel*(runTime.seconds()*runTime.seconds()))/2;
           currentY = currentY + (robot.imu.getLinearAcceleration().yAccel*(runTime.seconds()*runTime.seconds()))/2;
           double distance = Math.sqrt(Math.pow(points[i].x - currentX, 2) + Math.pow(points[i].y - currentY, 2));
           if(i != 0){
               lastTheta = theta;
           }
           theta = Math.asin(currentX/distance);

           if(points[i].x < currentX){
               theta = -theta;
           }
          //might have to replace with new functions to work with this function
           //remove runtime functionality
           //remove the stopping of motors, requires debug and test
           gyroTurn(turnSpeed, lastTheta, theta, 1, false, false, 2);
           followHeadingPID(theta, straightsSpeed, distance, false, 3, points[i].y < currentY);

       }
    }
    /**
     * runAuto
     * Helper method called from CCAutoRedStoneInsideOpMode or CCAutoRedStoneOutsideMode
     *
     * @param inside
     * @param startStone
     */

    /*
     *Blue Stone Inside
     * Route Close to Mid
     *Blue Stone Outside
     * Route Away from Mid
     */
    protected void runAuto(boolean inside, boolean startStone, boolean park) {
        CCAutoRingsLocation loc = CCAutoRingsLocation.CC_RING_UNKNOWN;
        Log.v("BOK", "Angle at runAuto start " +
                robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle);

        // Step 1: find skystone location
        try {
            loc = findRings();
        } catch (java.lang.Exception e) {
            Log.v("BOK", "Exception at findCube()");
        }

        Log.v("BOK", "StoneLoc: " + loc);

        Log.v("BOK", "Color: " + allianceColor + " Inside Route: " +
                inside + " Stating At Stone: " + startStone);
        followHeadingPID(0, 0.2, 2, false, 3, true);
        robot.shooter.setPower(0.95);
        robot.shooterServo.setPosition(robot.getShooterAngle(robot.getBatteryVoltage()));
       // followHeadingPID(0, 0.5, 10, false, 2, true);
        gyroTurn(0.3, 0, -20, 1, false, false, 3);
        followHeadingPID(-20, 0.5, 15, false, 2, true);
        gyroTurn(0.3, -20, 25, 1, false, false, 3);
        followHeadingPID(25, 0.5, 20, false, 4, true);
        gyroTurn(0.25, 25, 90, 1, false, false, 3);
        followHeadingPID(90, 0.3, 3, false, 2, true);
        shootRings(3, 7);
        robot.verticalIntake.setPower(0);
        robot.shooter.setPower(0);
        robot.frontIntake.setPower(0);
        robot.gateServo.setPosition(robot.GATE_DOWN);
        if (loc == CCAutoRingsLocation.CC_RING_BACK) {
            gyroTurn(0.3, 90, 170, 1, false, false, 3);
            followHeadingPID(170, 0.5, 37, false, 3, false);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.2, 170, 160, 1, false, false, 3);
            followHeadingPID(160, 0.5, 49, false, 6, true);
            gyroTurn(0.3, 160, 30, 1, false, false, 3);

            //   robot.frontIntake.setPower(-1);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(30, 0.2, 8, false, 3, false);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-150);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(30, 0.3, 5, false, 3, true);
            gyroTurn(0.4, 30, 160, 2, false, false, 3);
            followHeadingPID(160, 0.6, 55, false, 5, false);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(160, 0.6, 15, false, 5, true);
        }
        if(loc == CCAutoRingsLocation.CC_RING_MID){
            gyroTurn(0.3, 90, 170, 1, false, false, 3);
            followHeadingPID(170, 0.4, 15, false, 3, false);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(750);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.2, 170, 150, 1, false, false, 3);
            followHeadingPID(150, 0.5, 30, false, 5, true);
            gyroTurn(0.35, 150, 50, 1, false, false, 3);

            //   robot.frontIntake.setPower(-1);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-350);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(50, 0.2, 10, false, 3, false);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-150);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(50, 0.3, 8, false, 3, true);
            gyroTurn(0.4, 50, 178, 2, false, false, 3);
            followHeadingPID(178, 0.6, 30, false, 5, false);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(750);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(178, 0.3, 2, false, 5, true);
        }
        if(loc == CCAutoRingsLocation.CC_RING_FRONT){
             gyroTurn(0.3, 90, 120, 1, false, false, 3);
            followHeadingPID(110, 0.3, 13, false, 3, false);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.2, 120, 110, 1, false, false, 3);
            followHeadingPID(110, 0.5, 32, false, 5, true);
            gyroTurn(0.3, 110, 45, 1, false, false, 3);

            //   robot.frontIntake.setPower(-1);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-350);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(45, 0.3, 20, false, 3, false);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-150);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(45, 0.3, 15, false, 3, true);
            gyroTurn(0.4, 45, 130, 2, false, false, 3);
            followHeadingPID(130, 0.6, 25, false, 5, false);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(750);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(130, 0.3, 5, false, 5, true);
            gyroTurn(0.4, 130, 180, 1, false, false, 3);
            move(0.3, 0.3, 5, false, 3);
        }

        //robot.frontIntake.setPower(1);
     //   gyroTurn(0.3, 0, 20, 1, false, false, 3);
      /*  robot.frontIntake.setPower(-1);
        move(0.2, 0.2, 15, true, 3);
        move(0.2, 0.2, 4, false, 3);
        robot.frontIntake.setPower(-1);
        robot.verticalIntake.setPower(-1);
        move(0.05, 0.05, 15, true, 5);
        opMode.sleep(2000);
        robot.frontIntake.setPower(0);
        robot.verticalIntake.setPower(0);

       */



        /*
        robot.shooterServo.setPosition(robot.SHOOTER_POWER_SHOT_ANGLE + 0.008);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter.setPower(-0.95);
        followHeadingPID(0, 0.65, 43, false, 3, true);
        opMode.sleep(300);
        gyroTurn(0.1, 0, 2, 1, false, false, 3);
        opMode.sleep(300);
        robot.gateServo.setPosition(robot.FLICKER_IN);
        opMode.sleep(400);
        robot.gateServo.setPosition(robot.FLICKER_OUT);
        opMode.sleep(300);
        gyroTurn(0.1, 2, 9, 1, false, false, 3);
        opMode.sleep(300);
        robot.shooterServo.setPosition(robot.SHOOTER_POWER_SHOT_ANGLE);
        opMode.sleep(300);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter.setPower(-0.95);
        opMode.sleep(300);
        robot.gateServo.setPosition(robot.FLICKER_IN);
        opMode.sleep(400);
        robot.gateServo.setPosition(robot.FLICKER_OUT);
        opMode.sleep(300);
        gyroTurn(0.1, 9, 15, 1, false, false, 3);
        opMode.sleep(300);
        robot.shooterServo.setPosition(robot.SHOOTER_POWER_SHOT_ANGLE);
        opMode.sleep(300);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter.setPower(-0.95);
        opMode.sleep(300);
        robot.gateServo.setPosition(robot.FLICKER_IN);
        opMode.sleep(400);
        robot.gateServo.setPosition(robot.FLICKER_OUT);
        opMode.sleep(300);

        robot.shooter.setPower(0);
        //  robot.boxServo.setPosition(robot.BOX_DOWN);

        if (loc == CCAutoRingsLocation.CC_RING_BACK) {
            gyroTurn(0.3, 15, 160, 1, false, false, 4);
            followHeadingPID(160, 0.6, 42, false, 4, false);
            //  gyroTurn(0.3, 160, 160, 1, false, false, 3);

            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(160, 0.5, 10, false, 4, true);
            gyroTurn(0.3, 160, 30, 1, false, false, 3);
            followHeadingPID(30, 0.45, 47, false, 3, false);
            gyroTurn(0.3, 30, 0, 1, false, false, 3);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE + 0.1);
            followHeadingPID(0, 0.2, 5, false, 3, false);
            // moveWithRangeSensorBack(0.2, 17, 25, 3, robot.distanceBack, false, 1);
            // gyroTurn(0.15, 0, -20, 1, false, false, 3);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);

            followHeadingPID(0, 0.8, 50, false, 4, true);
            gyroTurn(0.5, 0, -175, 3, false, false, 3);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            move(0.5, 0.5, 15, true, 3);

            //  strafe(0.8, 4, true, 5);

        }
        if (loc == CCAutoRingsLocation.CC_RING_MID) {
            gyroTurn(0.3, 12, 160, 1, false, false, 3);
            followHeadingPID(160, 0.6, 20, false, 4, false);
            //  gyroTurn(0.3, 160, 160, 1, false, false, 3);

            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(160, 0.45, 5, false, 4, true);
            gyroTurn(0.3, 160, 40, 1, false, false, 3);
            followHeadingPID(40, 0.45, 43, false, 3, false);
            gyroTurn(0.3, 40, 0, 1, false, false, 3);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            // robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE + 0.1);
            // followHeadingPID(0, 0.2, 15, false, 3, false);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE + 0.1);
            followHeadingPID(0, 0.2, 6, false, 3, false);
            // moveWithRangeSensorBack(0.2, 17, 25, 3, robot.distanceBack, false, 1);
            // gyroTurn(0.15, 0, -20, 1, false, false, 3);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            gyroTurn(0.2, 0, 30, 1, false, false, 3);
            followHeadingPID(30, 0.8, 35, false, 4, true);
            gyroTurn(0.5, 30, -175, 3, false, false, 3);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            move(0.5, 0.5, 5, true, 3);
        }
        if (loc == CCAutoRingsLocation.CC_RING_FRONT) {
            gyroTurn(0.3, 12, 120, 1, false, false, 3);
            followHeadingPID(120, 0.6, 10, false, 4, false);
            //  gyroTurn(0.3, 160, 160, 1, false, false, 3);

            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(120, 0.45, 5, false, 4, true);
            gyroTurn(0.3, 120, 55, 1, false, false, 3);
            followHeadingPID(55, 0.45, 30, false, 3, false);
            gyroTurn(0.3, 55, 0, 1, false, false, 3);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            // robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE + 0.1);
            // followHeadingPID(0, 0.2, 15, false, 3, false);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE + 0.1);
            followHeadingPID(0, 0.2, 5, false, 3, false);
            // moveWithRangeSensorBack(0.2, 17, 25, 3, robot.distanceBack, false, 1);
            // gyroTurn(0.15, 0, -20, 1, false, false, 3);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            // gyroTurn(0.2, 0, 30, 1, false, false, 3);
            followHeadingPID(0, 0.6, 9, false, 4, true);
            gyroTurn(0.5, 0, -175, 3, false, false, 3);
            // followHeadingPID(-175, 0.6, 10, false, 4, false);

            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            // move(0.15, 0.15, 10, false, 3);
            followHeadingPID(-175, 0.3, 10, false, 3, true);
            gyroTurn(0.3, -175, -140, 1, false, false, 3);
            followHeadingPID(-140, 0.6, 40, false, 3, false);


        }
*/
    }


        public enum CCAutoRingsLocation {
            CC_RING_UNKNOWN,
            CC_RING_BACK,
            CC_RING_MID,
            CC_RING_FRONT
        }

}


