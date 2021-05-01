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
    private int YELLOW_PERCENT = 92;
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


                        Rect TopRoi = new Rect(new Point(370, 450), new Point(390, 500));
                        Rect BotRoi = new Rect(new Point(455, 459), new Point(475, 500));
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
    public double gyroTurnNoStop(double speed,
                           double init_angle,
                           double angle,
                           int threshold,
                           boolean tank,
                           boolean leftTank,
                           double waitForSeconds) {
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //runTime.reset();

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() &&
                !onHeading(speed, init_angle, angle, threshold, tank, leftTank, P_TURN_COEFF)) {

            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(CCHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

       // robot.setPowerToDTMotors(0.05);
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
           //     Log.v("FollowPath", "here");
            } else {
              //  Log.v("FollowPath", "two");
                rightSpeed = Range.clip(rightSpeed,
                        -DT_TURN_SPEED_HIGH,
                        -DT_TURN_SPEED_LOW);
            }

            if (!tank) {
                leftSpeed = rightSpeed;
           //     Log.v("FollowPath", "no");
            } else if (leftTank) {
             //   Log.v("FollowPath", "yes");
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
                                    double waitForSec, boolean forward, boolean reducePower) {
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
              //  Log.v("BOK", "angle: " + angle + " error: " + error);
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                Log.v("BOK", "angle: " + angle + " error: " + error + " turn: " + turn + " distance: " + (targetEnc - robot.getAvgEncCount()));


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
                if(robot.getAvgEncCount()/targetEnc >= 0.6 && reducePower){
                    speedR *= .6;
                    speedL *= .6;
                    Log.v("BOK", "Reduced Power! " + speedR);
                }
                if(robot.getAvgEncCount()/targetEnc >= 0.8 && reducePower){
                    speedL *= .4;
                    speedR *= .4;
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
      //  robot.setModeForDTMotors(Dc);
       Log.v("BOK", "Final Distance: " +( robot.getAvgEncCount() - targetEnc));
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "followHeadingPID timed out!");
        }

    }
    protected void followHeadingPIDNoStop(double heading,
                                          double power,
                                          double dist,
                                          double waitForSec, boolean forward) {
        double angle, error, diffError, turn, speedL, speedR,
                sumError = 0, lastError = 0, lastTime = 0;
        double Kp = 0.01, Ki = 0, Kd = 0; // Ki = 0.165; Kd = 0.093;
        double targetEnc = robot.getTargetEncCount(dist); // convert inches to target enc count
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        Log.v("BOK", "followHeadingPIDNoStop: " + heading + ", dist: " + dist);
        robot.resetDTEncoders();
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  runTime.reset();

        while (opMode.opModeIsActive() &&
                (robot.getAvgEncCount() < targetEnc)) {

            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES);
                angle = angles.thirdAngle;
                error = angle - heading;
               // Log.v("BOK", "angle: " + angle + " error: " + error);
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                Log.v("BOK", "angle: " + angle + " error: " + error + " turn: " + turn + " distance: " + (targetEnc - robot.getAvgEncCount()));


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
                if(robot.getAvgEncCount()/targetEnc >= 0.75 || robot.getAvgEncCount()/targetEnc <= 0.25){
                    speedR *= .6;
                    speedL *= .6;
                    Log.v("BOK", "Reduced Power! " + speedR);
                }
                if(robot.getAvgEncCount()/targetEnc >= 0.9){
                    speedL *= .4;
                    speedR *= .4;
                }

                speedL = Range.clip(speedL, -0.9, 0.9);
                speedR = Range.clip(speedR, -0.9, 0.9);
                robot.setPowerToDTMotors(speedL, speedR);
                Log.v("BOK", "Follow Heading PID Speed Left: " + speedL + " Speed Right: " + speedR);
                lastError = error;
                lastTime = currTime;

            }
        }
        robot.setPowerToDTMotors(0);

        Log.v("BOK", "EncoderCounts: " + (targetEnc - robot.getAvgEncCount()));
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);

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


    protected void shootRings(int numRings, int waitForSec){
        int ringCount = 0;
        runTime.reset();
        robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_ANGLE_AUTO+.012));

        while(opMode.opModeIsActive() && ringCount < numRings && runTime.seconds() < waitForSec){

            if(ringCount == 1){
                robot.frontIntake.setPower(-1);
                robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.shooterServo.getPosition() - 0.007));
            }
          // robot.gateServo.setPosition(robot.GATE_DOWN);
            if(ringCount == 2) {
                robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_ANGLE_AUTO+.01));
            }

           robot.gateServo.setPosition(robot.GATE_UP);
           robot.verticalIntake.setPower(-1);
           opMode.sleep(750);
           ringCount++;
        //   robot.shooter.setPower(.95);
        }
    }
    protected void shootRingsTwo(int numRings, int waitForSec){
        int ringCount = 0;
        runTime.reset();
        robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_OPTIMUM_ANGLE+0.01));

        while(opMode.opModeIsActive() && ringCount < numRings && runTime.seconds() < waitForSec){
            if(ringCount == 1){
                robot.frontIntake.setPower(-1);
                // robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_ANGLE_AUTO)+.001);

            }

            // robot.gateServo.setPosition(robot.GATE_DOWN);
            if(ringCount == 2) {
                robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_OPTIMUM_ANGLE));
            }
            else if(ringCount == 1){
                robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_OPTIMUM_ANGLE+0.01));
            }
            robot.gateServo.setPosition(robot.GATE_UP);
            robot.verticalIntake.setPower(-1);
            opMode.sleep(750);
            ringCount++;
            //   robot.shooter.setPower(.95);
        }
    }
    protected void intakeRings(int waitForSec){
        robot.verticalIntake.setPower(-1);
        robot.frontIntake.setPower(-1);
        robot.intakePlate.setPosition(robot.PLATE_DOWN);
        runTime.reset();
        int counter = 0;
        while(opMode.opModeIsActive() && runTime.seconds() < waitForSec){
            if(counter == 10){
                robot.intakeGate.setPosition(robot.INTAKE_GATE_DOWN);
                robot.verticalIntake.setPower(0);
                //counter = 0;
            }
            if(robot.ods.getLightDetected() >= 0.4){
                counter++;
            }
            else {
                counter = 0;
            }


        }
        robot.frontIntake.setPower(0);
    }
    protected void SemiCircleMove(double leftPwr, double rightPwr, double leftEnc, double rightEnc){
        int left = (int)robot.getTargetEncCount(leftEnc);
        int right = (int)robot.getTargetEncCount(rightEnc);

        robot.setDTMotorEncoderTarget(left, right);
        robot.setPowerToDTMotors(leftPwr, rightPwr);
    }
    protected CCPoint goToPoint(Point target, CCPoint init_point, double turnSpeed,
                                double straightsSpeed, boolean forward, int waitForSec){
        double distance = Math.sqrt(Math.pow(target.x - init_point.x, 2) +
                Math.pow(target.y - init_point.y, 2));
        double theta = (Math.asin((target.y-init_point.y)/distance)*57.296);



        Log.v("FollowPath", "Current values: current X: " + init_point.x +
                ", current Y:" + init_point.y + ", currentTheta: " + init_point.theta);
        Log.v("FollowPath", "followPath target values: target X: " + target.x +
                ", target Y:" + target.y + ", target Theta: " + theta +
                ", target distance: " + distance);
        if(target.x < init_point.x){
            theta = -(theta -90);
        }
        else{
            theta = theta - 90;
        }
        //try wo/ gyro turn
        if(Math.abs(theta - init_point.theta) >= 1) {
            gyroTurn(turnSpeed, (init_point.theta - 90), (theta), 1, false,
                    false, 5);
        }
        followHeadingPID((theta), straightsSpeed, distance,
                false, 5, forward, true);
        Log.v("BOK", "      ");
        return new CCPoint((target.x > init_point.x) ? Math.abs((distance*Math.cos((Math.abs(theta)+90)/57.296))) + init_point.x : init_point.x - Math.abs((distance*Math.cos((Math.abs(theta)+90)/57.296))),
                (target.y > init_point.y) ? Math.abs((distance*Math.sin((Math.abs(theta)+90)/57.296))) + init_point.y : init_point.y - Math.abs((distance*Math.sin((Math.abs(theta)+90)/57.296))), (Math.abs(theta)+90));

    }
    protected CCPoint followPath(int numPoints, Point[] points, double lastx, double lasty,
                                 double lasttheta, double turnSpeed, double straightsSpeed, boolean forward,
                                 int waitForSec){
        runTime.reset();
        CCPoint tempPoint = new CCPoint(lastx, lasty, lasttheta);

            for (int i = 0; i <= numPoints; i++) {
                tempPoint = goToPoint(points[i], tempPoint,
                        0.3, 0.3, forward, 10);

            }

            robot.setPowerToDTMotors(0);
       return tempPoint;
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
    //Back(4 stack)
    Point origin = new Point(69, 7);
    Point nextPoint = new Point(75, 36);
    Point[] shootPositionBack = {new Point(67, 54.148), new Point(65, 62.398), new Point(63, 67.41), new Point(60, 71.299), new Point(58, 72)};
    Point movePointAfterShootBack = new Point(59, 76.05);//back
    Point[] wobblePositionBack = {new Point(61, 84.05), new Point(63, 92.45), new Point(65, 101.25), new Point(68.248, 116.394)};//back
    Point movePointAfterWobbleBack = new Point(47.752, 100);
    Point[] toRingStackBack = {new Point(45, 85), new Point(43, 72.2), new Point(41, 57.8), new Point(40, 50)};
    Point moveThroughRingsBackMid = new Point(68, 46);
    Point[] toSecondWobbleBackMid = {new Point(65, 45.5), new Point(60, 42.685), new Point(55, 37.36), new Point(53, 34.536)};//back
    Point leaveWobbleBackMid = new Point(65, 62.398);
    Point[] backToShootBackMid = {new Point(63, 67.41), new Point(60, 71.299), new Point(58, 72)};
    Point moveAfterSecondWobbleBack = new Point(85, 124);
    Point parkBack = new Point(80, 80);

    //Mid(1 stack)
    Point movePointAfterShootMid = new Point(68, 115.2);//back
    Point movePointAfterWobbleMid = new Point(44.14, 49.694);
    Point placeSecondWobbleMid = new Point(48, 100);
    Point parkMid = new Point(48, 80);

    //Front(0 stack)
    Point pointForWobbleFront = new Point(80, 92);
    Point pointForSecondWobbleFront = new Point(53, 34);
    Point placeWobbleFront = new Point(80, 80);
    Point parkFront = new Point(72, 40);


    CCPoint init_point;
    protected void runAuto(boolean inside, boolean startStone, boolean park) {
       /* Point[] testPoints = {new Point(25, 3),
                new Point(27, 5.196),  new Point(29, 6.708),
                 new Point(31, 7.937),
                new Point(33, 9),  new Point(35, 9.949)};
        followPath(5, testPoints, 24, 0, 90, 0.35, 0.35, true,10);

        //SemiCircleMove(-0.5, -0.166, 37.699, 12.566);
        //followHeadingPID(0, 0.2, 24, false, 3, true);
*/
        CCAutoRingsLocation loc = CCAutoRingsLocation.CC_RING_UNKNOWN;
        Log.v("BOK", "Angle at runAuto start " +
                robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle);


        try {
            loc = findRings();
        } catch (java.lang.Exception e) {
            Log.v("BOK", "Exception at findCube()");
        }

        Log.v("BOK", "StoneLoc: " + loc);

        Log.v("BOK", "Color: " + allianceColor + " Inside Route: " +
                inside + " Stating At Stone: " + startStone);

        robot.shooter.setPower(robot.getShooterPwr(robot.getBatteryVoltage(), .9));
        followHeadingPID(0, 0.5, 42, false, 4, false, true);
        gyroTurn(0.2, 0, 3, 1, false, false, 3);
        //opMode.sleep(1000);
        robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
        opMode.sleep(300);
        robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
        robot.shooter.setPower(.95);
        opMode.sleep(300);
        robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
        opMode.sleep(300);
        robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
        opMode.sleep(300);
        robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
        opMode.sleep(300);
        robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
        robot.shooter.setPower(0);

        if(loc == CCAutoRingsLocation.CC_RING_BACK){
            followHeadingPID(0, 0.5, 50, false, 4, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            gyroTurn(0.2, 0, -5, 1, false, false, 3);
            robot.intakePlate.setPosition(robot.PLATE_DOWN);
            robot.frontIntake.setPower(-1);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            followHeadingPID(-5, 0.5, 33, false, 4, true, true);
            gyroTurn(0.25, -5, -42, 1, false, false, 4);
            followHeadingPID(-42, 0.25, 18, false, 5, true, false);
            followHeadingPID(-42, 0.2, 1, false, 2, false, false);
           // opMode.sleep(200);
            followHeadingPID(-42, 0.2, 4, false, 2, true, false);
            //opMode.sleep(200);
            followHeadingPID(-42, 0.2, 1, false, 2, false, false);
            followHeadingPID(-42, 0.2, 3, false, 2, true, false);
          //  followHeadingPID(-32, 0.2, 1, false, 10, false, false);

            //   followHeadingPID(-30, 0.2, 2, false, 2, false, false);
           //followHeadingPID(-32, 0.4, 5, false, 10, false, false);
           //robot.frontIntake.setPower(-1);
            //followHeadingPID(-32, 0.4, 5, false, 10, true, false);

            //followHeadingPID(-30, 0.2, 2, false, 2, false, false);

            //followHeadingPID(-25, 0.4, 5, false, 10, true, false);
            //opMode.sleep(700);
          //  robot.frontIntake.setPower(0);
            robot.shooter.setPower(robot.getShooterPwr(robot.getBatteryVoltage(),.95));
            gyroTurn(0.3, -42, -10, 1, false, false, 3);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            robot.shooter.setPower(.95);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            robot.shooter.setPower(0);
          //  gyroTurn(0.2, -10, -32, 1, false, false, 3);
            //followHeadingPID(-32, 0.2, 3, false, 2, true, false);
          //  followHeadingPID(-10, 0.4, 5, false, 3, false, true);

            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            robot.frontIntake.setPower(0);
          /*  robot.intakePlate.setPosition(robot.PLATE_DOWN);
            robot.frontIntake.setPower(-1);
            followHeadingPID(-32, 0.2, 5, false, 3, true, false);
            opMode.sleep(500);
           // robot.frontIntake.setPower(0);
            followHeadingPID(-32, 0.3, 15, false, 3, false, true);
            //opMode.sleep(500);
            robot.frontIntake.setPower(0);
            robot.shooter.setPower(.9);
            gyroTurn(0.2, -32, -9, 1, false, false, 3);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            robot.shooter.setPower(.95);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            robot.shooter.setPower(0);

           */
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            gyroTurn(0.45, -13, 178, 2, false, false, 4);
            followHeadingPID(178, 0.3, 5, false, 3, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);

            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-200);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(178, 0.7, 35, false, 4, true, true);
            gyroTurn(0.5, 178, 0, 2, false, false, 3);
            followHeadingPID(0, 0.7, 25, false, 3, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-200);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(0, 0.7, 10, false, 3, true, true);
            robot.setPowerToDTMotors(0);
        }
        if(loc == CCAutoRingsLocation.CC_RING_MID){
            followHeadingPID(20, 0.5, 40, false, 4, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            gyroTurn(0.2, 0, -5, 1, false, false, 3);
            robot.intakePlate.setPosition(robot.PLATE_DOWN);
            robot.frontIntake.setPower(-1);
            followHeadingPID(-5, 0.5, 40, false, 4, true, true);
          //  gyroTurn(0.25, -5, -32, 1, false, false, 4);
            //followHeadingPID(-32, 0.2, 12, false, 10, true, false);
            //followHeadingPID(-32, 0.2, 1, false, 10, false, false);

            //followHeadingPID(-32, 0.2, 3, false, 10, true, false);
            //followHeadingPID(-32, 0.2, 1, false, 10, false, false);

            //followHeadingPID(-32, 0.2, 2, false, 10, true, false);
            //  followHeadingPID(-32, 0.2, 1, false, 10, false, false);

            //   followHeadingPID(-30, 0.2, 2, false, 2, false, false);
            //followHeadingPID(-32, 0.4, 5, false, 10, false, false);
            //robot.frontIntake.setPower(-1);
            //followHeadingPID(-32, 0.4, 5, false, 10, true, false);

            //followHeadingPID(-30, 0.2, 2, false, 2, false, false);

            //followHeadingPID(-25, 0.4, 5, false, 10, true, false);
            //opMode.sleep(700);
            //  robot.frontIntake.setPower(0);
            robot.shooter.setPower(robot.getShooterPwr(robot.getBatteryVoltage(),.92));
            gyroTurn(0.25, -20, 10, 1, false, false, 3);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            robot.shooter.setPower(.95);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
            opMode.sleep(300);
            robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
            robot.shooter.setPower(0);
            //  gyroTurn(0.2, -10, -32, 1, false, false, 3);
            //followHeadingPID(-32, 0.2, 3, false, 2, true, false);
            followHeadingPID(-10, 0.4, 5, false, 3, false, true);

            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            robot.frontIntake.setPower(0);

            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            gyroTurn(0.45, -13, 175, 2, false, false, 4);
            followHeadingPID(175, 0.3, 17, false, 3, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-200);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            opMode.sleep(500);
            followHeadingPID(175, 0.7, 30, false, 4, true, true);
            gyroTurn(0.5, 175, 0, 2, false, false, 3);
            followHeadingPID(0, 0.7, 20, false, 3, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-200);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(0, 0.7, 10, false, 3, true, true);
            robot.setPowerToDTMotors(0);
        }
        if(loc == CCAutoRingsLocation.CC_RING_FRONT){
            followHeadingPID(0, 0.5, 20, false, 4, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            gyroTurn(0.25, 0, -160, 1, false, false, 3);
            followHeadingPID(-160, 0.5, 40, false, 4, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            robot.frontIntake.setPower(0);

            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(-160, 0.2, 10, false, 3, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-200);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(-160, 0.5, 40, false, 4, true, true);
            gyroTurn(0.24, -160, 0, 1, false, false, 3);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-200);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(0, 0.2, 10, false, 3, true, true);
            gyroTurn(0.25, 0, 160, 2, false, false, 3);
            followHeadingPID(160, 0.3, 10, false, 4, true, true);



        }
       /*
        robot.shooter.setPower(0.95);
        robot.shooterServo.setPosition(robot.getShooterAngleAutonomous(robot.getBatteryVoltage(), robot.SHOOTER_ANGLE_AUTO));
        followHeadingPID(0, 0.4, 10, false, 2, true, true);
        gyroTurn(0.35, 0, 93, 1, false, false, 3);
        shootRings(3, 7);
        //robot.verticalIntake.setPower(0);
        robot.shooter.setPower(0);
        //robot.frontIntake.setPower(0);
        robot.gateServo.setPosition(robot.GATE_DOWN);

        if(loc == CCAutoRingsLocation.CC_RING_BACK){
            gyroTurn(0.27, 93, 22, 1, false, false, 3);
            robot.intakePlate.setPosition(robot.PLATE_DOWN);
            robot.verticalIntake.setPower(-1);
            followHeadingPID(22, 0.24, 27, false, 3, true, false);
         //   followHeadingPID(22, 0.3, 5, false, 2, false);
           // followHeadingPID(22, 0.2, 10, false, 3, true);
            //robot.intakeGate.setPosition(robot.INTAKE_GATE_DOWN);
           // robot.verticalIntake.setPower(0);

            //  followHeadingPID(22, 0.3, 5, false, 2, false);
        //    followHeadingPID(22, 0.2, 5, false, 1, true);

            opMode.sleep(100);
            robot.shooter.setPower(0);
            robot.shooter.setPower(0.95);
            robot.intakeGate.setPosition(robot.INTAKE_GATE_UP);
            robot.frontIntake.setPower(0);

            gyroTurn(0.35, 0, 90, 1, false, false, 3);

            shootRingsTwo(3, 7);
            robot.verticalIntake.setPower(0);
            robot.shooter.setPower(0);
            robot.frontIntake.setPower(0);
            gyroTurn(0.45, 90, 170, 1, false, false, 3);
            followHeadingPID(170, 0.8, 50, false, 4, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-315);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(170, 0.7, 50, false, 4, true, true);
            gyroTurn(0.45, 170, 0, 2, false, false, 4);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-350);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(0, 0.3, 15 , false, 3, false, true);

            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);

            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-100);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.4, 0, 170, 2, false, false, 3);
            followHeadingPID(170, 0.9, 67, false, 3, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(170, 0.8, 15, false, 1, true, true);

        }
        if(loc == CCAutoRingsLocation.CC_RING_MID){
            gyroTurn(0.27, 93, 22, 1, false, false, 3);
            robot.intakePlate.setPosition(robot.PLATE_DOWN);
            //robot.verticalIntake.setPower(-1);
            followHeadingPID(22, 0.24, 20, false, 3, true, false);
            //   followHeadingPID(22, 0.3, 5, false, 2, false);
            // followHeadingPID(22, 0.2, 10, false, 3, true);
            //robot.intakeGate.setPosition(robot.INTAKE_GATE_DOWN);
            // robot.verticalIntake.setPower(0);

            //  followHeadingPID(22, 0.3, 5, false, 2, false);
            //    followHeadingPID(22, 0.2, 5, false, 1, true);

            opMode.sleep(1000);
            robot.shooter.setPower(0);
            robot.frontIntake.setPower(0);
            robot.shooter.setPower(0.95);
            robot.intakeGate.setPosition(robot.INTAKE_GATE_UP);
            gyroTurn(0.35, 0, 90, 1, false, false, 3);
            shootRingsTwo(1, 2);
            robot.verticalIntake.setPower(0);
            robot.shooter.setPower(0);
            robot.frontIntake.setPower(0);
            gyroTurn(0.35, 90, -170, 1, false, false, 3);
            followHeadingPID(-170, 0.75, 37, false, 4, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            followHeadingPID(-170, 0.7, 30, false, 4, true, true);
            gyroTurn(0.45, -170, -1, 2, false, false, 4);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-350);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(-1, 0.3, 8, false, 3, false, true);

            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);

            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-100);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.4, -1, -170, 2, false, false, 3);
            followHeadingPID(-170, 0.75, 37, false, 3, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-350);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            followHeadingPID(-170, 0.2, 1, false, 1, true, true);

        }
        if(loc == CCAutoRingsLocation.CC_RING_FRONT){
          //  gyroTurn(0.27, 94, 22, 1, false, false, 3);
           // robot.intakePlate.setPosition(robot.PLATE_DOWN);
            //robot.verticalIntake.setPower(-1);
          //  followHeadingPID(22, 0.24, 20, false, 3, true, false);
            //   followHeadingPID(22, 0.3, 5, false, 2, false);
            // followHeadingPID(22, 0.2, 10, false, 3, true);
            //robot.intakeGate.setPosition(robot.INTAKE_GATE_DOWN);
            // robot.verticalIntake.setPower(0);

            //  followHeadingPID(22, 0.3, 5, false, 2, false);
            //    followHeadingPID(22, 0.2, 5, false, 1, true);

         //   opMode.sleep(1000);
            robot.shooter.setPower(0);
            robot.frontIntake.setPower(0);
            //robot.shooter.setPower(0.95);
            robot.intakeGate.setPosition(robot.INTAKE_GATE_UP);
            gyroTurn(0.35, 0, 170, 1, false, false, 3);
           // shootRingsTwo(1, 2);
            robot.verticalIntake.setPower(0);
            robot.shooter.setPower(0);
            robot.frontIntake.setPower(0);
          //  gyroTurn(0.45, 90, 175, 1, false, false, 3);
            followHeadingPID(170, 0.5, 32, false, 4, false, true);
            robot.wobbleGoalArm.setPower(-0.3);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.5);
            robot.wobbleGoalArm.setTargetPosition(0);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.3, 170, 110, 1, false, false, 3);
            followHeadingPID(110, 0.5, 9, false, 4, true, true);
            gyroTurn(0.45, 110, 1, 2, false, false, 4);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-330);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(1, 0.3, 17, false, 3, false, true);

            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);

            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-100);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gyroTurn(0.4, 1, 160, 2, false, false, 3);
            followHeadingPID(160, 0.6, 25, false, 3, false, true);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            robot.wobbleGoalArm.setPower(-0.4);
            robot.wobbleGoalArm.setTargetPosition(-350);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            followHeadingPID(160, 0.3, 5, false, 1, true, true);
            gyroTurn(0.3, 160, 20, 1, false, false, 3);
            followHeadingPID(20, 0.3, 20, false,3, true, true);

        }
        robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
        robot.wobbleGoalArm.setPower(-0.4);
        robot.wobbleGoalArm.setTargetPosition(-400);
        robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*gyroTurn(0.3, 0, -20, 1, false, false, 3);
        followHeadingPID(-20, 0.8, 15, false, 2, true);
        gyroTurn(0.3, -20, 25, 1, false, false, 3);
        followHeadingPID(25, 0.8, 20, false, 4, true);
        gyroTurn(0.25, 25, 90, 1, false, false, 3);
        followHeadingPID(90, 0.4, 3, false, 2, true);
        shootRings(3, 7);
        robot.verticalIntake.setPower(0);
        robot.shooter.setPower(0);
        robot.frontIntake.setPower(0);
        robot.gateServo.setPosition(robot.GATE_DOWN);
        /*
        if (loc == CCAutoRingsLocation.CC_RING_BACK) {
            gyroTurn(0.3, 90, 170, 1, false, false, 3);
            followHeadingPID(170, 0.8, 37, false, 3, false);
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
            followHeadingPID(160, 0.8, 49, false, 6, true);
            gyroTurn(0.3, 160, 30, 1, false, false, 3);

            //   robot.frontIntake.setPower(-1);
            robot.wobbleGoalArm.setPower(-0.5);
            robot.wobbleGoalArm.setTargetPosition(-325);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            followHeadingPID(30, 0.4, 8, false, 3, false);
            robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            opMode.sleep(500);
            robot.wobbleGoalArm.setPower(0.2);
            robot.wobbleGoalArm.setTargetPosition(-150);
            robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.sleep(500);
            followHeadingPID(30, 0.4, 5, false, 3, true);
            gyroTurn(0.4, 30, 160, 2, false, false, 3);
            followHeadingPID(160, 0.9, 55, false, 5, false);
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



