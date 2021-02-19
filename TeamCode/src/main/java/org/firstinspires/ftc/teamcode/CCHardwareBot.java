package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Box3D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class CCHardwareBot {
    // CONSTANTS
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT = 10;

    protected static final double SPEED_COEFF_SLOW = 0.35;
    protected static final double SPEED_COEFF_FAST = 0.8;
    protected static final double SPEED_COEFF_TURN = 0.7;

    protected static final double GAME_STICK_DEAD_ZONE = 0.1;
    protected static final int WAIT_PERIOD = 40; // 40 ms

    protected final double BOX_UP = 1.0;
    protected final double BOX_DOWN = 0.65;

    protected final double FLICKER_OUT = 0.43;
    protected final double FLICKER_IN = 0.03;

    protected final double WOBBLE_GRIP = 0;
    protected final double WOBBLE_RELEASE = 0.3;
    //Motors
   private static final String WOBBLE_GOAL_ARM = "wbA";
   private static final String FRONT_INTAKE_MOTOR = "fiM";
   private static final String SHOOTER_MOTOR = "shM";
   private static final String VERTICAL_INTAKE_MOTOR = "viM";
    //Servos
    private static final String SHOOTER_SERVO = "shS";
    private static final String BOX_FLICKER_SERVO = "bfS";
    private static final String BOX_SERVO = "boS";
    private static final String WOBBLE_CLAW_SERVO = "wbS";
    //Sensors
    private static final String IMU_TOP = "imu_top";        // IMU
    private static final String DISTANCE_SENSOR_BACK = "dsB";
    private static final String DISTANCE_SENSOR_FRONT = "dsF";

    protected DcMotor wobbleGoalArm;
    protected DcMotor frontIntake;
    protected DcMotor shooter;
    protected DcMotor verticalIntake;

    protected Servo shooterServo;
    protected Servo boxFlickerServo;
    protected Servo boxServo;
    protected Servo wobbleClaw;

    // Sensors
    protected BNO055IMU imu;

    protected AnalogInput distanceForward;
    protected AnalogInput distanceRight;
    protected AnalogInput distanceBack;
    protected AnalogInput distanceLeft;
    protected OpticalDistanceSensor opticalDistanceSensor;
    LinearOpMode opMode; // current opMode
    private Orientation angles;

    // waitForTicks
    private ElapsedTime period = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    protected BoKHardwareStatus initHardware(LinearOpMode opMode) {
        this.opMode = opMode;
        // First initialize the drive train
        BoKHardwareStatus rc = initDriveTrainMotors();
        if (rc == BoKHardwareStatus.BOK_HARDWARE_SUCCESS) {
            // Next initialize the other motors and sensors
            rc = initMotorsAndSensors();
        }
        return rc;
    }

    /*
     * The initHardware() method initializes the hardware on the robot including the drive train.
     * It calls the abstract initDriveTrainMotors() and initMotorsAndSensors() methods.
     * Returns BOK_SUCCESS if the initialization is successful, BOK_FAILURE otherwise.
     */

    /*
     * The initMotorsAndSensors() method initializes the motors (other than the drive train) and
     * sensors on the robot.
     */
    private BoKHardwareStatus initMotorsAndSensors() {
        //Motors
        wobbleGoalArm = opMode.hardwareMap.dcMotor.get(WOBBLE_GOAL_ARM);
        if(wobbleGoalArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        frontIntake = opMode.hardwareMap.dcMotor.get(FRONT_INTAKE_MOTOR);
        if(frontIntake == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        shooter = opMode.hardwareMap.dcMotor.get(SHOOTER_MOTOR);
        if(shooter == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        verticalIntake = opMode.hardwareMap.dcMotor.get(VERTICAL_INTAKE_MOTOR);
        if(verticalIntake == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        //Servos;
        shooterServo = opMode.hardwareMap.servo.get(SHOOTER_SERVO);
        if(shooterServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        boxFlickerServo = opMode.hardwareMap.servo.get(BOX_FLICKER_SERVO);
        if(boxFlickerServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        boxServo = opMode.hardwareMap.servo.get(BOX_SERVO);
        if(boxServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        wobbleClaw = opMode.hardwareMap.servo.get(WOBBLE_CLAW_SERVO);
        if(wobbleClaw == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        wobbleGoalArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        //Sensors



     distanceBack = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_BACK);
        if (distanceBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        distanceForward = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_FRONT);
        if (distanceForward == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

    */


        //Dc Motor Init
      //  liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Servos initialization
        /*
        if (opMode.getClass().getName().contains("Tele")) {
         //   intakeServo.setPosition(INTAKE_GRAB_POS);
           // inRotateServo.setPosition(ROTATE_UP_POS);
            //foundationGripServo.setPosition(FOUNDATION_GRIP_DOWN);
        } else {
            // Do nothing for Teleop so that the robot hardware does not move during
            // initialization
        }

         end comment here*/
        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if (imu == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

       // wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // wobbleGoalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    protected void initializeImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors();

    // Using the drive train is public
    protected abstract void testDTMotors();

   // protected abstract void moveTank(double leftPwr, double rightPwr, boolean rightPos, boolean leftPos);

    protected abstract void resetDTEncoders();

    protected abstract boolean areDTMotorsBusy();

    protected abstract void setPowerToDTMotors(double power);

    protected abstract void setPowerToDTMotors(double power, boolean forward);

    protected abstract void setPowerToDTMotors(double leftPower, double rightPower);

    protected abstract void setDTMotorEncoderTarget(int leftTarget, int rightTarget);

    protected abstract void setPowerToDTMotorsStrafe(double power, boolean right);

    protected abstract void setModeForDTMotors(DcMotor.RunMode runMode);

    protected abstract void setOnHeading(double leftPower, double rightPower);

    // Autonomous driving
    protected abstract int startMove(double leftPower,
                                     double rightPower,
                                     double inches,
                                     boolean backward);

    protected abstract void startEncMove(double leftPower,
                                         double rightPower,
                                         int encCount,
                                         boolean forward);

    protected abstract int startStrafe(double power, double rotations,
                                       boolean right) throws UnsupportedOperationException;

    protected abstract int startStrafeWEnc(double power, double rotations,
                                           boolean right) throws UnsupportedOperationException;

    protected abstract void stopMove();

    protected abstract double getTargetEncCount(double targetDistanceInches);

    protected abstract double getAvgEncCount();

    protected abstract int getLFEncCount();

    protected abstract int getRFEncCount();

    // Teleop driving
    protected abstract void moveRobotTele(double speedCoef);

    /**
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    protected void waitForTick(long periodMs) {
        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    protected double getAngle() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    /**
     * getDistanceCM
     *
     * @param mb1240
     * @param target
     * @param time
     * @param opMode
     * @return
     */
    protected double getDistanceCM(AnalogInput mb1240, double target, double time, CCAutoOpMode opMode) {
        runTime.reset();
        double dist = (mb1240.getVoltage() / 0.003222);
        while (((dist > target) || (dist == 0)) && (runTime.seconds() <= time) && opMode.opModeIsActive())
            dist = (mb1240.getVoltage() / 0.00322);
        return (runTime.seconds() > time) ? target : dist;
        //return mb1240.getVoltage() / 0.00189;
    }



    // return status
    protected enum BoKHardwareStatus {
        BOK_HARDWARE_FAILURE,
        BOK_HARDWARE_SUCCESS
    }
}
