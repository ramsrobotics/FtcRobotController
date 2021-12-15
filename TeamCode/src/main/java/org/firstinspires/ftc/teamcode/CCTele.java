package org.firstinspires.ftc.teamcode;

import android.util.Log;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Thread.sleep;

public class CCTele {

    // CONSTANTS
    //private static final double GAME_STICK_DEAD_ZONE_LEFT_STICK = 0.3;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double INTAKE_MOTOR_CAP_SPEED = 0.9;
    private static final double Kp = 0.7, Ki = 0.525, Kd = 0.2;
    boolean closeGamepad = false;
    boolean downFinished = true;
    private CCHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = CCHardwareBot.SPEED_COEFF_FAST;
    int counter = 0;
    int liftTargetPosition = 0;
    int counterTransfer = 0;
    boolean next = false;
    boolean shooterManualControl = false;
    boolean transfer = false;
    boolean soundDebounce = false;
    double shooterPwr = .95;
    boolean quackDebounce = false;
    boolean bassDebounce = false;

    protected ElapsedTime runTime = new ElapsedTime();



    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      CCHardwareBot robot) {
        this.opMode = opMode;
        this.robot = robot;
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.initializeImu();
        robot.linearLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.quack();

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware() {
       // CCPoint currentPosition = new CCPoint(0, 0, 0, 0, 0, 0, 0, 0);
        runTime.reset();
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode
          //  currentPosition = updatePosition(currentPosition);
           // opMode.telemetry.addData("Position: ", "x: " + currentPosition.x + " y: " + currentPosition.y);
            moveRobot();
            if (opMode.gamepad1.y) {
                speedCoef = CCHardwareBot.SPEED_COEFF_SLOW;
                counter = 0;
            }
            if (opMode.gamepad1.a) {
                speedCoef = CCHardwareBot.SPEED_COEFF_FAST;
            }

            if (opMode.gamepad2.x) {
                //Middle platform
                liftTargetPosition = -280;
            }
            if (opMode.gamepad2.y) {
                //top platform
                liftTargetPosition = -550;
            }
            if (opMode.gamepad2.b) {
                if (robot.deliverServo.getPosition() >= 0.6 && !soundDebounce) {
                    soundDebounce = true;
                    robot.sound("slip");
                }
                robot.deliverServo.setPosition(0.3);
            }
            if (soundDebounce && robot.deliverServo.getPosition() >= 0.6){
                soundDebounce = false;
            }

            if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE){
                robot.deliverServo.setPosition(robot.deliverServo.getPosition() - 0.01);
                opMode.telemetry.addData("CC", "Deliver: " + robot.deliverServo.getPosition());
            }
            if (opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                robot.deliverServo.setPosition(robot.deliverServo.getPosition() + 0.01);
                opMode.telemetry.addData("CC", "Deliver: " + robot.deliverServo.getPosition());
            }
            if (opMode.gamepad2.left_stick_button){
                robot.deliverServo.setPosition(1);
            }
            if (opMode.gamepad2.left_trigger != 0){
                robot.deliverServo.setPosition(0.7);
            }
            if(opMode.gamepad2.right_trigger > 0){
                robot.carouSpinner.setPower(0.5);
                quackDebounce = true;
            }
            if (opMode.gamepad2.right_bumper) {
                robot.carouSpinner.setPower(-0.5);
                quackDebounce = true;
            }
            if(opMode.gamepad2.right_trigger == 0 && !opMode.gamepad2.right_bumper){
                robot.carouSpinner.setPower(0);
                if (quackDebounce){
                    quackDebounce = false;
                    robot.sound("quack");
                }
            }
            if (opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE) { //down
                downFinished = false;
                if(robot.deliverServo.getPosition() >= 0.75){
                robot.deliverServo.setPosition(0.9); }
                robot.linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.linearLift.setPower(0.5);
                liftTargetPosition = robot.linearLift.getCurrentPosition();
            }
            if (opMode.gamepad2.right_stick_y < -GAME_TRIGGER_DEAD_ZONE) { //up
                downFinished = false;
                if(robot.deliverServo.getPosition() >= 0.75){
                    robot.deliverServo.setPosition(0.9); }
                robot.linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.linearLift.setPower(-0.2);
                liftTargetPosition = robot.linearLift.getCurrentPosition();
            }
            if(opMode.gamepad2.right_stick_y > -GAME_TRIGGER_DEAD_ZONE && opMode.gamepad2.right_stick_y < GAME_TRIGGER_DEAD_ZONE){
                robot.linearLift.setTargetPosition(liftTargetPosition);
                robot.linearLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(liftTargetPosition != 0){
                   robot.linearLift.setPower(0.1);
               } else {
                    robot.linearLift.setPower(1);
                    //robot.linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    //robot.linearLift.setPower(0);
                    if(robot.linearLift.getCurrentPosition() >= -45) {
                        downFinished = true;
                        robot.sound("anvil");
                    }
               }
            }
            if(opMode.gamepad2.back){
                robot.deliverServo.setPosition(0.925);
                liftTargetPosition = 0;
                downFinished = false;
            }
            if(downFinished) {
                robot.deliverServo.setPosition(1);
            }
            if (bassDebounce && robot.linearLift.getCurrentPosition() < -40){
                bassDebounce = false;
            }
            if(opMode.gamepad2.right_stick_button){
                if (!bassDebounce && robot.linearLift.getCurrentPosition() > -20) {
                    bassDebounce = true;
                    robot.sound("bass");
                }
                opMode.telemetry.addData("CC", "Lift: " + robot.linearLift.getCurrentPosition());
                liftTargetPosition = -50;
            }

            if(opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_down){
                if (opMode.gamepad2.dpad_up){
                    robot.frontIntake.setPower(0.5);
                } else {
                    robot.frontIntake.setPower(-0.5);
                }
            } else {
                robot.frontIntake.setPower(0);
            }

            if(opMode.gamepad2.dpad_left){
                //TSE Up
                robot.tseRotateServo.setPosition(robot.tseRotateServo.getPosition() + 0.012);
            }
            if(opMode.gamepad2.dpad_right){
                //TSE Down
                robot.tseRotateServo.setPosition(robot.tseRotateServo.getPosition() - 0.012);
            }
            if (opMode.gamepad2.a) {
                robot.tseExtendServo.setDirection(FORWARD);
                robot.tseExtendServo.setPower(1);
            } else if (opMode.gamepad2.left_bumper) {
                robot.tseExtendServo.setDirection(REVERSE);
                robot.tseExtendServo.setPower(1);
            } else {
                robot.tseExtendServo.setDirection(REVERSE);
                robot.tseExtendServo.setPower(0.394);
            }

            opMode.telemetry.update();
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }
    private void moveRobot() {
        robot.moveRobotTele(speedCoef);
    }

    public enum BoKTeleStatus {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }
}
