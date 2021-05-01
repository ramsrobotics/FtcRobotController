package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class CCTele {

    // CONSTANTS
    //private static final double GAME_STICK_DEAD_ZONE_LEFT_STICK = 0.3;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double INTAKE_MOTOR_CAP_SPEED = 0.9;
    private static final double Kp = 0.7, Ki = 0.525, Kd = 0.2;
    boolean closeGamepad = false;
    private CCHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = CCHardwareBot.SPEED_COEFF_FAST;
    int counter = 0;
    int counterTransfer = 0;
    boolean next = false;
    boolean shooterManualControl = false;
    boolean transfer = false;
    double shooterPwr = .95;

    protected ElapsedTime runTime = new ElapsedTime();



    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      CCHardwareBot robot) {
        this.opMode = opMode;
        this.robot = robot;
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.initializeImu();

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
            /*
            if(counter == 11){
                robot.intakeGate.setPosition(robot.INTAKE_GATE_DOWN);
                counter = 0;
            }
            if(robot.ods.getLightDetected() >= 0.4){
                counter++;
            }
            else {
                counter = 0;
            }

             */
            if(opMode.gamepad1.dpad_up){
                robot.intakePlate.setPosition(robot.PLATE_UP);

            }
            if(opMode.gamepad1.dpad_down){
                robot.intakePlate.setPosition(robot.PLATE_DOWN);
            }
            if(opMode.gamepad2.b){
               // robot.intakeGate.setPosition(robot.INTAKE_GATE_DOWN);
            }
            if(opMode.gamepad2.a){
               // robot.intakeGate.setPosition(robot.INTAKE_GATE_UP);
            }
            if (opMode.gamepad1.y) {
                speedCoef = CCHardwareBot.SPEED_COEFF_SLOW;
                counter = 0;
            }
            if (opMode.gamepad1.a) {
                speedCoef = CCHardwareBot.SPEED_COEFF_FAST;
            }
           if (opMode.gamepad2.a) {
                shooterPwr = .75;

            }
            if (opMode.gamepad2.b) {
                shooterPwr = .95;
            }


/*
            if (opMode.gamepad2.dpad_up) {
              //  robot.gateServo.setPosition(robot.GATE_UP);
                robot.shooterServo.setPosition(robot.SHOOTER_POWER_SHOT_ANGLE);
                shooterManualControl = true;
            }
            if (opMode.gamepad2.dpad_down) {
           //     robot.gateServo.setPosition(robot.GATE_DOWN);
            }
            if (opMode.gamepad2.dpad_left ) {
                //robot.gateServo.setPosition(robot.FLICKER_OUT);
                robot.verticalIntake.setPower(1);
            }
            if (opMode.gamepad2.dpad_right) {
               // robot.gateServo.setPosition(robot.FLICKER_IN);
                Log.v("BOK", "Angle: " + robot.getShooterAngle(robot.getBatteryVoltage()) + ", Voltage: " + robot.getBatteryVoltage());
                robot.verticalIntake.setPower(-1);

            }

 */


            if (opMode.gamepad2.y) {
                robot.wobbleGoalArm.setPower(-0.2);
                robot.wobbleGoalArm.setTargetPosition(350);//350
                robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (opMode.gamepad2.x) {
                robot.wobbleGoalArm.setPower(0.5);
                robot.wobbleGoalArm.setTargetPosition(120);//150
                robot.wobbleGoalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
            if (opMode.gamepad2.right_bumper) {
                robot.wobbleClaw.setPosition(robot.WOBBLE_GRIP);
            }
            if (opMode.gamepad2.left_bumper) {
                robot.wobbleClaw.setPosition(robot.WOBBLE_RELEASE);
            }


            if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE){
                shooterManualControl = true;
             //   robot.shooterServo.setPosition(robot.shooterServo.getPosition() - 0.001);
              //  Log.v("CC", "Shooter: " + robot.shooterServo.getPosition());
            }
            if(opMode.gamepad2.dpad_down){
                shooterManualControl = false;
            }
            if (opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
              //  robot.shooterServo.setPosition(robot.shooterServo.getPosition() + 0.001);
             //   Log.v("CC", "Shooter: " + robot.shooterServo.getPosition());
            }
            if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
               // robot.shooterServo.setPosition(robot.getShooterAngle(robot.getBatteryVoltage()));
              //  robot.gateServo.setPosition(robot.GATE_UP);
              //  robot.intakeGate.setPosition(robot.INTAKE_GATE_UP);
              //  robot.verticalIntake.setPower(-1);
              //  robot.frontIntake.setPower(-1);
                robot.shooter.setPower(robot.getShooterPwr(robot.getBatteryVoltage(), shooterPwr));

               if(counterTransfer == 7){
                    robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
                    transfer = false;
                    Log.v("BOK", "lmao");
               }
               if(counterTransfer == 0){
                    robot.transerFlick.setPosition(robot.TRANSFER_ACTIVE);
                    transfer = true;
                   Log.v("BOK", "lmao2");

               }
               if(transfer) {
                   counterTransfer = counterTransfer + 1;
               }
               if(!transfer && counterTransfer != 0){
                   counterTransfer = counterTransfer - 1;
               }


            }
            if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
              //  shooterManualControl = true;
              //  robot.shooterServo.setPosition(robot.SHOOTER_POWER_SHOT_ANGLE);
                //  robot.boxFlickerServo.setPosition(robot.FLICKER_IN);//maybe
                robot.shooter.setPower(robot.getShooterPwr(robot.getBatteryVoltage(),shooterPwr));

            }
            if(opMode.gamepad2.left_trigger == 0 ){
              //  robot.gateServo.setPosition(robot.GATE_DOWN);
            //    robot.frontIntake.setPower(0);
                counterTransfer = 0;
                robot.transerFlick.setPosition(robot.TRANSFER_PASSIVE);
              //  robot.verticalIntake.setPower(0);
            }
            if(opMode.gamepad2.right_trigger == 0){
                robot.shooter.setPower(0);
            }
            if (opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                robot.frontIntake.setPower(1);
            }
            if (opMode.gamepad2.right_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                robot.frontIntake.setPower(-1);
            }
            if(opMode.gamepad2.right_stick_y == 0){
                robot.frontIntake.setPower(0);
            }

        /*    if (!shooterManualControl){
               robot.shooterServo.setPosition(robot.getShooterAngle(robot.getBatteryVoltage()));
            }

         */
            opMode.telemetry.update();
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }
    private CCPoint updatePosition(CCPoint last_point){
        //meter/s^2 to in/s^2
        //radians to degrees
       // runTime.reset();
        double acceleration = robot.imu.getLinearAcceleration().yAccel*39.3701;
        if(acceleration< 2 && acceleration > -2){
            acceleration = 0;
        }
        Log.v("BOK", "    ");
        Log.v("BOK", "accel: " + robot.imu.getLinearAcceleration().yAccel);
        double theta = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle;
        if(theta < 1 && theta > -1){
            theta = 0;
        }
        double accelerationX = acceleration * Math.sin(theta/57.296);
        double accelerationY = acceleration * Math.cos(theta/57.296);

        if(accelerationX < 2 && accelerationX > -2){
            accelerationX = 0;
        }
        if(accelerationY < 2 && accelerationX > -2){
            accelerationY = 0;
        }

        double velocityX = accelerationX * runTime.seconds();
        double velocityY = accelerationY * runTime.seconds();
        Log.v("BOK", "accelX: " + accelerationX + " accelY: " + accelerationY + " theta: " + theta);
        double x = last_point.x + last_point.velocityX*runTime.seconds() + (0.5*(accelerationX)*Math.pow(runTime.seconds(), 2));
        double y = last_point.y + last_point.velocityY*runTime.seconds() + (0.5*(accelerationY)*Math.pow(runTime.seconds(), 2));
        Log.v("BOK", "\n X: " + x + " Y: " + y);
        runTime.reset();
        return new CCPoint(x, y, theta, velocityX, velocityY, accelerationX, accelerationY, acceleration);
    }
    private void moveRobot() {
        robot.moveRobotTele(speedCoef);
    }

    public enum BoKTeleStatus {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }
}
