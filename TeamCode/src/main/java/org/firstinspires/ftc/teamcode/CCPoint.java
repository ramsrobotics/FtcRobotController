package org.firstinspires.ftc.teamcode;

public class CCPoint {
    double x;
    double y;
    double theta;

    double velocityX;
    double velocityY;

    double accelerationX;
    double accelerationY;
    double acceleration;
    public CCPoint(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    public CCPoint(double x, double y, double theta, double velocityX, double velocityY, double accelerationX, double accelerationY, double acceleration){
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.accelerationX = accelerationX;
        this.accelerationY = accelerationY;
        this.acceleration = acceleration;;
    }

}
