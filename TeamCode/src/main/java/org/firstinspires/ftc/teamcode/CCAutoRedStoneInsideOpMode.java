package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoRedStoneInside objects
 */
@Autonomous(name = "CC Auto RED Stone Inside", group = "CCAutoRed")
@Disabled
public class CCAutoRedStoneInsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoRedStoneInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}