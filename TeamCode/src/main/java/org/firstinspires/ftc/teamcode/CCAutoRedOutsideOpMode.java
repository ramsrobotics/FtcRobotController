package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto Red Outside", group = "CCAutoRed")
public class CCAutoRedOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoRedOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
