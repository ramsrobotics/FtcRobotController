package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto Blue Outside", group = "CCAutoBlue")
public class CCAutoBlueOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoBlueOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
