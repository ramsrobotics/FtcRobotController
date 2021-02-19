package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CC Auto BLUE Stone Outside", group = "CCAutoBlue")

@Disabled
public class CCAutoBlueStoneOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoBlueStoneOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
