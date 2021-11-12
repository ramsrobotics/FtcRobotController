package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto Blue Inside", group = "CCAutoBlue")
public class CCAutoBlueInsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoBlueInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
