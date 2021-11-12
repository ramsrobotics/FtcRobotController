package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto Red Inside", group = "CCAutoRed")
public class CCAutoRedInsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoRedInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
