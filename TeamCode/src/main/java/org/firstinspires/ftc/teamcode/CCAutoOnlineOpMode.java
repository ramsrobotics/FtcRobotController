package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto Online", group = "CCAutoRed")
public class CCAutoOnlineOpMode extends CCAutoOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoOnline(); // use interface (polymorphism)
        super.runOpMode();
    }
}
