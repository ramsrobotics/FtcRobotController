package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class CCAutoBlueInside extends CCAutoCommon
{
    public CCAutoBlueInside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(true);
    }
}
