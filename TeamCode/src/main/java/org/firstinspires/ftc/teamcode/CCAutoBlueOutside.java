package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class CCAutoBlueOutside extends CCAutoCommon
{
    public CCAutoBlueOutside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(false);
    }
}
