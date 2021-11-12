package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class CCAutoRedInside extends CCAutoCommon
{
    public CCAutoRedInside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() {
        runAuto(true);
    }
}
