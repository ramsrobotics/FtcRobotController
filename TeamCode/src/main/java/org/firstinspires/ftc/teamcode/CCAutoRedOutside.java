package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class CCAutoRedOutside extends CCAutoCommon
{
    public CCAutoRedOutside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() { runAuto(false); }
}
