package org.firstinspires.ftc.teamcode;

public class CCAutoOnline extends CCAutoCommon
{
    public CCAutoOnline() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() {
        runAuto(true, false, false);
    }
}
