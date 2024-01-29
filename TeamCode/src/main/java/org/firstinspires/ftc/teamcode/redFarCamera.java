package org.firstinspires.ftc.teamcode;

public class redFarCamera extends redAllianceCamera {
    @Override
    public void center() {

    }

    @Override
    public void left() {

    }

    @Override
    public void right() {
        driveOnHeading(18,0.3,0);
    }
}
