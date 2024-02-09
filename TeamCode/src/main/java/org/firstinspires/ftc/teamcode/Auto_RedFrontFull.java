package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
@Autonomous(name = "Red Frontc", group = "CenterStage", preselectTeleOp = "Full")
public class Auto_RedFrontFull extends CSBase {
    @Override
    public void runOpMode() {
        color teamColor = color.r;
        stageSide = side.f;
        setup(teamColor);

        // ---------------------
        // ------Main Code------
        // ---------------------

        s(1);
        pos = findPos();
        int ID = setID(pos, teamColor);
        telemetry.addData("Team Prop X", x);
        telemetry.addData("Team Prop Position", pos);
        telemetry.update();
        purplePixel();
        drive(-2);
        turn(-90);
        s(3);
        drive(70);
        // startStrafe(dir.l);
        // while(opModeIsActive() && !detectTag(ID)) { continue; }
        // endStrafe();
        // align(ID);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel();
        drive(5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }

}
