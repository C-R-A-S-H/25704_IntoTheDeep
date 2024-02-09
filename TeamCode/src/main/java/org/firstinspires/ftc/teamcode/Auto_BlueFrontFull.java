package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Blue Front Full", group = "CenterStage", preselectTeleOp = "Full")
//@Disabled
public class Auto_BlueFrontFull extends CSBase {
    @Override
    public void runOpMode() {
        stageSide = side.f;
        setup(color.b);

        // ---------------------
        // ------Main Code------
        // ---------------------


        findPos();
        purplePixel();
        drive(-2);
        turn(90);
        s(3);
        drive(70);
        setSpeed(1000);
        drive(15);
        setSpeed(2000);
        ejectPixel();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        s(1);  // Pause to display final telemetry message.
    }


}
