package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

public class BlueBackstage extends LinearOpMode {

    RobotClass teamBot = new RobotClass(this);

    @Override
    public void runOpMode() throws InterruptedException {
        teamBot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            teamBot.strafing(RobotClass.Direction.LEFT, 0.6, 3000);
        }

        //Strafe to the wall

    }
}
