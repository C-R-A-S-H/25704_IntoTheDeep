package org.firstinspires.ftc.team8923_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueDeliverWarehouse")
public class BlueDeliverWarehouse extends MasterAutonomous {
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        double referenceAngle = imu.getAngularOrientation().firstAngle;
        moveForward(-16.0, 10, 10);
        autoDeliver();
        imuPivot(referenceAngle, -90, 35, 0.015, 3.0);
        moveForward(50.0, 10, 10);
    }
}