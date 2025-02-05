package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Bucket_Auto")
public class Bucket_Auto extends LinearOpMode {
    private static final double motorAutonomousSpeed = 0.4;
    
    private DcMotor FL_Motor, FR_Motor, BL_Motor, BR_Motor, Arm;
    private Servo Intake;
    private DcMotorEx IL_L, IL_R;
    
  @Override
  public void runOpMode() {
    int Time;
    
    FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
    FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
    BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
    BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
    Arm = hardwareMap.get(DcMotor.class, "Arm");
    Intake = hardwareMap.get(Servo.class, "Intake");
    IL_L = hardwareMap.get(DcMotorEx.class, "Intake_Lift_Left");
    IL_R = hardwareMap.get(DcMotorEx.class, "Intake_Lift_Right");
    
    
    //Create instance of Wall Following Assist from wallFollowAssist.java
    

    BL_Motor.setDirection(DcMotor.Direction.REVERSE);
    FL_Motor.setDirection(DcMotor.Direction.REVERSE);
    BR_Motor.setDirection(DcMotor.Direction.FORWARD);
    FR_Motor.setDirection(DcMotor.Direction.FORWARD);

    BR_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BL_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FR_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    
    FL_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    IL_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    IL_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // Put initialization blocks here.
    waitForStart();
    // Time = 0;
    
    if (opModeIsActive()) {
      Intake.setPosition(1);
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // lift arm
        Arm.setPower(0.5);
        sleep(3600);
          // lift intake
        Arm.setPower(0);
        
        IL_L.setPower(0.6);
        IL_R.setPower(-0.6);
        sleep(950);
          // forward
        IL_L.setPower(0);
        IL_R.setPower(0);
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(0.44);
        BR_Motor.setPower(0.44);
        BL_Motor.setPower(0.4);
        sleep(1250);
        
         //turn left
        FL_Motor.setPower(-0.4);
        FR_Motor.setPower(0.44);
        BR_Motor.setPower(0.44);
        BL_Motor.setPower(-0.4);
          
        sleep(505);
        
        // stop
        FL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
        BL_Motor.setPower(0);
        
        //position sample
        IL_L.setPower(0.6);
        IL_R.setPower(-0.6);
        sleep(450);
        
        //Score first sample
        Intake.setPosition(-1);
        sleep(400);
        
        //Move intake back
        IL_L.setPower(-0.6);
        IL_R.setPower(0.6);
        sleep(450);
        
        //back up
        FL_Motor.setPower(-0.4);
        FR_Motor.setPower(-0.44);
        BR_Motor.setPower(-0.44);
        BL_Motor.setPower(-0.4);
        sleep(500);
        
        FL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
        BL_Motor.setPower(0);
        
        //arm down/rotate
        Arm.setPower(-0.5);
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(-0.44);
        BR_Motor.setPower(-0.44);
        BL_Motor.setPower(0.4);
          
        sleep(1300);
        
        //forward
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(0.44);
        BR_Motor.setPower(0.44);
        BL_Motor.setPower(0.4);
        sleep(200);
        
        FL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
        BL_Motor.setPower(0);
        
        //clamp 2nd sample
        IL_L.setPower(0.6);
        IL_R.setPower(-0.6);
        sleep(600);
        Intake.setPosition(1);
        sleep(200);
        
        //move arm up
        Arm.setPower(0.5);
        sleep(3600);
        Arm.setPower(0);
        
        sleep(99999999);
        //break;


}
}
}  
}
