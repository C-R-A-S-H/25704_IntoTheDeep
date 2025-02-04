package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Regionals_Sensor_Auto")
public class Regionals_Sensor_Auto extends LinearOpMode {
    private static final double TARGET_DISTANCE = 13.8; // CM from wall
    private static final double motorAutonomousSpeed = 0.4;  //Base speed for auto.  This is further modified in WallFollowAssist::driveToTargetDistance
    
    
    
    private DistanceSensor LEFT_DS, RIGHT_DS;
    private WallFollowAssist wallFollowAssist;
    private DcMotor FL_Motor, FR_Motor, BL_Motor, BR_Motor, Arm;
    private Servo Claw;


  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    int Time;
    
    FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
    FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
    BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
    BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
    Arm = hardwareMap.get(DcMotor.class, "Arm");
    Claw = hardwareMap.get(Servo.class, "Claw");
    LEFT_DS = hardwareMap.get(DistanceSensor.class, "LEFT_DS");
    RIGHT_DS = hardwareMap.get(DistanceSensor.class, "RIGHT_DS");
    
    //Create instance of Wall Following Assist from wallFollowAssist.java
    wallFollowAssist = new WallFollowAssist(LEFT_DS, RIGHT_DS, 
        FL_Motor, FR_Motor, BL_Motor, BR_Motor);    

    BL_Motor.setDirection(DcMotor.Direction.REVERSE);
    FL_Motor.setDirection(DcMotor.Direction.REVERSE);
    BR_Motor.setDirection(DcMotor.Direction.FORWARD);
    FR_Motor.setDirection(DcMotor.Direction.FORWARD);

    BR_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BL_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FR_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    
    FL_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // Put initialization blocks here.
    waitForStart();
    // Time = 0;
    
    if (opModeIsActive()) {
      // Put run blocks here.
      Claw.setPosition(1);  //close the claw
      while (opModeIsActive()) {
        // Put loop blocks here.
           
        
        
        // arm up
        Claw.setPosition(1);  //close claw
        Arm.setPower(0.5);
        sleep(2400);
          // forward
        Arm.setPower(0);
 
       // sleep(500); 
        
        //turn left
        FL_Motor.setPower(-0.4);
        FR_Motor.setPower(0.44);
        BR_Motor.setPower(0.44);
        BL_Motor.setPower(-0.4);
          
        sleep(200);
        // go forward
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(0.44);
        BR_Motor.setPower(0.44);
        BL_Motor.setPower(0.4);
        sleep(900);
        // stop
        FL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
        BL_Motor.setPower(0);
      //  sleep(250);
  
         // move towards the submersible
        wallFollowAssist.driveToTargetDistance(TARGET_DISTANCE, motorAutonomousSpeed, telemetry);
          
        sleep(300);
        
        //realign distance
      //  wallFollowAssist.driveToTargetDistance(TARGET_DISTANCE, motorAutonomousSpeed, telemetry);
          
        // go forward
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(0.44);
        BR_Motor.setPower(0.44);
        BL_Motor.setPower(0.4);
        sleep(200);
        // stop
        FL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
        BL_Motor.setPower(0);
   
        //arm down
       Arm.setPower(-0.4);
       sleep(250);
    
        
        // open claw
        sleep(350);
        Arm.setPower(0);
        Claw.setPosition(-1);
        sleep(1200);
        
         // realign robot
        wallFollowAssist.driveToTargetDistance(TARGET_DISTANCE, 0.4, telemetry);
          
        sleep(300);
        
        //back-up
        FL_Motor.setPower(-0.4);
        FR_Motor.setPower(-0.44);
        BR_Motor.setPower(-0.44);
        BL_Motor.setPower(-0.4);
          
        sleep(250);
          
        // stop
        Arm.setPower(-0.4);
        FL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
        BL_Motor.setPower(0);
          
        sleep(500);
        
        // stop arm
        Arm.setPower(0);
        
        //strafe right
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(-0.47);
        BR_Motor.setPower(0.47);
        BL_Motor.setPower(-0.4);
          
        sleep(1800);
        
        // forward
          Arm.setPower(0);
          FL_Motor.setPower(0.6);
          FR_Motor.setPower(0.6);
          BR_Motor.setPower(0.6);
          BL_Motor.setPower(0.6);
          sleep(800);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          // move right
          FL_Motor.setPower(0.4);
          FR_Motor.setPower(-0.4);
          BR_Motor.setPower(0.4);
          BL_Motor.setPower(-0.4);
          
          sleep(550);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          
          
          // back up
          FL_Motor.setPower(-0.6);
          FR_Motor.setPower(-0.6);
          BR_Motor.setPower(-0.6);
          BL_Motor.setPower(-0.6);
          
          sleep(1400);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          // forward
          Arm.setPower(0);
          FL_Motor.setPower(0.6);
          FR_Motor.setPower(0.6);
          BR_Motor.setPower(0.6);
          BL_Motor.setPower(0.6);
          sleep(1320);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          // move right
          FL_Motor.setPower(0.4);
          FR_Motor.setPower(-0.4);
          BR_Motor.setPower(0.4);
          BL_Motor.setPower(-0.4);
          
          sleep(950);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          // turn
          FL_Motor.setPower(0.4);
          FR_Motor.setPower(-0.4);
          BR_Motor.setPower(-0.4);
          BL_Motor.setPower(0.4);
          
          sleep(100);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          
          // back up
          FL_Motor.setPower(-0.6);
          FR_Motor.setPower(-0.6);
          BR_Motor.setPower(-0.6);
          BL_Motor.setPower(-0.6);
          
          sleep(1600);
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
        
        // forward
          Arm.setPower(0);
          FL_Motor.setPower(0.6);
          FR_Motor.setPower(0.6);
          BR_Motor.setPower(0.6);
          BL_Motor.setPower(0.6);
          sleep(1400);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          // move right
          FL_Motor.setPower(0.4);
          FR_Motor.setPower(-0.4);
          BR_Motor.setPower(0.4);
          BL_Motor.setPower(-0.4);
          
          sleep(950);
          
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          // back up
          FL_Motor.setPower(-0.6);
          FR_Motor.setPower(-0.6);
          BR_Motor.setPower(-0.6);
          BL_Motor.setPower(-0.6);
          
          sleep(1600);
          // stop
          FL_Motor.setPower(0);
          FR_Motor.setPower(0);
          BR_Motor.setPower(0);
          BL_Motor.setPower(0);
          
          sleep(100);
          
          //strafe left
         FL_Motor.setPower(-0.4);
         FR_Motor.setPower(0.4);
         BR_Motor.setPower(-0.4);
         BL_Motor.setPower(0.4);
          
         sleep(600);
         
         // forward
        Arm.setPower(0);
        FL_Motor.setPower(0.6);
        FR_Motor.setPower(0.6);
        BR_Motor.setPower(0.6);
        BL_Motor.setPower(0.6);     
        
        sleep(100);
        
         // turn
        FL_Motor.setPower(0.4);
        FR_Motor.setPower(-0.4);
        BR_Motor.setPower(-0.4);
        BL_Motor.setPower(0.4);
          
        sleep(2500);
        
        break;
          }
        }
      }
    }
