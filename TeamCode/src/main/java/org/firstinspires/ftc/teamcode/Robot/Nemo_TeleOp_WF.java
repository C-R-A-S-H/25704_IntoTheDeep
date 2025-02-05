package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Nemo_TeleOp_WF")
public class Nemo_TeleOp_WF extends LinearOpMode {

    private DcMotor BR_Motor, BL_Motor, FL_Motor, FR_Motor;
    private DcMotorEx IL_L, IL_R, Arm;
    private Servo Claw, Intake;
    private DistanceSensor LEFT_DS, RIGHT_DS;
    private WallFollowAssist wallFollowAssist;
    
    //These are the values for the prepositioning of the claw/intake arm for clipping and picking operations
    private static int ARM_PICKUP_TARGET_POSITION = 1200;    // Adjust this value based on testing
    private static int IL_L_PICKUP_TARGET_POSITION = 0;      // Adjust this value based on testing  Make sure this matches Right
    private static int IL_R_PICKUP_TARGET_POSITION = 0;      // Adjust this value based on testing  Make sure this mathes left
    private static int ARM_CLIPPING_TARGET_POSITION = 2940;    // Adjust this value based on testing
    private static int IL_L_CLIPPING_TARGET_POSITION = 0;      // Adjust this value based on testing  Make sure this matches Right
    private static int IL_R_CLIPPING_TARGET_POSITION = 0;      // Adjust this value based on testing  Make sure this matches Left
    
    private boolean isInPositionMode = false;           //This means the motors are busy moving the arm
    private static final double STICK_DEADZONE = 0.1;   //This is hysteresis of the Gamepad2 control sticks
    private int armMinPosition = Integer.MAX_VALUE;     //This sets up the lowest value of the Arm motor encoder
    private int il_LMinPosition = Integer.MAX_VALUE;      //This sets up the lowest value of the Intake_Lift motor encoder
    private int il_RMinPosition = Integer.MAX_VALUE;      //This sets up the lowest value of the Intake_Lift motor encoder
    private static final double MOVE_POWER = 1.0;       //Power to run the motors at when just moving
    private static final int OVERSHOOT_UP = 15;         //How many encoder steps to overshoot by to compensate for gravity
    private static final int UNDERSHOOT_DOWN = 8;       //How many steps to slow down before reaching the target when coming down
    private static final int POSITION_TIMEOUT_MS = 1000; // 1 second timeout
    private static final double POSITION_TOLERANCE = 20; // encoder ticks tolerance
    private long positionModeStartTime = 0;              //timer to timeout the prepositioning routines    
    
    //Values for motor speeds under various conditions
    private static double motorNormalSpeed = 2;  //normal driving speed multiplier
    private static double motorSlowSpeed = 0.4;  //when pressing left trigger, this is the multiplier for the speed
    private static double motorWFSpeed = 0.4;  //this is the speed that the Wall Follow Assist routine will use for motor speed
    private static double motorARMSpeed = 1.25;  //this is the speed that the ARM motor will use for motor speed
    private static double motorILSpeed = 1.25;  //this is the speed that the Intake Lift (both) motors will use for motor speed

    @Override
    public void runOpMode() {
 // Hardware mapping initialization
        BR_Motor = hardwareMap.get(DcMotor.class, "BR_Motor");
        BL_Motor = hardwareMap.get(DcMotor.class, "BL_Motor");
        FL_Motor = hardwareMap.get(DcMotor.class, "FL_Motor");
        FR_Motor = hardwareMap.get(DcMotor.class, "FR_Motor");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Intake = hardwareMap.get(Servo.class, "Intake");
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        IL_L = hardwareMap.get(DcMotorEx.class, "Intake_Lift_Left");
        IL_R = hardwareMap.get(DcMotorEx.class, "Intake_Lift_Right");
        LEFT_DS = hardwareMap.get(DistanceSensor.class, "LEFT_DS");
        RIGHT_DS = hardwareMap.get(DistanceSensor.class, "RIGHT_DS");
        wallFollowAssist = new WallFollowAssist(LEFT_DS, RIGHT_DS, 
            FL_Motor, FR_Motor, BL_Motor, BR_Motor);    

        // Setup the arm motors
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        IL_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        IL_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IL_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IL_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            BL_Motor.setDirection(DcMotor.Direction.REVERSE);
            FL_Motor.setDirection(DcMotor.Direction.REVERSE);
            IL_R.setDirection(DcMotor.Direction.REVERSE);  //Need this because the motor is installed backwards

            while (opModeIsActive()) {
                telemetry.update();

                // Get current positions once for all uses
                int currentArmPos = Arm.getCurrentPosition();
//                int currentIL_LPos = IL_L.getCurrentPosition();
//                int currentIL_RPos = IL_R.getCurrentPosition();

                // Update minimum positions
                if (currentArmPos < armMinPosition) {
                    armMinPosition = currentArmPos;
                    ARM_PICKUP_TARGET_POSITION = armMinPosition + 1265;
                }
                
                if (currentArmPos < armMinPosition) {
                    armMinPosition = currentArmPos;
                    ARM_CLIPPING_TARGET_POSITION = armMinPosition + 2940;
                }

/*                
                if (currentIL_LPos < il_LMinPosition) {
                    il_LMinPosition = currentIL_LPos;
                    IL_L_PICKUP_TARGET_POSITION = il_LMinPosition;
                }
                if (currentIL_RPos < il_RMinPosition) {
                    il_RMinPosition = currentIL_RPos;
                    IL_R_PICKUP_TARGET_POSITION = il_RMinPosition;
                }                
*/
  // Gamepad 1 / Driver 1 controls:
                double y = -gamepad1.left_stick_y;  
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);  
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                
                if (gamepad1.left_trigger > 0) {
                    frontLeftPower *= motorSlowSpeed;
                    backLeftPower *= motorSlowSpeed;
                    frontRightPower *= motorSlowSpeed;
                    backRightPower *= motorSlowSpeed;
                    telemetry.addData("Drive Mode", "50% Speed");
                } else {
                    frontLeftPower *= motorNormalSpeed;
                    backLeftPower *= motorNormalSpeed;
                    frontRightPower *= motorNormalSpeed;
                    backRightPower *= motorNormalSpeed;
                    telemetry.addData("Drive Mode", "Full Speed");
                }

                if (gamepad1.x) {
                    ((DcMotorImplEx) BL_Motor).setVelocity(0);
                    ((DcMotorImplEx) BR_Motor).setVelocity(0);
                    ((DcMotorImplEx) FR_Motor).setVelocity(0);
                    ((DcMotorImplEx) FL_Motor).setVelocity(0);
                    ((DcMotorImplEx) Arm).setVelocity(-800);
                }
                
                if (gamepad1.right_trigger > 0) {
                    wallFollowAssist.applyWallAssist(y, x, rx, motorWFSpeed);
                    telemetry.addData("Wall Assist", "ACTIVE");
                    telemetry.addData("Drive Inputs", String.format("y:%.2f x:%.2f rx:%.2f", y, x, rx));
                    telemetry.addData("Sensor Data", wallFollowAssist.getDebugData());
                } else {
                    FL_Motor.setPower(frontLeftPower);
                    BL_Motor.setPower(backLeftPower);
                    FR_Motor.setPower(frontRightPower);
                    BR_Motor.setPower(backRightPower);
                }

   // Gamepad 2 / Driver 2 controls:
                Claw.scaleRange(-1, 1);
                Intake.scaleRange(-1, 1);

                if (gamepad2.a) {
                    Intake.setPosition(1);
                } else if (gamepad2.b) {
                    Intake.setPosition(-1);
                }

                if (gamepad2.x) {
                    Claw.setPosition(1);
                } else if (gamepad2.y) {
                    Claw.setPosition(-1);
                }

                if (gamepad2.left_trigger > 0 && !isInPositionMode) {  //This is the prepositioning code for Gamepad2, left trigger button
                    isInPositionMode = true;
                    positionModeStartTime = System.currentTimeMillis();  //setup a method to exit out after so much time, to prevent the controls from locking up
                    
                    boolean armMovingUp = ARM_PICKUP_TARGET_POSITION > currentArmPos;
//                    boolean il_LMovingUp = IL_L_PICKUP_TARGET_POSITION > currentIL_LPos;
//                    boolean il_RMovingUp = IL_R_PICKUP_TARGET_POSITION > currentIL_RPos;
                    
                    if (armMovingUp) {
                        Arm.setTargetPosition(ARM_PICKUP_TARGET_POSITION + OVERSHOOT_UP);
                    } else {
                        Arm.setTargetPosition(ARM_PICKUP_TARGET_POSITION - UNDERSHOOT_DOWN);
                    }
                    
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(MOVE_POWER);

/*                    
                    if (il_LMovingUp) {
                        IL_L.setTargetPosition(IL_L_PICKUP_TARGET_POSITION + OVERSHOOT_UP);
                    } else {
                        IL_L.setTargetPosition(IL_L_PICKUP_TARGET_POSITION - UNDERSHOOT_DOWN);
                    }
                    if (il_RMovingUp) {
                        IL_R.setTargetPosition(IL_R_PICKUP_TARGET_POSITION + OVERSHOOT_UP);
                    } else {
                        IL_R.setTargetPosition(IL_R_PICKUP_TARGET_POSITION - UNDERSHOOT_DOWN);
                    }
                    
                    IL_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    IL_L.setPower(MOVE_POWER);
                    IL_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    IL_R.setPower(MOVE_POWER);
*/                    
                    telemetry.addData("Arm Direction", armMovingUp ? "UP" : "DOWN");
//                    telemetry.addData("IL Direction", il_LMovingUp ? "UP" : "DOWN");
                }

                if (gamepad2.right_trigger > 0 && !isInPositionMode) {  //This is the prepositioning code for Gamepad2, right trigger button
                    isInPositionMode = true;
                    positionModeStartTime = System.currentTimeMillis();  //setup a method to exit out after so much time, to prevent the controls from locking up
                    
                    boolean armMovingUp = ARM_CLIPPING_TARGET_POSITION > currentArmPos;
//                    boolean il_LMovingUp = IL_L_CLIPPING_TARGET_POSITION > currentIL_LPos;
//                    boolean il_RMovingUp = IL_R_CLIPPING_TARGET_POSITION > currentIL_RPos;
                    
                    if (armMovingUp) {
                        Arm.setTargetPosition(ARM_CLIPPING_TARGET_POSITION + OVERSHOOT_UP);
                    } else {
                        Arm.setTargetPosition(ARM_CLIPPING_TARGET_POSITION - UNDERSHOOT_DOWN);
                    }
                    
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(MOVE_POWER);
 /*                   
                    if (il_LMovingUp) {
                        IL_L.setTargetPosition(IL_L_CLIPPING_TARGET_POSITION + OVERSHOOT_UP);
                    } else {
                        IL_L.setTargetPosition(IL_L_CLIPPING_TARGET_POSITION - UNDERSHOOT_DOWN);
                    }
                    if (il_RMovingUp) {
                        IL_R.setTargetPosition(IL_R_CLIPPING_TARGET_POSITION + OVERSHOOT_UP);
                    } else {
                        IL_R.setTargetPosition(IL_R_CLIPPING_TARGET_POSITION - UNDERSHOOT_DOWN);
                    }
                    
                    IL_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    IL_L.setPower(MOVE_POWER);
                    IL_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    IL_R.setPower(MOVE_POWER);
  */                  
                    telemetry.addData("Arm Direction", armMovingUp ? "UP" : "DOWN");
//                    telemetry.addData("IL_L Direction", il_LMovingUp ? "UP" : "DOWN");
                }

                if (isInPositionMode) {
                    long currentTime = System.currentTimeMillis();
                    boolean timeout = (currentTime - positionModeStartTime) > POSITION_TIMEOUT_MS;
                    
                    // Get target positions (use whichever is active)
                    int armTarget = Arm.getTargetPosition();
//                    int il_LTarget = IL_L.getTargetPosition();
//                    int il_RTarget = IL_R.getTargetPosition();
                    
                    // Check if we're within tolerance of target
                    boolean armAtTarget = Math.abs(currentArmPos - armTarget) < POSITION_TOLERANCE;
//                    boolean il_LAtTarget = Math.abs(currentIL_LPos - il_LTarget) < POSITION_TOLERANCE;
//                    boolean il_RAtTarget = Math.abs(currentIL_RPos - il_RTarget) < POSITION_TOLERANCE;
                    
                    // Get motor velocities
                    double armVelocity = Math.abs(Arm.getVelocity());
//                    double il_LVelocity = Math.abs(IL_L.getVelocity());
//                    double il_RVelocity = Math.abs(IL_R.getVelocity());
                    
                    // Check if motors are stalled (very low velocity while power is applied)
                    boolean armStalled = armVelocity < 10 && Math.abs(Arm.getPower()) > 0.1;
/*                    boolean il_LStalled = il_LVelocity < 10 && Math.abs(IL_L.getPower()) > 0.1;
                    boolean il_RStalled = il_RVelocity < 10 && Math.abs(IL_R.getPower()) > 0.1;
                    
                    if ((!Arm.isBusy() && !IL_L.isBusy() && !IL_R.isBusy()) || // Original condition
                        ((armAtTarget && il_LAtTarget) && il_RAtTarget)||    // New: position reached within tolerance
                        timeout ||                        // New: timeout reached
                        (armStalled && il_LStalled && il_RStalled)) {     // New: motors stalled
*/
                    if ((!Arm.isBusy()) || // Original condition
                        (armAtTarget )||    // New: position reached within tolerance
                        timeout ||                        // New: timeout reached
                        (armStalled )) {     // New: motors stalled
                        
                        // Movement complete - apply brake but maintain encoder values
                        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        IL_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        IL_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        
                        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        IL_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        IL_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        
                        Arm.setPower(0);
                        IL_L.setPower(0);
                        IL_R.setPower(0);
                        
                        isInPositionMode = false;
                        
                        telemetry.addData("Status", "Position held");
                        telemetry.addData("Exit Reason", 
                            timeout ? "Timeout" : 
                            armStalled ? "Motors Stalled" :
                            armAtTarget ? "Position Reached" :
//                            armStalled && il_LStalled ? "Motors Stalled" :
//                            armAtTarget && il_LAtTarget ? "Position Reached" :
                            "Motors Not Busy");
                    } else {
                        telemetry.addData("Status", "Moving to position");
                        telemetry.addData("Arm Error", Math.abs(currentArmPos - armTarget));
//                        telemetry.addData("IL_L Error", Math.abs(currentIL_LPos - il_LTarget));
                        telemetry.addData("Time Elapsed", (currentTime - positionModeStartTime) + "ms");
                    }
                    
                    telemetry.addData("Arm Position", currentArmPos);
//                    telemetry.addData("IL_L Position", currentIL_LPos);
                }

                // Manual control section
                double armPower = -gamepad2.left_stick_y;
                double liftPower = gamepad2.right_stick_y;

                if (Math.abs(armPower) < STICK_DEADZONE) {
                    armPower = 0;
                    if (!isInPositionMode) {
                        Arm.setPower(0);
                    }
                }
                
                if (Math.abs(liftPower) < STICK_DEADZONE) {
                    liftPower = 0;
                    if (!isInPositionMode) {
                        IL_L.setPower(0);
                        IL_R.setPower(0);
                    }
                }

                if (Math.abs(armPower) > STICK_DEADZONE || Math.abs(liftPower) > STICK_DEADZONE) {
                    isInPositionMode = false;
                    
                    if (Arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //                  Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);                        
                    }
/*  
  
                    if (IL_L.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        IL_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        IL_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 //                       IL_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 //                       IL_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);                        
                    }
  */                  
                    if (Math.abs(armPower) > STICK_DEADZONE) {
                        Arm.setPower(armPower * motorARMSpeed);
                    }
                    if (Math.abs(liftPower) > STICK_DEADZONE) {
                        IL_L.setPower(liftPower * motorILSpeed);
                        IL_R.setPower(liftPower * motorILSpeed);
                    }
                    
                    telemetry.addData("Mode", "Manual Control");
                }
                telemetry.addData("Arm Min Position", armMinPosition);
//                telemetry.addData("IL_L Min Position", il_LMinPosition);
                telemetry.addData("Arm Current Position", currentArmPos);
//                telemetry.addData("IL_L Current Position", currentIL_LPos);
                
            }  //end  while (opModeIsActive())
        }  //end if (opModeIsActive())
    } //end public void runOpMode()

}  //end public class Nemo_TeleOp_WF extends LinearOpMode
