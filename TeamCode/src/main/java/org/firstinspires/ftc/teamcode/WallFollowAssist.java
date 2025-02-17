package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WallFollowAssist {
    // Constants for rotation - maintaining aggressive response
    private static final double STATIONARY_KP = 0.2;  
    private static final double PARALLEL_THRESHOLD = 0.3;  // Reduced from 0.5 for finer control
    private static final double MIN_ROTATION_POWER = 0.15;  // Reduced for smoother low-error correction
    private static final double MAX_ROTATION_POWER = 0.9;  
    
    // Fine adjustment thresholds
    private static final double FINE_ADJUST_THRESHOLD = 0.8; // cm
    private static final double FINE_ADJUST_KP = 0.15; // Proportional control for fine adjustments
    private static final double FINE_ADJUST_MIN_POWER = 0.12; // Minimum power to ensure movement
    
    // Constants for wall following during movement
    private static final double BASE_KP = 0.15;
    private static final double CLOSE_RANGE_KP = 0.3;
    private static final double CLOSE_RANGE_THRESHOLD = 30.0;
    private static final double MAX_CORRECTION = 1.0;
    private static final double DISTANCE_TOLERANCE = 1.0; // CM tolerance for target distance

    // Hardware objects
    private DistanceSensor leftSensor;
    private DistanceSensor rightSensor;
    private DcMotor FL_Motor, FR_Motor, BL_Motor, BR_Motor;
    
    private static final double DISTANCE_KP = 0.1; // Adjust this value as needed

    public WallFollowAssist(DistanceSensor leftSensor, DistanceSensor rightSensor,
                           DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
        this.FL_Motor = fl;
        this.FR_Motor = fr;
        this.BL_Motor = bl;
        this.BR_Motor = br;
    }

    public void applyWallAssist(double y, double x, double rx, double speed) {
        double leftDistance = leftSensor.getDistance(DistanceUnit.CM);
        double rightDistance = rightSensor.getDistance(DistanceUnit.CM);
        double difference = leftDistance - rightDistance;
        
        boolean isStationary = Math.abs(y) < 0.1 && Math.abs(x) < 0.1 && Math.abs(rx) < 0.1;
        
        if (isStationary && Math.abs(difference) > PARALLEL_THRESHOLD) {
            double rotationPower;
            
            // Different power calculations based on error magnitude
            if (Math.abs(difference) > FINE_ADJUST_THRESHOLD) {
                // Normal aggressive correction for larger errors
                rotationPower = difference * STATIONARY_KP;
                rotationPower = Math.copySign(
                    Math.max(MIN_ROTATION_POWER, 
                    Math.min(MAX_ROTATION_POWER, Math.abs(rotationPower))),
                    rotationPower);
            } else {
                // Fine adjustment mode for smaller errors
                rotationPower = difference * FINE_ADJUST_KP;
                rotationPower = Math.copySign(
                    Math.max(FINE_ADJUST_MIN_POWER, 
                    Math.min(MIN_ROTATION_POWER, Math.abs(rotationPower))),
                    rotationPower);
            }

            // Apply rotation based on direction needed
            if (difference > 0) {
                // CCW position, need CW rotation
                BR_Motor.setPower(-Math.abs(rotationPower) * speed);
                BL_Motor.setPower(Math.abs(rotationPower) * speed);
                FL_Motor.setPower(Math.abs(rotationPower) * speed);
                FR_Motor.setPower(-Math.abs(rotationPower) * speed);
            } else {
                // CW position, need CCW rotation
                BR_Motor.setPower(Math.abs(rotationPower) * speed);
                BL_Motor.setPower(-Math.abs(rotationPower) * speed);
                FL_Motor.setPower(-Math.abs(rotationPower) * speed);
                FR_Motor.setPower(Math.abs(rotationPower) * speed);
            }
        } else if (!isStationary) {
            // Normal driving with wall following
            double averageDistance = (leftDistance + rightDistance) / 2.0;
            
            double correction = difference * BASE_KP;
            if (averageDistance < CLOSE_RANGE_THRESHOLD) {
                correction *= (CLOSE_RANGE_KP / BASE_KP);
            }
            
            correction = Math.min(Math.max(-MAX_CORRECTION, correction), MAX_CORRECTION);
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            
            if (Math.abs(y) > Math.abs(x)) {
                frontLeftPower += correction;
                backLeftPower += correction;
                frontRightPower -= correction;
                backRightPower -= correction;
            }
            
            frontLeftPower = Math.min(Math.max(-1.0, frontLeftPower), 1.0);
            backLeftPower = Math.min(Math.max(-1.0, backLeftPower), 1.0);
            frontRightPower = Math.min(Math.max(-1.0, frontRightPower), 1.0);
            backRightPower = Math.min(Math.max(-1.0, backRightPower), 1.0);
            
            FL_Motor.setPower(frontLeftPower * speed);
            BL_Motor.setPower(backLeftPower * speed);
            FR_Motor.setPower(frontRightPower * speed);
            BR_Motor.setPower(backRightPower * speed);
        } else {
            stopMotors();
        }
    }

    public void stopMotors() {
        FL_Motor.setPower(0);
        BL_Motor.setPower(0);
        FR_Motor.setPower(0);
        BR_Motor.setPower(0);
    }

    public String getDebugData() {
        double leftDistance = leftSensor.getDistance(DistanceUnit.CM);
        double rightDistance = rightSensor.getDistance(DistanceUnit.CM);
        double difference = leftDistance - rightDistance;
        
        String mode = "NORMAL";
        if (Math.abs(difference) <= PARALLEL_THRESHOLD) {
            mode = "PARALLEL";
        } else if (Math.abs(difference) <= FINE_ADJUST_THRESHOLD) {
            mode = "FINE";
        }
        
        return String.format("L: %.1f cm, R: %.1f cm, Diff: %.1f cm, Mode: %s", 
            leftDistance, rightDistance, difference, mode);
    }
    
    //This is the part we are adding to do the drive up autonomous to the tank
    //Call driveToDistance, and the first variable is the distance we need the robot to drive
    //up to the tank (17.8cm?), and then the second is a multiplier (0-1) on max motor speed
    public void driveToDistance(double targetDistance, double speedFactor) {
    // Validate inputs
    speedFactor = Math.min(Math.max(0.1, speedFactor), 1.0);
    
    // Get current distances
    double leftDistance = leftSensor.getDistance(DistanceUnit.CM);
    double rightDistance = rightSensor.getDistance(DistanceUnit.CM);
    double averageDistance = (leftDistance + rightDistance) / 2.0;
    double difference = leftDistance - rightDistance;
    
    // Calculate base power
    double distanceRemaining = averageDistance - targetDistance;
    double basePower;
    
    // Progressive slowdown as we approach the target distance
    if (Math.abs(distanceRemaining) > 0) {
        if (Math.abs(distanceRemaining) < 5.0) {
            // Very close to target (under 5cm) - very slow approach
            basePower = speedFactor * 0.2; // Only 20% of requested speed
        }
        else if (Math.abs(distanceRemaining) < 10.0) {
            // Somewhat close to target (5-10cm) - slow approach
            basePower = speedFactor * 0.4; // 40% of requested speed
        }
        else if (Math.abs(distanceRemaining) < 20.0) {
            // Approaching target (10-20cm) - moderate speed
            basePower = speedFactor * 0.6; // 60% of requested speed
        }
        else {
            // Far from target - full requested speed
            basePower = speedFactor;
        }
        
        // Ensure minimum power to keep moving
        basePower = Math.max(0.15, basePower);
        
        // Determine direction
        if (distanceRemaining < 0) {  //uh oh!  We went too far.  Back up!
            // Need to move backward
            basePower = -basePower;
        }
        
        // Calculate wall following correction (reduced strength)
        double correction = difference * (BASE_KP * 0.5); // Reduced correction factor
        correction = Math.min(Math.max(-MAX_CORRECTION * 0.5, correction), MAX_CORRECTION * 0.5);
        
        // Since FL and BL are REVERSE, positive power makes them go forward
        // Since FR and BR are FORWARD, positive power makes them go forward
        // Therefore, for forward motion, we want:
        // FL and BL: positive power (they're reversed)
        // FR and BR: positive power
        double leftPower = basePower + correction;
        double rightPower = basePower - correction;
        
        // Apply power limits
        leftPower = Math.min(Math.max(-speedFactor, leftPower), speedFactor);
        rightPower = Math.min(Math.max(-speedFactor, rightPower), speedFactor);
        
        // Set motor powers
        FL_Motor.setPower(leftPower);   // REVERSE direction
        BL_Motor.setPower(leftPower);   // REVERSE direction
        FR_Motor.setPower(rightPower);  // FORWARD direction
        BR_Motor.setPower(rightPower);  // FORWARD direction
        
    } else {
        stopMotors();
    }
 }

public void driveToTargetDistance(double targetDistance, double speedFactor, Telemetry telemetry) {
    boolean isAtTarget = false;
    
    while (!isAtTarget && !Thread.currentThread().isInterrupted()) {
        // Call driveToDistance to update motor powers
        driveToDistance(targetDistance, speedFactor);
        
        // Check if we've reached target
        isAtTarget = isAtTargetDistance(targetDistance, DISTANCE_TOLERANCE);
        
        // Update telemetry
        if (telemetry != null) {
            telemetry.addData("Auto Distance", "ACTIVE");
            telemetry.addData("Target Distance", targetDistance);
            telemetry.addData("Current Left", leftSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Current Right", rightSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Difference", 
                leftSensor.getDistance(DistanceUnit.CM) - rightSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Motor Powers", String.format("FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                FL_Motor.getPower(), FR_Motor.getPower(), BL_Motor.getPower(), BR_Motor.getPower()));
            telemetry.update();
        }
    }
    
    // Stop motors once target is reached
    stopMotors();
}

    // Helper method to check if we've reached the target distance
    public boolean isAtTargetDistance(double targetDistance, double tolerance) {
        double leftDistance = leftSensor.getDistance(DistanceUnit.CM);
        double rightDistance = rightSensor.getDistance(DistanceUnit.CM);
        double averageDistance = (leftDistance + rightDistance) / 2.0;
        
        return Math.abs(averageDistance - targetDistance) <= tolerance;
    }

    // Enhanced debug data to include distance-to-target information
    public String getDebugDataWithTarget(double targetDistance) {
        double leftDistance = leftSensor.getDistance(DistanceUnit.CM);
        double rightDistance = rightSensor.getDistance(DistanceUnit.CM);
        double difference = leftDistance - rightDistance;
        double averageDistance = (leftDistance + rightDistance) / 2.0;
        double distanceToTarget = averageDistance - targetDistance;
        
        return String.format("L: %.1f cm, R: %.1f cm, Diff: %.1f cm, To Target: %.1f cm", 
            leftDistance, rightDistance, difference, distanceToTarget);
    }
    
}
