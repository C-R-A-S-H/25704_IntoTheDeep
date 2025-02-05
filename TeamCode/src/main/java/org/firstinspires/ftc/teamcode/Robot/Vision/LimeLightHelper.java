package org.firstinspires.ftc.teamcode.Robot.Vision;


import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
;

import java.util.ArrayList;
import java.util.List;


public class LimeLightHelper implements Subsystem {
    private Limelight3A ll;
    private double LLAngle = 0;
    private double SamepleHeight = 1.5;
    private double LLHeight = 0;
    private Pose2d lastPose;

    public LimeLightHelper(HardwareMap hardwareMap){
        ll = hardwareMap.get(Limelight3A.class,"ll");
    }
    private double heading;

    private PIDController LignUpPid = new PIDController(0.017,0,0);


    public void PipelineSwitch(int index) {
        this.ll.pipelineSwitch(index);
    }
    //public Pose3D GetPosition(){
   //   return this.ll.getLatestResult().getBotpose();
   // }

    public Limelight3A getLL(){
        return this.ll;
    }
    public List<List<Double>> getColorData(){
        LLResult data = this.ll.getLatestResult();
        List<List<Double>> colorData = new ArrayList<>();
        List<LLResultTypes.ColorResult> colorResults = data.getColorResults();
        if(!colorResults.isEmpty()) {
            for (LLResultTypes.ColorResult cr : colorResults) {
                List<Double> SigmaList = new ArrayList<>();
                SigmaList.add(cr.getTargetXDegrees());
                SigmaList.add(cr.getTargetYDegrees());
                SigmaList.add(cr.getTargetArea());
                colorData.add(SigmaList);
            }
        }
        return  colorData;
    }

    public double lookAtSample() {
        double power = 0;
        LLResult result = this.ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                power = LignUpPid.calculate(0, colorResults.get(0).getTargetXDegrees());

            }

        }
        return power;
    }

    public boolean SampleInSight(){
        LLResult result = this.ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return true;


            }

        }
        return false;
    }

    public boolean LookingAtSample(double powerVal){
        LLResult result = this.ll.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                return tolerance(colorResults.get(0).getTargetXDegrees(), -0.1,0.1);


            }

        }
        return false;
    }
    private boolean tolerance(double value,double min,double max){
        return value >= min && value <= max;
    }
    public double getDistanceFromSample(List<Double> sample){
        double offset = sample.get(1);

        double angleToGoalDegrees = LLAngle + offset;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double distanceFromLimelightToGoalInches = (SamepleHeight - LLHeight) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;

    }

    public boolean checkForValidTags(){
        LLResult results = this.ll.getLatestResult();
        if (results.isValid()){
            return results.getBotposeTagCount() >= 1;
        }
       return false;
    }

    public void updateHeading(double heading){
        this.ll.updateRobotOrientation(heading);
        this.heading = heading;
    }

    public Pose2d getPose(){

        LLResult results = this.ll.getLatestResult();
        if (results.isValid()) {
            Pose3D pose = results.getBotpose_MT2();
            lastPose = new  Pose2d(
                    pose.getPosition().x,
                    pose.getPosition().y,
                    new Rotation2d(this.heading)
            );
            return lastPose;
        }
        return lastPose;
    }


    public void init() {
        this.ll.start();
        this.ll.pipelineSwitch(0);
    }

    @Override
    public void periodic() {

    }


}
