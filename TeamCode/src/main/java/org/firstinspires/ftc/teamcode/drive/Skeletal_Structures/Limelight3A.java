package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;

public class Limelight3A {

    public static com.qualcomm.hardware.limelightvision.Limelight3A limelight;
    Pose3D botPose;
    ArtifactControl AC;
    public void Init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        //limelight.updateRobotOrientation(Math.toDegrees(AC.drive.getHeading()));

        LLResult result = limelight.getLatestResult();
        if(result.isValid()){
            botPose = result.getBotpose();
        }
    }
    public static double pedroX = 0.0;
    public static double pedroY = 0.0;
    public static double heading = 0.0;

    public static void Update(){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid()){
            Pose3D pose = result.getBotpose();
            if(pose!=null){
                double xMetres = pose.getPosition().x;
                double yMetres = pose.getPosition().y;
                double xInches = xMetres * 39.3701;
                double yInches = yMetres * 39.3701;

                xInches +=72.0;
                yInches +=72.0;

                pedroX = 144.0;
                pedroY = xInches;

                double rawYaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                heading = (rawYaw < 0) ? rawYaw + 360.0 : rawYaw;


            }
        }
    }
    public LLResult getLLResult(){
         return limelight.getLatestResult();
    }

    public void updateRobotOrientationCustom(Follower f){
        limelight.updateRobotOrientation(Math.toDegrees(f.getHeading()));
    }

    public double getLimelightX(){
        // attempt to refresh pose if not yet available
        if (botPose == null && limelight != null) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                botPose = r.getBotpose();
            }
        }
        if (botPose == null || botPose.getPosition() == null) {
            return Double.NaN;
        }
        return botPose.getPosition().x;
    }

    public  double getLimelightY(){
        if (botPose == null && limelight != null) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                botPose = r.getBotpose();
            }
        }
        if (botPose == null || botPose.getPosition() == null) {
            return Double.NaN;
        }
        return  botPose.getPosition().y;
    }

    public double getLimelightYaw(){
        if (botPose == null && limelight != null) {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                botPose = r.getBotpose();
            }
        }
        if (botPose == null || botPose.getOrientation() == null) {
            return Double.NaN;
        }
        double LimelightYaw = botPose.getOrientation().getYaw();
        if(LimelightYaw < 0){
            LimelightYaw = 360 - Math.abs(LimelightYaw);
        }
        return LimelightYaw;
    }



}
