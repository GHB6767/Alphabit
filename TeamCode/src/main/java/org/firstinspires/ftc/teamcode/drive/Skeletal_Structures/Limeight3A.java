package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limeight3A{

    public Limelight3A limelight;
    Pose3D botPose;

    public void Init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        LLResult result = limelight.getLatestResult();
        if(result.isValid()){
            botPose = result.getBotpose();
        }
    }

    public double getLimelightX(){
        return botPose.getPosition().x;
    }

    public  double getLimelightY(){
        return  botPose.getPosition().y;
    }

    public double getLimelightYaw(){
        double LimelightYaw = botPose.getOrientation().getYaw();
        if(LimelightYaw < 0){
            LimelightYaw = 360 - Math.abs(LimelightYaw);
        }
        return LimelightYaw;
    }



}
