package org.firstinspires.ftc.teamcode.drive.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Pinpoint;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.drive.Structure.ChasisControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//FTC Decode 2026 TeleOp_Decode
@TeleOp
public class TeleOp_Decode extends LinearOpMode {

    MultipleTelemetry telemetrys;
    //ChasisControl chasis_control;
    ArtifactControl artifactControl;
    //Pinpoint pp = new Pinpoint();
    //Follower follower;
    int failSafeCase = 0;
    boolean toggleButton = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //chasis_control = new ChasisControl(hardwareMap, gamepad1);
        artifactControl = new ArtifactControl(hardwareMap, gamepad2, gamepad1, telemetrys);
        //follower = Constants.createFollower(hardwareMap);
        while(opModeInInit()){
            if(gamepad1.dpad_left){
                if(!toggleButton) {
                    failSafeCase = failSafeCase + 1;
                    toggleButton = true;
                }
            }else if(gamepad1.dpad_up){
                if(!toggleButton) {
                    failSafeCase = failSafeCase + 2;
                    toggleButton = true;
                }
            }else if(gamepad1.dpad_right) {
                if (!toggleButton) {
                    VarStorage.autonomous_case = failSafeCase;
                    toggleButton = true;
                }
            }else{
                toggleButton = false;
            }

            telemetrys.addData("[->] Pattern ", artifactControl.artifactPattern);
            telemetrys.addData("[->] Case selected ", VarStorage.autonomous_case);

            telemetrys.update();
        }

        waitForStart();

        artifactControl.initServo();
        artifactControl.resetYaw();
        artifactControl.initRobotPose();
        //pp.gyroInit(hardwareMap);

        while(opModeIsActive()){
            //chasis_control.Run();
            artifactControl.Run();
            //pp.pinpoint.update();
            //follower.update();

            if(artifactControl.manualControl){
                telemetrys.addData("[->] MANUAL CONTROL ", " ACTIVE [<-]");
                telemetrys.addData(" ", " ");
            }

            telemetrys.addData("[Artifact] Current Left Turret Position ", artifactControl.current_leftturret_position);
            telemetrys.addData("[Artifact] Current Right Turret Position ", artifactControl.current_rightturret_position);
            telemetrys.addData("[Artifact] Current Angle Turret Position ", artifactControl.current_angleturret_position);
            telemetrys.addData("[Artifact] Current Block Position ", artifactControl.artifact_status_blocked);

            telemetrys.addData("[Artifact] Current Heading Angle ", artifactControl.headingAngle);

            //telemetrys.addData("[Pedropathing] Robot Pose X", artifactControl.drive.getPose().getX());
            //telemetrys.addData("[Pedropathing] Robot Pose Y",artifactControl.drive.getPose().getY());

            telemetrys.addData("[Artifact] [RR Pose X] ",artifactControl.rrXPosition);
            telemetrys.addData("[Artifact] [RR Pose Y] ",artifactControl.rrYPosition);

            //telemetrys.addData("[Artifact] [Pinpoint] X Position: ", pp.getx());
            //telemetrys.addData("[Artifact] [Pinpoint] Y Position: ", pp.gety());

            telemetrys.addData("[Artifact] X Position: ", artifactControl.x_position);
            telemetrys.addData("[Artifact] Y Position: ", artifactControl.y_position);


            telemetrys.addData("[Artifact] allowedToShoot ", artifactControl.allowedToShoot);
            telemetrys.addData("[Artifact] Basket angle ", artifactControl.getBasketDirection());
            telemetrys.addData("[Artifact] Basket distance ", artifactControl.getBasketDistance(0,0,false,false));
            telemetrys.addData("[Artifact] Turret position ", artifactControl.getTurretPosition());
            telemetrys.addData("[Artifact] Turret angle ", artifactControl.getTurretAngle());
            telemetrys.addData("[Artifact] FlyWheel Power ", artifactControl.getFlyWheelPower(0,0,false,false));
            telemetrys.addData("[Artifact] Default FlyWheel Power ", artifactControl.defaultFlyWheelPower);
            telemetrys.addData("[Artifact] Pattern ", artifactControl.artifactPattern);
            telemetrys.addData("[Artifact] Left FlyWheel Speed ", artifactControl.leftFlyWheelSpeed);
            telemetrys.addData("[Artifact] Right FlyWheel Speed ", artifactControl.rightFlyWheelSpeed);
            telemetrys.addData("[Artifact] Burst counter ", artifactControl.burstCounter);
            telemetrys.addData("[Artifact] WantsToThrowArtifacts ", artifactControl.wantsToThrowArtifacts);
            telemetrys.addData("[Artifact] Force Activate Intake Counter ", artifactControl.forceActivationOfIntake_counter);
            telemetrys.addData("[Artifact] PushArtifact ", artifactControl.pushArtifact);
            telemetrys.addData("[Artifact] AprilTag Robot Pose X ", artifactControl.calculatedRobotPose_X);
            telemetrys.addData("[Artifact] AprilTag Robot Pose Y ", artifactControl.calculatedRobotPose_Y);
            telemetrys.addData("[Artifact] AprilTag Robot Angle ", artifactControl.robotAngleAprilTag);
            telemetrys.addData("[Artifact] Robot Velocity ", artifactControl.robotVelocity);
            telemetrys.addData("[Artifact] Robot Stationary ", artifactControl.isRobotStationary);
            telemetrys.addData("[Artifact] Automated Pose Reset ", artifactControl.automatedRobotPoseReset);
            telemetrys.addData("[Artifact] Current Target FlyWheel Velocity ", artifactControl.currentTargetFlyWheelVelocity);
            telemetrys.addData("[Artifact] Robot Wants to Auto Shoot ", artifactControl.robotWantsToAutoShoot);
            telemetrys.addData("[Artifact] Robot Wants to Start Intake ", artifactControl.robotWantsToStartIntake);
            telemetrys.addData("[Artifact] leftDistanceSensor Distance ", artifactControl.leftDistanceSensor_distance);
            telemetrys.addData("[Artifact] rightDistanceSensor Distance ", artifactControl.rightDistanceSensor_distance);
            telemetrys.addData("[Artifact] redNorm ", artifactControl.redNorm);
            telemetrys.addData("[Artifact] greenNorm ", artifactControl.greenNorm);
            telemetrys.addData("[Artifact] blueNorm ", artifactControl.blueNorm);
            telemetrys.addData("[Artifact] artifact Counter ", artifactControl.artifactCounter);
            telemetrys.addData("[Artifact] Light intensity ", artifactControl.lightIntensity);
            telemetrys.addData("[Artifact] generalIntakeActivaton ", artifactControl.generalIntakeActivaton);
            telemetrys.addData("[Artifact] Robot Auto Shoot toggle ", artifactControl.robotAutoShootToggle);
            telemetrys.addData("[Artifact] Robot Auto Intake toggle ", artifactControl.robotAutoIntakeToggle);
            telemetrys.addData("[Artifact] isRedAlliance ", artifactControl.isRedAlliance);

            telemetrys.update();
        }
    }
}
