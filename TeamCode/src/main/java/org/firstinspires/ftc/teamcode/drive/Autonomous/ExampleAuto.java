package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Configurable
public class ExampleAuto extends OpMode {
    private Timer pathTimer, opModeTimer;
    ArtifactControl artifactControl;
    MultipleTelemetry telemetrys;
    public Follower follower;
    public enum PathState{
        PATH1,
        SHOOT1,
        PATH2,
        PATH3

    }

    PathState pathState;

    public Pose startPose = new Pose(116.920,132.140,Math.toRadians(36.5));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public void buildPaths(){
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(116.920, 132.140),
                                new Pose(88.752, 85.624)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.752, 85.624),
                                new Pose(102.625, 68.481),
                                new Pose(128.346, 71.189)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(67, 67),
                                new Pose(102.625, 68.481),
                                new Pose(128.346, 71.189)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();


    }
    boolean runOnce = false;

    public void statePathUpdate(){
        switch (pathState){
            case PATH1:
                follower.followPath(Path1, true);
                setPathState(PathState.SHOOT1);
                break;
            case SHOOT1:
                shootArtifact();//shoot artifact if shoot state

                if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                    setPathState(PathState.PATH2); //set pathstate to next pathstate
                    runOnce = false;
                }
                break;
            case PATH2:
                artifactControl.getArtifacts(false); //if path need to intake artifacts
                if(!follower.isBusy()){
                    follower.followPath(Path2,false);
                    setPathState(PathState.PATH2/*next path*/);
                }
                break;
            case PATH3:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();// if the path before had to intake
                    follower.followPath(Path3,false);
                    setPathState(PathState.PATH2/*next path*/);
                }


            default:
                telemetrys.addLine("no state commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void shootArtifact(){
        if(!runOnce){
            artifactControl.setAutonomousResetFlags();
            artifactControl.setAutonomousThrowFlags();
            artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);
            runOnce = true;
        }
    }

    @Override
    public void init(){
        VarStorage.autonomous_case = 2;
        artifactControl = new ArtifactControl(hardwareMap, gamepad2,gamepad1,telemetrys);
        pathState = PathState.PATH1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        artifactControl.initServo();
        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        if(artifactControl.wantsToThrowArtifacts){
            artifactControl.throwArtifacts(artifactControl.getFlyWheelPower(convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,true), true, true);
        }

        telemetrys.addData("Path State", pathState);
        telemetrys.addData("X", follower.getPose().getX());
        telemetrys.addData("Y", follower.getPose().getY());
        telemetrys.addData("Heading", follower.getPose().getHeading());
        telemetrys.update();
    }

    public double convertPedroToFTCCoordsX(double RRposX){
        if(RRposX >= 72){
            RRposX = -(RRposX - 72);
        }else if(RRposX < 72){
            RRposX = Math.abs(RRposX - 72);

        }
        return RRposX;
    }

    public double convertPedroToFTCCoordsY(double RRposY){
        if(RRposY >= 72){
            RRposY = Math.abs(RRposY - 72);
        }else if(RRposY < 72){
            RRposY = RRposY - 72;
        }
        return RRposY;
    }
}
