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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class redBasketGoodAliance extends OpMode {
    private Timer pathTimer, opModeTimer,stayTimer;
    ArtifactControl artifactControl;
    MultipleTelemetry telemetrys;
    public Follower follower;
    public enum PathState{
        PATH1,
        SHOOT1,
        PATH2,
        PATH3,
        PATH4,
        SHOOT4,
        PATH5,
        PATH6,
        PATH7,
        PATH8,
        SHOOT8,
        PATH9,
        PATH10,
        SHOOT10,
        PATH11,
        Finish
    }

    PathState pathState;

    public Pose startPose = new Pose(116.920,132.140,Math.toRadians(36.5));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;

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
                                new Pose(126.795, 71.411)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.795, 71.411),
                                new Pose(81.305, 73.994),
                                new Pose(81.365, 58.435),
                                new Pose(132.923, 58.265)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.923, 58.265),
                                new Pose(109.978, 56.583),
                                new Pose(87.280, 86.778)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.280, 86.778),
                                new Pose(94.618, 70.197),
                                new Pose(127.218, 67.794)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(127.218, 67.794),
                                new Pose(111.335, 61.172),
                                new Pose(132.923, 50.068)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.923, 50.068),
                                new Pose(132.347, 56.771)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(70))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.347, 56.771),
                                new Pose(88.631, 85.148)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.631, 85.148),
                                new Pose(128.314, 84.717)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.314, 84.717),
                                new Pose(87.957, 85.455)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.957, 85.455),
                                new Pose(105.920, 84.997)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    boolean runOnce = false;

    public void statePathUpdate(){
        switch (pathState){
            case PATH1:
                follower.followPath(Path1,1, true);
                setPathState(PathState.SHOOT1);
                break;
            case SHOOT1:
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts){ //|| pathTimer.getElapsedTimeSeconds() > 8
                        setPathState(PathState.PATH2);
                        runOnce = false;
                    }
                }

                break;
            case PATH2:
                if(!follower.isBusy()){
                    //stayTimer.resetTimer();
                    follower.followPath(Path2,1,false);
                    if(pathTimer.getElapsedTimeSeconds() > 3.75){
                        setPathState(PathState.PATH3);
                    }
                }
                break;
            case PATH3:
                if(!follower.isBusy()){
                    if(!runOnce){
                        artifactControl.getArtifacts(false);
                        runOnce = true;
                    }
                    follower.followPath(Path3,1,true);
                    setPathState(PathState.PATH4);
                    runOnce = false;
                }
                break;
            case PATH4:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path4,1,true);
                    setPathState(PathState.SHOOT4);
                }
                break;
            case SHOOT4:
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts){
                        setPathState(PathState.PATH5);
                        runOnce = false;
                    }
                }
                break;
            case PATH5:
                if(!follower.isBusy()){
                    follower.followPath(Path5,1,true);
                    if(pathTimer.getElapsedTimeSeconds() > 2.75){
                        setPathState(PathState.PATH6);
                    }

                }
                break;
            case PATH6:
                if(!follower.isBusy()){
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path6,1,true);
                    setPathState(PathState.PATH7);
                }
                break;
            case PATH7:
                if(!follower.isBusy()){
                    stayTimer.resetTimer();

                    //if(pathTimer.getElapsedTimeSeconds()>1){
                        follower.followPath(Path7,1,false);
                        setPathState(PathState.PATH8);
                    //}

                }
                break;
            case PATH8:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path8,1,true);
                    setPathState(PathState.SHOOT8);
                }
                break;
            case SHOOT8:
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts){
                        setPathState(PathState.PATH9);
                        runOnce = false;
                    }
                }
                break;
            case PATH9:
                if(!follower.isBusy()){
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path9,1,false);
                    setPathState(PathState.PATH10);
                }
                break;
            case PATH10:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path10,1,false);
                    setPathState(PathState.SHOOT10);
                    runOnce = false;

                }
                break;
            case SHOOT10:
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts){
                        setPathState(PathState.PATH11);
                        runOnce = false;
                    }
                }
            case PATH11:
                if(!follower.isBusy()){
                    follower.followPath(Path11, 1,true);
                    setPathState(PathState.Finish);
                }
                break;
            case Finish:
                if(!follower.isBusy()){
                    follower.breakFollowing();
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

    @Override
    public void init(){
        VarStorage.autonomous_case = 2;
        artifactControl = new ArtifactControl(hardwareMap, gamepad2,gamepad1,telemetrys);
        pathState = PathState.PATH1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        stayTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        artifactControl.initServo();
        buildPaths();
        follower.setPose(startPose);
    }

    public void shootArtifact(){
        if(!runOnce){
            artifactControl.setAutonomousResetFlags();
            artifactControl.setAutonomousThrowFlags();
            artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);
            runOnce = true;
        }
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
