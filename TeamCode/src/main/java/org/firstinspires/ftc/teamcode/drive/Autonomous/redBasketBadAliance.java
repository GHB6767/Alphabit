package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class redBasketBadAliance extends OpMode {
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
        SHOOT7,
        PATH8,
        PATH9,
        SHOOT9,
        PATH10
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
                                new Pose(128.346, 71.189),
                                new Pose(91.052, 81.969),
                                new Pose(81.365, 58.435),
                                new Pose(134.474, 58.265)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(134.474, 58.265),
                                new Pose(109.978, 56.583),
                                new Pose(87.280, 86.778)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.280, 86.778),
                                new Pose(127.882, 64.027)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(127.882, 64.027),
                                new Pose(131.525, 45.821),
                                new Pose(133.655, 56.232)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.655, 56.232),
                                new Pose(88.509, 86.423)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.509, 86.423),
                                new Pose(128.314, 84.717)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.314, 84.717),
                                new Pose(87.957, 85.455)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.957, 85.455),
                                new Pose(113.563, 85.065)
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
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        setPathState(PathState.PATH2);
                        runOnce = false;
                    }
                }

                break;
            case PATH2:
                if(!follower.isBusy()){
                    stayTimer.resetTimer();

                    follower.followPath(Path2,true);
                    if(stayTimer.getElapsedTimeSeconds() > 0.5){
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
                    follower.followPath(Path3,true);
                    setPathState(PathState.PATH4);
                    runOnce = false;
                }
                break;
            case PATH4:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path4,true);
                    setPathState(PathState.SHOOT4);
                }
                break;
            case SHOOT4:
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        setPathState(PathState.PATH5);
                        runOnce = false;
                    }
                }
                break;
            case PATH5:
                if(!follower.isBusy()){
                    stayTimer.resetTimer();
                    follower.followPath(Path5,true);
                    if(stayTimer.getElapsedTimeSeconds() > 0.5){
                        setPathState(PathState.PATH6);
                    }
                }
                break;
            case PATH6:
                if(!follower.isBusy()){
                    stayTimer.resetTimer();
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path6,true);
                    if(stayTimer.getElapsedTimeSeconds() > 1){
                        setPathState(PathState.PATH7);

                    }

                }
                break;
            case PATH7:
                if(!follower.isBusy()){
                    follower.followPath(Path7,false);
                    setPathState(PathState.SHOOT7);
                }
                break;
            case SHOOT7:
                if(!follower.isBusy()){
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        setPathState(PathState.PATH8);
                        runOnce = false;
                    }
                }
                break;

            case PATH8:
                if(!follower.isBusy()){
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path8,false);
                    setPathState(PathState.PATH9);
                }
                break;
            case PATH9:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path10,false);
                    setPathState(PathState.SHOOT9);
                    runOnce = false;

                }
                break;
            case SHOOT9:
                if(!follower.isBusy()){
                    shootArtifact();

                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        setPathState(PathState.PATH10);
                        runOnce = false;
                    }
                }
            case PATH10:
                if(!follower.isBusy()){
                    follower.followPath(Path10);
                }
                break;

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

    public void shootArtifact(){
        if(!runOnce){
            artifactControl.setAutonomousResetFlags();
            artifactControl.setAutonomousThrowFlags();
            artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);
            runOnce = true;
        }
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
