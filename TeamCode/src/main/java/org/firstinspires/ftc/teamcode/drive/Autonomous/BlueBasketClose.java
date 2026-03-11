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
public class BlueBasketClose extends OpMode {
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
        SHOOT6,
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
                                new Pose(27.080, 132.140),
                                new Pose(55.248, 85.624)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143.5), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.248, 85.624),
                                new Pose(59.762, 53.859),
                                new Pose(10.781, 59.005)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.781, 59.005),
                                new Pose(42.520, 64.799),
                                new Pose(17.723, 70.449)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17.723, 70.449),
                                new Pose(48.494, 67.365),
                                new Pose(53.357, 86.865)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(53.357, 86.865),
                                new Pose(38.475, 83.297),
                                new Pose(16.929, 83.717)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.929, 83.717),
                                new Pose(53.815, 88.942)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(53.815, 88.942),
                                new Pose(40.492, 79.197)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    boolean runOnce = false;
    boolean RunOnce = false;

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
                    artifactControl.getArtifacts(false);
                    if(!runOnce){
                        follower.followPath(Path2,0.6,false);
                        runOnce = true;
                    }
                    setPathState(PathState.PATH3);
                    runOnce = false;
                }
                break;
            case PATH3:
                if(!follower.isBusy()){
                    follower.followPath(Path3,1,true);
                    setPathState(PathState.PATH4);
                    runOnce = false;
                }
                break;
            case PATH4:
                if(!follower.isBusy()){
                    artifactControl.stopIntake();
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
                    artifactControl.getArtifacts(false);

                    if(!runOnce){
                        follower.followPath(Path5,0.6,false);
                        runOnce = true;
                    }
                    setPathState(PathState.PATH6);
                    runOnce = false;
                }
                break;
            case PATH6:
                if(!follower.isBusy()){
                    follower.followPath(Path6,1,true);
                    runOnce = false;
                    RunOnce = false;
                    setPathState(PathState.SHOOT6);
                }
                break;
            case SHOOT6:
                if(!follower.isBusy()){
                    if(!RunOnce){
                        artifactControl.stopIntake();
                        RunOnce = true;
                    }
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts) {
                        follower.followPath(Path7,1,false);
                        setPathState(PathState.PATH7);
                        runOnce = false;
                    }
                }
                break;
            case PATH7:
                if(!follower.isBusy()){
                    follower.followPath(Path7,1,true);
                    setPathState(PathState.Finish);
                }
                break;
            case Finish:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    follower.breakFollowing();

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
            artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),false,false);
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
            artifactControl.throwArtifacts(artifactControl.getFlyWheelPower(convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),false,true), true, true);
        }
        //artifactControl.throwArtifacts(artifactControl.getFlyWheelPower(convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,true), true, true);

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
