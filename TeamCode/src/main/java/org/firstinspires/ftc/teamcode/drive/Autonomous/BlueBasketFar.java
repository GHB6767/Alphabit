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

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class BlueBasketFar extends OpMode {
    private Timer pathTimer, opModeTimer, stayTimer;
    ArtifactControl artifactControl;
    MultipleTelemetry telemetrys;
    public Follower follower;

    public enum PathState {
        PATH1,
        SHOOT1,
        PATH2,
        PATH3,
        PATH4,
        PATH5,
        SHOOT5,
        PATH6,
        PATH7,
        SHOOT7,
        PATH8,
        Finish
    }

    PathState pathState;

    public Pose startPose = new Pose(62.966, 6.892, Math.toRadians(180));

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


    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(62.966, 6.892),
                                new Pose(58.929, 15.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(58.929, 15.951),
                                new Pose(18.308, 19.126)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-145))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.308, 19.126),
                                new Pose(7.723, 24.086),
                                new Pose(10.412, 11.520)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-145), Math.toRadians(-150))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.412, 11.520),
                                new Pose(10.062, 8.225)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-170))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.062, 8.225),
                                new Pose(13.508, 22.178),
                                new Pose(56.455, 11.575)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-170), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.455, 11.575),
                                new Pose(72.902, 37.334),
                                new Pose(10.505, 36.089)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.505, 36.089),
                                new Pose(57.889, 11.775)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.889, 11.775),
                                new Pose(11.855, 14.705)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();
    }

    boolean runOnce = false;
    boolean RunOnce = false;

    public void statePathUpdate() {
        switch (pathState) {
            case PATH1:
                follower.followPath(Path1, 1, true);
                setPathState(PathState.SHOOT1);
                break;
            case SHOOT1:
                if (!follower.isBusy()) {
                    shootArtifact();
                    if (!artifactControl.wantsToThrowArtifacts) { //|| pathTimer.getElapsedTimeSeconds() > 8
                        setPathState(PathState.PATH2);
                        runOnce = false;
                    }
                }
                break;
            case PATH2:
                if (!follower.isBusy()) {
                    if (!runOnce) {
                        follower.followPath(Path2, 1, false);
                        runOnce = true;
                    }
                    setPathState(PathState.PATH3);
                    runOnce = false;
                }
                break;
            case PATH3:
                if (!follower.isBusy()) {
                    if (!runOnce) {
                        artifactControl.getArtifacts(false);
                        follower.followPath(Path3, 0.65, true);
                    }

                    setPathState(PathState.PATH4);
                    runOnce = false;

                }
                break;
            case PATH4:
                if (!follower.isBusy()) {
                    follower.followPath(Path4, 0.65, true);
                    setPathState(PathState.PATH5);
                    runOnce = false;
                    RunOnce = false;
                }
                break;
            case PATH5:
                if (!follower.isBusy()) {
                    follower.followPath(Path5);
                    setPathState(PathState.SHOOT5);
                    runOnce = false;
                    RunOnce = false;
                }

                break;
            case SHOOT5:
                if (!follower.isBusy()) {
                    if(!RunOnce){
                        artifactControl.stopIntake();
                        RunOnce = true;
                    }

                    if (!runOnce) {
                        shootArtifact();
                        //runOnce = true;
                        //RunOnce = true;
                    }
                    //runOnce = false;
                    if(!artifactControl.wantsToThrowArtifacts){
                        setPathState(PathState.PATH6);
                        runOnce = false;
                        RunOnce = false;

                    }
                }
                break;
            case PATH6:
                if (!follower.isBusy()) {

                    artifactControl.getArtifacts(false);
                    follower.followPath(Path6, 0.65, true);
                    runOnce = false;
                    RunOnce = false;
                    setPathState(PathState.PATH7);
                }
                break;
            case PATH7:
                if (!follower.isBusy()) {
                    //artifactControl.getArtifacts(false);
                    follower.followPath(Path7, 1, true);
                    setPathState(PathState.SHOOT7);
                }
                break;
            case SHOOT7:
                if (!follower.isBusy()) {
                    if(!RunOnce){
                        artifactControl.stopIntakeOuttake();
                        RunOnce = true;
                    }
                    shootArtifact();
                    if(!artifactControl.wantsToThrowArtifacts){
                        setPathState(PathState.PATH8);
                        runOnce = false ;
                        RunOnce = false;
                    }
                }
                break;
            case PATH8:
                if (!follower.isBusy()) {
                    follower.followPath(Path8, 1, false);
                    setPathState(PathState.Finish);
                }
                break;
            case Finish:
                if (!follower.isBusy()) {
                    artifactControl.stopIntakeOuttake();
                    follower.breakFollowing();

                }
                break;

            default:
                telemetrys.addLine("no state commanded");

                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        VarStorage.autonomous_case = 1;
        artifactControl = new ArtifactControl(hardwareMap, gamepad2, gamepad1, telemetrys);
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

    public void shootArtifact() {
        if (!runOnce) {
            artifactControl.setAutonomousResetFlags();
            artifactControl.setAutonomousThrowFlags();
            artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()), convertPedroToFTCCoordsY(follower.getPose().getX()), false, false);
            runOnce = true;
        }
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        if (artifactControl.wantsToThrowArtifacts) {
            artifactControl.throwArtifacts(artifactControl.getFlyWheelPower(convertPedroToFTCCoordsX(follower.getPose().getY()), convertPedroToFTCCoordsY(follower.getPose().getX()), false, true), true, true);
        }
        //artifactControl.throwArtifacts(artifactControl.getFlyWheelPower(convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,true), true, true);

        telemetrys.addData("Path State", pathState);
        telemetrys.addData("X", follower.getPose().getX());
        telemetrys.addData("Y", follower.getPose().getY());
        telemetrys.addData("Heading", follower.getPose().getHeading());
        telemetrys.update();
    }

    public double convertPedroToFTCCoordsX(double RRposX) {
        if (RRposX >= 72) {
            RRposX = -(RRposX - 72);
        } else if (RRposX < 72) {
            RRposX = Math.abs(RRposX - 72);

        }
        return RRposX;
    }

    public double convertPedroToFTCCoordsY(double RRposY) {
        if (RRposY >= 72) {
            RRposY = Math.abs(RRposY - 72);
        } else if (RRposY < 72) {
            RRposY = RRposY - 72;
        }
        return RRposY;
    }

}
