package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarRedBasket", group = "Autonomous")
@Configurable
public class FarRedBasket extends OpMode {
    private Timer pathTimer, opModeTimer;
    ArtifactControl artifactControl;
    MultipleTelemetry telemetrys;
    public Follower follower;

    public enum PathState {
        PATH1,
        PATH2,
        SHOOT1,
        PATH3,
        PATH4,
        SHOOT3,
        PATH5,
        PATH6,
        SHOOT5,
        PATH7,
        PATH8,
        SHOOT7,
        PATH9
    }

    PathState pathState;

    public Pose startPose = new Pose(82.531, 8.667, Math.toRadians(0));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;

    boolean runOnce = false;

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(82.531, 8.667),
                                new Pose(116.148, 25.667),
                                new Pose(133.423, 10.804)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.423, 10.804),
                                new Pose(92.835, 9.920)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(92.835, 9.920),
                                new Pose(83.041, 38.140),
                                new Pose(135.313, 36.657)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(135.313, 36.657),
                                new Pose(89.956, 14.282)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(89.956, 14.282),
                                new Pose(133.154, 9.845)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(336))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.154, 9.845),
                                new Pose(94.443, 10.679)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(336), Math.toRadians(0))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(94.443, 10.679),
                                new Pose(133.154, 9.845)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(336))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(133.154, 9.845),
                                new Pose(94.688, 10.269)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(336), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(94.688, 10.269),
                                new Pose(101.476, 9.971)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case PATH1:
                artifactControl.getArtifacts(false);
                follower.followPath(Path1, true);
                setPathState(PathState.PATH2);
                break;

            case PATH2:
                if (!follower.isBusy()) {
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path2, true);
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!follower.isBusy()) {
                    shootArtifact();
                    if (!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5) {
                        runOnce = false;
                        setPathState(PathState.PATH3);
                    }
                }
                break;

            case PATH3:
                if (!follower.isBusy()) {
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path3, true);
                    setPathState(PathState.PATH4);
                }
                break;

            case PATH4:
                if (!follower.isBusy()) {
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path4, true);
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!follower.isBusy()) {
                    shootArtifact();
                    if (!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5) {
                        runOnce = false;
                        setPathState(PathState.PATH5);
                    }
                }
                break;

            case PATH5:
                if (!follower.isBusy()) {
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path5, true);
                    setPathState(PathState.PATH6);
                }
                break;

            case PATH6:
                if (!follower.isBusy()) {
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path6, true);
                    setPathState(PathState.SHOOT5);
                }
                break;

            case SHOOT5:
                if (!follower.isBusy()) {
                    shootArtifact();
                    if (!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5) {
                        runOnce = false;
                        setPathState(PathState.PATH7);
                    }
                }
                break;

            case PATH7:
                if (!follower.isBusy()) {
                    artifactControl.getArtifacts(false);
                    follower.followPath(Path7, true);
                    setPathState(PathState.PATH8);
                }
                break;

            case PATH8:
                if (!follower.isBusy()) {
                    artifactControl.stopIntakeOuttake();
                    follower.followPath(Path8, true);
                    setPathState(PathState.SHOOT7);
                }
                break;

            case SHOOT7:
                if (!follower.isBusy()) {
                    shootArtifact();
                    if (!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5) {
                        runOnce = false;
                        setPathState(PathState.PATH9);
                    }
                }
                break;

            case PATH9:
                if (!follower.isBusy()) {
                    follower.followPath(Path9, false);
                }
                break;

            default:
                telemetrys.addLine("no state commanded");
                break;
        }
    }

    public void shootArtifact() {
        if (!runOnce) {
            artifactControl.setAutonomousResetFlags();
            artifactControl.setAutonomousThrowFlags();
            artifactControl.setAutonomousShooter(
                    Math.toDegrees(follower.getHeading()),
                    convertPedroToFTCCoordsX(follower.getPose().getY()),
                    convertPedroToFTCCoordsY(follower.getPose().getX()),
                    true,
                    true
            );
            runOnce = true;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        VarStorage.autonomous_case = 0;
        artifactControl = new ArtifactControl(hardwareMap, gamepad2, gamepad1, telemetrys);
        pathState = PathState.PATH1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        artifactControl.initServo();
        buildPaths();
        follower.setPose(startPose);
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
            artifactControl.throwArtifacts(
                    artifactControl.getFlyWheelPower(
                            convertPedroToFTCCoordsX(follower.getPose().getY()),
                            convertPedroToFTCCoordsY(follower.getPose().getX()),
                            true,
                            true
                    ),
                    true,
                    true
            );
        }

        telemetrys.addData("Path State", pathState);
        telemetrys.addData("X", follower.getPose().getX());
        telemetrys.addData("Y", follower.getPose().getY());
        telemetrys.addData("Heading", follower.getPose().getHeading());
        telemetrys.update();
    }

    public double convertPedroToFTCCoordsX(double RRposX) {
        if (RRposX >= 72) {
            RRposX = -(RRposX - 72);
        } else {
            RRposX = Math.abs(RRposX - 72);
        }
        return RRposX;
    }

    public double convertPedroToFTCCoordsY(double RRposY) {
        if (RRposY >= 72) {
            RRposY = Math.abs(RRposY - 72);
        } else {
            RRposY = RRposY - 72;
        }
        return RRposY;
    }
}
