package org.firstinspires.ftc.teamcode.drive.Autonomous;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;

import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        VarStorage.autonomous_case = 0;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path6;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(116.920, 132.140),
                                    new Pose(88.705, 90.396)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.705, 90.396),
                                    new Pose(104.860, 80.299),
                                    new Pose(128.357, 83.882)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(128.357, 83.882),
                                    new Pose(115.133, 76.589),
                                    new Pose(128.535, 70.001)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.535, 70.001),
                                    new Pose(121.116, 72.071)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(342))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(121.116, 72.071),
                                    new Pose(86.999, 83.819)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(342), Math.toRadians(342))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.999, 83.819),
                                    new Pose(92.831, 69.012),
                                    new Pose(89.154, 57.164),
                                    new Pose(134.322, 59.068)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(342), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(134.322, 59.068),
                                    new Pose(127.449, 61.678)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(336))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.449, 61.678),
                                    new Pose(86.537, 80.307)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(336), Math.toRadians(336))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(paths.Path1);
                if(!follower.isBusy()) {
                    pathState = 1;
                }
                break;
            case 1:
                follower.followPath(paths.Path2);
                if(!follower.isBusy()) {
                    pathState = 2;
                }
                break;
            case 2:
                follower.followPath(paths.Path3);
                if(!follower.isBusy()) {
                    pathState = 3;
                }
                break;
            case 3:
                follower.followPath(paths.Path4);
                if(!follower.isBusy()) {
                    pathState = 4;
                }
                break;
            case 4:
                follower.followPath(paths.Path5);
                if(!follower.isBusy()) {
                    pathState = 5;
                }
                break;
            case 5:
                follower.followPath(paths.Path6);
                if(!follower.isBusy()) {
                    pathState = 6;
                }
                break;
            case 6:
                follower.followPath(paths.Path7);
                if(!follower.isBusy()) {
                    pathState = 7;
                }
                break;
            case 7:
                follower.followPath(paths.Path8);
                if(!follower.isBusy()) {
                    pathState = 8;
                }
                break;
        }
        return pathState;
    }
}//sex