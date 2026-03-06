package org.firstinspires.ftc.teamcode.drive.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private Timer pathTimer, opModeTimer;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    public enum PathState{
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > SCORE ARTIFACT

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        FIRST_ARTIFACT_LINE
    }

    PathState pathState;

    private final Pose startPose = new Pose(116.920, 132.140,Math.toRadians(36.5));
    private final Pose shootPose = new Pose(87.037, 96.993,Math.toRadians(0));
    private final  Pose firstArtifactLine = new Pose(102.80301274623407,84.08922363847047, Math.toRadians(0));

    private PathChain driveStartPosToShootPos, driveShootPosToFirstArtifactLine;


    public void buildPaths(){
        //put in coordonates for starting pos > put in coordonates for end pos
        driveStartPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        driveShootPosToFirstArtifactLine = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstArtifactLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(),firstArtifactLine.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosToShootPos,true);
                setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);// reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                //TODO add shooting logic
                //check is follower is done ?
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
                    //transition to next state
                }
                break;
            case FIRST_ARTIFACT_LINE:
                if(!follower.isBusy()){
                    follower.followPath(driveShootPosToFirstArtifactLine,true);
                    telemetry.addLine("Done Path 2");
                }
            default:
                telemetry.addLine("no state commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);
    }


    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();


        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
}