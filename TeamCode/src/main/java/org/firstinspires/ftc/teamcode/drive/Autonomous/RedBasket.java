package org.firstinspires.ftc.teamcode.drive.Autonomous;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Basket_firstAngle;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Basket_secondAngle;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.VarStorage;
import org.firstinspires.ftc.teamcode.drive.Structure.ArtifactControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "RedBasket", group = "Autonomous")
@Configurable // Panels
public class RedBasket extends OpMode {
    int failSafeCase = 0;
    private Timer pathTimer, opModeTimer;
    ArtifactControl artifactControl;

    //private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    public enum PathState{
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > SCORE ARTIFACT

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_FIRST_ARTEFACT_LINE,
        OPEN_GATE,
        ROTATE_AFTER_OPEN_GATE,
        ROTATE_AFTER_OPEN_TO_SHOOT,
        SHOOT_SECOND_ARTIFACT,
        SHOOT_TO_SECOND_ARTIFACT_LINE,
        ROTATE_AFTER_SECOND_LINE,
        ROTATE_AFTER_SECOND_LINE_TO_SHOOT
    }

    PathState pathState;

    public  Pose startPose = new Pose(116.920,132.140,Math.toRadians(36.5));
    public PathChain startPoseToShootPose;
    public PathChain shootPoseToFirstArtefactLine;
    public PathChain openGate;
    public PathChain rotateAfterOpenGate;
    public PathChain rotateAfterOpenToShoot;
    public PathChain shootToSecondArtifacLine;
    public PathChain rotateAfterSecondLine;
    public PathChain rotateAfterSecondLineToShoot;


    public void buildPaths(){
        //put in coordonates for starting pos > put in coordonates for end pos

        startPoseToShootPose = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(116.920, 132.140),
                                new Pose(88.705, 90.396)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36.5), Math.toRadians(0))
                .build();
        shootPoseToFirstArtefactLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.705, 90.396),
                                new Pose(104.860, 80.299),
                                new Pose(128.357, 83.882)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        openGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.357, 83.882),
                                new Pose(115.133, 76.589),
                                new Pose(128.535, 70.001)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

//        rotateAfterOpenGate = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(128.535, 70.001),
//                                new Pose(86.999, 83.819)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                .build();
        rotateAfterOpenToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.535, 70.001),
                                new Pose(86.999, 83.819)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shootToSecondArtifacLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.999, 83.819),
                                new Pose(92.831, 69.012),
                                new Pose(89.488, 57.331),
                                new Pose(134.322, 59.068)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

//        rotateAfterSecondLine = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(134.322, 59.068),
//                                new Pose(127.449, 61.678)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(336))
//                .build();

        rotateAfterSecondLineToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(134.322, 59.068),
                                new Pose(86.537, 80.307)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(336))
                .build();
    }


    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(startPoseToShootPose,false);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                //TODO add shooting logic
                if(!follower.isBusy()){
                    artifactControl.setAutonomousThrowFlags();
                    artifactControl.setAutonomousShooter(Basket_firstAngle,true,follower.getPose().getX(),follower.getPose().getY(),true,false);
                    if(pathTimer.getElapsedTimeSeconds() > 2){
                        telemetry.addLine("Done Path 1: Shot Preload");
                        setPathState(PathState.DRIVE_FIRST_ARTEFACT_LINE);
                    }
                    //pui setAutonomousThrowFlags  cand vr sa trag
                    //setAutonomousShooter dai pozitie robotului
                }
                break;
            case DRIVE_FIRST_ARTEFACT_LINE:
                if(!follower.isBusy()){
                    artifactControl.getArtifacts(false);
                    //getartifacts()
                    follower.followPath(shootPoseToFirstArtefactLine,true);
                    telemetry.addLine("Done Path 2: Driving to First Artifact");
                    setPathState(PathState.OPEN_GATE);
                }
                break;
            case OPEN_GATE:
                if(!follower.isBusy()){
                    artifactControl.stopIntakeOuttake();
                    //stopintakeouttake()
                    follower.followPath(openGate,true);
                    telemetry.addLine("Opening Gate");
                    setPathState(PathState.ROTATE_AFTER_OPEN_TO_SHOOT);
                }
                break;
            case ROTATE_AFTER_OPEN_GATE:
                if(!follower.isBusy()){
                    follower.followPath(rotateAfterOpenGate,true);
                    telemetry.addLine("Rotating after opening gate");
                    setPathState(PathState.ROTATE_AFTER_OPEN_TO_SHOOT);
                }
                break;
            case ROTATE_AFTER_OPEN_TO_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(rotateAfterOpenToShoot,true);
                    telemetry.addLine("Rotating to shoot position");
                    setPathState(PathState.SHOOT_SECOND_ARTIFACT);
                }
                break;
            case SHOOT_SECOND_ARTIFACT:
                //TODO add shooting logic
                if(!follower.isBusy()){
                    artifactControl.setAutonomousResetFlags();
                    artifactControl.setAutonomousThrowFlags();

                    //artifactControl.setAutonomousShooter(Basket_secondAngle,true,follower.getPose().getX(),follower.getPose().getY(),);
                    //daca ai tras inainte bagi setAutonomousResetFlags ca sa resetezi
                    //dupa bagi setAutonomousThrowFlags
                    //dupa setAutonomousShooter

                    telemetry.addLine("Shot Second Artifact");
                    setPathState(PathState.SHOOT_TO_SECOND_ARTIFACT_LINE);
                }
                break;
            case SHOOT_TO_SECOND_ARTIFACT_LINE:
                if(!follower.isBusy()){
                    follower.followPath(shootToSecondArtifacLine,true);
                    telemetry.addLine("Driving to Second Artifact Line");
                    setPathState(PathState.ROTATE_AFTER_SECOND_LINE_TO_SHOOT);
                }
                break;
            case ROTATE_AFTER_SECOND_LINE:
                if(!follower.isBusy()){
                    follower.followPath(rotateAfterSecondLine,true);
                    telemetry.addLine("Rotating after second artifact line");
                    setPathState(PathState.ROTATE_AFTER_SECOND_LINE_TO_SHOOT);
                }
                break;
            case ROTATE_AFTER_SECOND_LINE_TO_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(rotateAfterSecondLineToShoot,true);
                    telemetry.addLine("Rotating to final shoot position");
                    //TODO transition to next cycle or end
                }
                break;
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
        VarStorage.autonomous_case = 0;
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanisms

        artifactControl = new ArtifactControl(hardwareMap, gamepad2,gamepad1, (MultipleTelemetry) telemetry);

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

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();

    }
}