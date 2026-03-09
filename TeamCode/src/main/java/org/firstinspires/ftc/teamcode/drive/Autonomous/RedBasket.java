package org.firstinspires.ftc.teamcode.drive.Autonomous;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Basket_firstAngle;
import static org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.AutoStorage.Basket_secondAngle;

import com.acmerobotics.dashboard.FtcDashboard;
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
    MultipleTelemetry telemetrys;

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
        ROTATE_AFTER_OPEN_TO_SHOOT,
        SHOOT_FIRST_ARTIFACT,
        SHOOT_TO_SECOND_ARTIFACT_LINE,
        SHOOT_SECOND_ARTIFACT,
        SHOOT_SECOND_ARTIFACT_FR
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
    boolean runOnece = false;

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(startPoseToShootPose,0.5,true);

                if(!runOnece){
                    artifactControl.setAutonomousThrowFlags();
                    runOnece = true;
                }
                artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);

                ;
               // artifactControl.updateBasketDistance(follower,true);
                if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                    setPathState(PathState.DRIVE_FIRST_ARTEFACT_LINE);
                    runOnece = false;
                }

                break;
            case SHOOT_PRELOAD:
                //TODO add shooting logic
                if(!follower.isBusy()){
                    if(!runOnece){
                        artifactControl.wantsToThrowArtifacts = true;
                        artifactControl.setAutonomousThrowFlags();
                        runOnece = true;
                    }
                    artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);

                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        runOnece = false;
                        telemetrys.addLine("Done Path 1: Shot Preload");
                        setPathState(PathState.DRIVE_FIRST_ARTEFACT_LINE);
                    }
                    //pui setAutonomousThrowFlags  cand vr sa trag
                    //setAutonomousShooter dai pozitie robotului
                }
                break;
            case DRIVE_FIRST_ARTEFACT_LINE:
                if(!follower.isBusy()){
                    if(!runOnece){
                        artifactControl.getArtifacts(false);
                        runOnece = true;
                    }
                    //getartifacts()
                    follower.followPath(shootPoseToFirstArtefactLine,true);
                    telemetrys.addLine("Done Path 2: Driving to First Artifact");
                    setPathState(PathState.OPEN_GATE);
                    runOnece = false;
                }
                break;
            case OPEN_GATE:
                if(!follower.isBusy()){
                    if(!runOnece){
                        artifactControl.stopIntakeOuttake();
                        runOnece = true;
                    }
                    //stopintakeouttake()
                    follower.followPath(openGate,false);
                    telemetrys.addLine("Opening Gate");
                    setPathState(PathState.ROTATE_AFTER_OPEN_TO_SHOOT);
                    runOnece = false;

                }
                break;
            case ROTATE_AFTER_OPEN_TO_SHOOT:
                if(!follower.isBusy()){
                    follower.followPath(rotateAfterOpenToShoot,true);
                    telemetrys.addLine("Rotating to shoot position");
                    setPathState(PathState.SHOOT_FIRST_ARTIFACT);
                }
                break;
            case SHOOT_FIRST_ARTIFACT:
                //TODO add shooting logic

                if(!follower.isBusy()){
                    if(!runOnece){
                        artifactControl.setAutonomousResetFlags();
                        artifactControl.wantsToThrowArtifacts = true;
                        artifactControl.setAutonomousThrowFlags();
                        artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);
                        runOnece = true;
                    }

                    //artifactControl.setAutonomousShooter(Basket_secondAngle,true,follower.getPose().getX(),follower.getPose().getY(),);
                    //daca ai tras inainte bagi setAutonomousResetFlags ca sa resetezi
                    //dupa bagi setAutonomousThrowFlags
                    //dupa setAutonomousShooter
                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        telemetrys.addLine("Shot Second Artifact");
                        setPathState(PathState.SHOOT_TO_SECOND_ARTIFACT_LINE);
                        runOnece = false;
                    }
                }
                break;
            case SHOOT_TO_SECOND_ARTIFACT_LINE:
                if(!follower.isBusy()){
                    if(!runOnece){
                        artifactControl.getArtifacts(false);
                        runOnece = true;
                    }

                    follower.followPath(shootToSecondArtifacLine,true);
                    telemetrys.addLine("Driving to Second Artifact Line");
                    setPathState(PathState.SHOOT_SECOND_ARTIFACT);
                    runOnece = false;
                }
                break;
            case SHOOT_SECOND_ARTIFACT:
                if(!follower.isBusy()){


                    if(!artifactControl.wantsToThrowArtifacts || pathTimer.getElapsedTimeSeconds() > 5){
                        follower.followPath(rotateAfterSecondLineToShoot);
                        setPathState(PathState.SHOOT_SECOND_ARTIFACT_FR);
                        telemetrys.addLine("Rotating to final shoot position");
                    }

                    //TODO transition to next cycle or end
                }
                break;
            case SHOOT_SECOND_ARTIFACT_FR:
                if(!follower.isBusy()){
                    if(!runOnece){
                        artifactControl.stopIntakeOuttake();
                        artifactControl.setAutonomousResetFlags();

                        artifactControl.wantsToThrowArtifacts = true;
                        artifactControl.setAutonomousThrowFlags();
                        artifactControl.setAutonomousShooter(Math.toDegrees(follower.getHeading()), convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,false);
                        runOnece = true;
                    }
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
    double autoCase  = 0;
    @Override
    public void init() {
        VarStorage.autonomous_case = 2;
        artifactControl = new ArtifactControl(hardwareMap, gamepad2,gamepad1,telemetrys);
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        telemetrys = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        artifactControl.initServo();
        //TODO add in any other init mechanisms


        buildPaths();
        follower.setPose(startPose);
    }


    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }


    double RRPosX = 0.0;
    double RRPosY = 0.0;
    double basketDistance;

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        //artifactControl.updateShooter(true,follower,true);

        RRPosX = convertPedroToFTCCoordsX(follower.getPose().getY());
        RRPosY = convertPedroToFTCCoordsY(follower.getPose().getX());

        if(artifactControl.wantsToThrowArtifacts) {
            artifactControl.throwArtifacts(artifactControl.getFlyWheelPower(convertPedroToFTCCoordsX(follower.getPose().getY()),convertPedroToFTCCoordsY(follower.getPose().getX()),true,true), true, true);
            //artifactControl.updateBasketDistance(follower,true);
            //artifactControl.updateShooter(true,follower,true);
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
            RRposY = -(RRposY - 72);
        }
        return RRposY;
    }
}