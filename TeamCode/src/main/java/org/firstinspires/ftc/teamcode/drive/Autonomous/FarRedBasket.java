package org.firstinspires.ftc.teamcode.drive.Autonomous;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FarRedBasket", group = "Autonomous")
@Configurable
public class FarRedBasket extends OpMode {
    private Timer pathTimer,opModeTimer;
    public Follower follower;
    public enum PathState{
        START_POS_TO_SHOOT_POSE,
        SHOOT_PRELAOD,
        SHOOT_POSE_TO_HUMAN_PLAYER_ARTEFACT,
        HUMAN_PLAYER_ARTEFACT_TO_SECOND_POS,
        HUMAN_PLAYER_SECOND_POSE_TO_SHOOT,
        SHOOT_POSE_TO_THIRD_LINE,
        THIRD_LINE_TO_SHOOT,
        EXIT_SHOOTING_ZONE
    }
    PathState pathState;

    public Pose startPose = new Pose(78.90382387022018,7.748551564310569,Math.toRadians(0));
    public PathChain StartPosToShootPose;
    public PathChain ShootPoseToHumanPlayerArtefact;
    public PathChain HumanPlayerArtefactToSecondPos;
    public PathChain HumanPlayerSecondPoseToShoot;
    public PathChain ShootPoseToThirdLine;
    public PathChain ThirdLineToShoot;
    public PathChain ExitShootingZone;

    public void buildPaths(){
        StartPosToShootPose = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(78.904, 7.749),
                                new Pose(82.457, 9.275)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ShootPoseToHumanPlayerArtefact = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(82.457, 9.275),
                                new Pose(132.987, 17.353)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .build();

        HumanPlayerArtefactToSecondPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.987, 17.353),
                                new Pose(121.241, 8.998),
                                new Pose(132.876, 7.079)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(0))
                .build();

        HumanPlayerSecondPoseToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132.876, 7.079),
                                new Pose(92.549, 7.222)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        ShootPoseToThirdLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(92.549, 7.222),
                                new Pose(85.371, 42.603),
                                new Pose(134.750, 34.443)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ThirdLineToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(134.750, 34.443),
                                new Pose(87.809, 10.381)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ExitShootingZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.809, 10.381),
                                new Pose(122.976, 20.524)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case START_POS_TO_SHOOT_POSE:
                follower.followPath(StartPosToShootPose);
                setPathState(PathState.SHOOT_PRELAOD);
                break;
            case SHOOT_PRELAOD:
                if(!follower.isBusy()){
                    //TODO add shooting logic
                    telemetry.addLine("Done Path 1: Shot Preload");
                    setPathState(PathState.SHOOT_POSE_TO_HUMAN_PLAYER_ARTEFACT);
                }
                break;
            case SHOOT_POSE_TO_HUMAN_PLAYER_ARTEFACT:
                if(!follower.isBusy()){
                    follower.followPath(ShootPoseToHumanPlayerArtefact,true);
                    telemetry.addLine("Done Path 2: Driving to First Artifact");
                    setPathState(PathState.HUMAN_PLAYER_ARTEFACT_TO_SECOND_POS);
                }
                break;
            case HUMAN_PLAYER_ARTEFACT_TO_SECOND_POS:
                if(!follower.isBusy()){
                    follower.followPath(HumanPlayerArtefactToSecondPos);
                    setPathState(PathState.HUMAN_PLAYER_SECOND_POSE_TO_SHOOT);
                }
                break;
            case HUMAN_PLAYER_SECOND_POSE_TO_SHOOT:
                if(!follower.isBusy()){
                    //TODO add shooting logic
                    follower.followPath(HumanPlayerSecondPoseToShoot);
                    setPathState(PathState.SHOOT_POSE_TO_THIRD_LINE);
                }
                break;
            case SHOOT_POSE_TO_THIRD_LINE:
                if(!follower.isBusy()){
                    follower.followPath(ShootPoseToThirdLine);
                    setPathState(PathState.THIRD_LINE_TO_SHOOT);
                }
                break;
            case THIRD_LINE_TO_SHOOT:
                if(!follower.isBusy()){
                    //TODO add shooting logic
                  follower.followPath(ThirdLineToShoot);
                  setPathState(PathState.EXIT_SHOOTING_ZONE);
                }
                break;
            case EXIT_SHOOTING_ZONE:
                if(!follower.isBusy()){
                    follower.followPath(ExitShootingZone);
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
    public void init(){
        pathState = PathState.START_POS_TO_SHOOT_POSE;
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
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();

    }
}
