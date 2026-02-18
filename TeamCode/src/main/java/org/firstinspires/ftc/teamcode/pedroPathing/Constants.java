package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class Constants {
    public static double forwardMultiplier = -0.0010602235610886;
    public static double strafeMultiplier = 0.0010602235610886;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5)
            .forwardZeroPowerAcceleration(-57.31167)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.012, 0.009))
            .headingPIDFCoefficients(new PIDFCoefficients(0.65, 0, 0.2, 0.005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.035,0.0,0.005,0.6,0.05))
            .centripetalScaling(0.000515);



    public static MecanumConstants driveConstants = new MecanumConstants()

            .maxPower(1)
            .rightFrontMotorName("Front_Right")
            .rightRearMotorName("Back_Right")
            .leftRearMotorName("Back_Left")
            .leftFrontMotorName("Front_Left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(72.88502)
            .yVelocity(66.2029);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3)
            .strafePodX(6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(37.25)//counts per MM not IN may cause trouble if so change pod offsets to MM
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
//            .forwardEncoder_HardwareMapName("Back_Left")
//            .strafeEncoder_HardwareMapName("Back_Right")
//            .forwardPodY(2.9960)
//            .strafePodX(-6.1417)
//            .forwardTicksToInches(forwardMultiplier)
//            .strafeTicksToInches(strafeMultiplier)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.UP
//                    )
//            );

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                //.twoWheelLocalizer(localizerConstants)
                .build();
    }
}
