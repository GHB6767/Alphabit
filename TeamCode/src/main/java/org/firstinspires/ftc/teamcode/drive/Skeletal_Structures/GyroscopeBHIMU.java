package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class GyroscopeBHIMU {
    IMU imu;

    double customHeadingAngle = 0;
    public void gyroscope_init(HardwareMap hwdmap) {
        imu = hwdmap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getHeading(){
        customHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if(customHeadingAngle < 0){
            customHeadingAngle = 360 - Math.abs(customHeadingAngle);
        }
        return customHeadingAngle;
    }

    public void resetHeading(){
        imu.resetYaw();
    }
}
