package org.firstinspires.ftc.teamcode.drive.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp
public class ShootetrPIDFTuner extends OpMode {

    public DcMotorEx FlfywheelMotor;

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double currTargetVelocity;


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
