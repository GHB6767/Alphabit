package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageReader {
    VoltageSensor voltageSensor;

    double minimumVoltage = 12.5;

    public VoltageReader(HardwareMap hwdmap) {
        voltageSensor = hwdmap.voltageSensor.iterator().next();
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public double getCompensation(){
        double currentVoltage = getVoltage();
        if(currentVoltage < minimumVoltage){
            return 1.0;
        }else{
            return minimumVoltage/currentVoltage;
        }
    }
}
