package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class DiffMap_Master extends HardwareMap_Master{
    public DiffSwerve diff = new DiffSwerve();

    @Override
    public void init(HardwareMap ahwMap, boolean chassisOnly) {
        hwMap = ahwMap;
        diff.initialize(ahwMap);
        motors = diff.motors;
        if (!chassisOnly)
        {
            //TODO: Initialize attachments
        }
    }
}
