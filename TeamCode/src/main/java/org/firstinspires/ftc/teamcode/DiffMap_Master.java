package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DiffMap_Master extends HardwareMap_Master{
    public DiffSwerve diff = new DiffSwerve();

    public DcMotor susan, slide;
    public Servo claw, wrist;

    @Override
    public void init(HardwareMap ahwMap, boolean chassisOnly) {
        hwMap = ahwMap;
        diff.initialize(ahwMap);
        motors = diff.motors;
        if (!chassisOnly)
        {
            susan = hwMap.get(DcMotor.class, "susan");
            slide = hwMap.get(DcMotor.class, "spool");

            claw = hwMap.get(Servo.class, "claw");
            wrist = hwMap.get(Servo.class, "wrist");
        }
    }
}
