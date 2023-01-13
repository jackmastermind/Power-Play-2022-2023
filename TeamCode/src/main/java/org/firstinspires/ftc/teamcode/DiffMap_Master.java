package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DiffMap_Master extends HardwareMap_Master{
    public DiffSwerve diff = new DiffSwerve();

    public DcMotor susan, slide;
    public Servo wrist, claw;

    @Override
    public void init(HardwareMap ahwMap, boolean chassisOnly) {
        hwMap = ahwMap;
        diff.initialize(ahwMap);

        if (!chassisOnly)
        {
            susan = hwMap.get(DcMotor.class, "susan");
            slide = hwMap.get(DcMotor.class, "spool");

            claw = hwMap.get(Servo.class, "claw");
            wrist = hwMap.get(Servo.class, "wrist");

            motors = new DcMotor[] {diff.leftTop, diff.leftBottom, diff.rightTop, diff.rightBottom,
                                    susan, slide};
            servos = new Servo[] {wrist, claw};
        }
        else
        {
            motors = diff.motors;
        }
    }
}
