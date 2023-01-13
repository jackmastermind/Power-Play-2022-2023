package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class HardwareMap_Master {
    public DcMotor[] motors;
    public Servo[] servos;
    public HardwareMap hwMap;

    public abstract void init(HardwareMap ahwMap, boolean chassisOnly);

    public void init(HardwareMap ahwMap)
    {
        init(ahwMap, false);
    }

    public String filenameSelect(LinearOpMode currentOpMode)
    {
        Gamepad gamepad1 = currentOpMode.gamepad1;
        if (gamepad1.a)
        {
            return "recordingA.json";
        }
        else if (gamepad1.b)
        {
            return "recordingB.json";
        }
        else if (gamepad1.x)
        {
            return "recordingX.json";
        }
        else if (gamepad1.y)
        {
            return "recordingY.json";
        }
        else if (gamepad1.dpad_up)
        {
            return "recordingUP.json";
        }
        else if (gamepad1.dpad_left)
        {
            return "recordingLEFT.json";
        }
        else if (gamepad1.dpad_down)
        {
            return "recordingDOWN.json";
        }
        else if (gamepad1.dpad_right)
        {
            return "recordingRIGHT.json";
        }
        return null;
    }
}
