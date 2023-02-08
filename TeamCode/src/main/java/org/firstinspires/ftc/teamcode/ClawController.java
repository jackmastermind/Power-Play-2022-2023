package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * CLAW CONTROLLER
 *
 * @author Jack Thompson
 */

public class ClawController
{
    public Servo wrist, claw;
    private double wristTarget = 0.5;
    private boolean open;
    public final double openPos = 0;
    public final double closePos = 0.28;

    public ClawController(HardwareMap hardwareMap)
    {
        wrist = hardwareMap.get(Servo.class, "clawWrist");
        claw = hardwareMap.get(Servo.class, "clawServo");

        wrist.setPosition(wristTarget);
        closeClaw();
    }

    public void openClaw()
    {
        claw.setPosition(openPos);
        open = true;
    }

    public void closeClaw()
    {
        claw.setPosition(closePos);
        open = false;
    }

    public double getWristTarget() {
        return wristTarget;
    }

    public void moveWrist(double input, double speed)
    {
        wristTarget += input * speed;
        wristTarget = Math.max(0, Math.min(1, wristTarget));

        wrist.setPosition(wristTarget);
    }

    public boolean isOpen() {
        return open;
    }

    public void toggleClaw()
    {
        if (open)
        {
            closeClaw();
        }
        else
        {
            openClaw();
        }
    }

    public void LogValues(Telemetry telemetry)
    {
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("wrist target", wristTarget);
        telemetry.addData("claw position", claw.getPosition());
    }
}

