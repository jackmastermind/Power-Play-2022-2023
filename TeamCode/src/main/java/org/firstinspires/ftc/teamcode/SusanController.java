package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * CLAW CONTROLLER
 *
 * @author Jack Thompson
 */

public class SusanController
{
    public DcMotor susan;
    public double positionLimit = 36; //Core hex does 288. 36 is ⅛ of that.
    public boolean ignoreMinMax;

    public SusanController(HardwareMap hardwareMap)
    {
        susan = hardwareMap.get(DcMotor.class, "susan");
    }

    public void moveSusan(double input, double speed)
    {
        if (!ignoreMinMax)
        {
            if (input < 0 && susan.getCurrentPosition() > -positionLimit)
            {
                susan.setPower(input * speed);
            }
            else if (input > 0 && susan.getCurrentPosition() < positionLimit)
            {
                susan.setPower(input * speed);
            }
            else
            {
                susan.setPower(0);
            }
        }
        else
        {
            susan.setPower(input * speed);
        }
    }

    public void LogValues(Telemetry telemetry)
    {
        telemetry.addData("susan power", susan.getPower());
        telemetry.addData("susan position", susan.getCurrentPosition());
        telemetry.addData("position limit", "±" + positionLimit);
    }
}

