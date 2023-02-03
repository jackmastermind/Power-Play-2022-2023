package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * SLIDE CONTROLLER
 * Class for storing methods used for the operating an x-rail slide
 *
 * @author Logan Wood
 */

public class SlideController
{
    //Motor for controlling the spool
    public DcMotor spoolMotor;

    //Values to limit how much the slide will extend
    double minTicks = 0;
    double maxTicks = 0;
    public boolean ignoreMinMax = false;

    public SlideController(HardwareMap hardwareMap)
    {
        spoolMotor = hardwareMap.get(DcMotor.class, "spoolMotor");

        //TODO: I'm not actually sure if all of these are correct
        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetMinMax(double min, double max)
    {
        minTicks = min;
        maxTicks = max;
    }

    /**
     * Moves the connected slide in direction of input
     * <p>Note: Limits for max and min extension are defined by
     * maxTicks and minTicks or disabled/enabled with ignoreMinMax</p>
     * @param input 1D input axis for controlling motor power
     * @param speed Speed multiplier for motor power
     */
    public void MoveSlide(double input, double speed)
    {
        if(!ignoreMinMax)
        {
            //Limit the spool based on minTicks and maxTicks
            if (input > 0 && spoolMotor.getCurrentPosition() < maxTicks)
            {
                spoolMotor.setPower(input * speed);
            }
            else if(input < 0 && spoolMotor.getCurrentPosition() > minTicks)
            {
                spoolMotor.setPower(input * speed);
            }
            else
            {
                spoolMotor.setPower(0);
            }
        }
        else
        {
            //Ignore limits
            spoolMotor.setPower(input * speed);
        }
    }


    public void LogValues(Telemetry telemetry)
    {
        telemetry.addLine("Slide Debug Output");
        telemetry.addData("Slide Motor Position", spoolMotor.getCurrentPosition());
        telemetry.addData("Min", minTicks);
        telemetry.addData("Max", maxTicks);
    }
}

