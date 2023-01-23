package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public double spoolSpeed = 0.4; //Todo: Test this

    //Values to limit how much the slide will extend
    double minTicks = 0;
    double maxTicks = 0;
    public boolean ignoreMinMax = false;

    public void Initialize(HardwareMap hardwareMap)
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
            if(input < 0 && spoolMotor.getCurrentPosition() > minTicks)
            {
                spoolMotor.setPower(input * -speed);
            }
        }
        else
        {
            //Ignore limits
            spoolMotor.setPower(input*spoolSpeed);
        }
    }
}

