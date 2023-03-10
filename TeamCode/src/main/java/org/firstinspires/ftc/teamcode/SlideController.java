package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public Servo leftLinear, rightLinear;

    //Values to limit how much the slide will extend
    double minTicks = 0;
    double maxTicks = 0;
    public boolean ignoreMinMax = false;

    public double dampValue;

    public SlideController(HardwareMap hardwareMap)
    {
        spoolMotor = hardwareMap.get(DcMotor.class, "spoolMotor");
        leftLinear = hardwareMap.get(Servo.class, "left");
        rightLinear = hardwareMap.get(Servo.class, "right");

        spoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            double pos = Math.abs(spoolMotor.getCurrentPosition());
            double d = -Math.pow(Math.max((pos-0)-maxTicks/2, (maxTicks-pos)-maxTicks/2),2)+1.1;
            dampValue = Math.max(0, Math.min(1, d)); //Clamp the value between 0 and 1

            //Limit the spool based on minTicks and maxTicks
            if (input > 0 && spoolMotor.getCurrentPosition() < maxTicks)
            {
                //double dampValue = (pos - minTicks) / maxTicks;
                spoolMotor.setPower((input * speed)); ///dampValue
            }
            else if(input < 0 && spoolMotor.getCurrentPosition() > minTicks)
            {
                //double dampValue = (maxTicks - pos) / maxTicks;
                spoolMotor.setPower((input * speed));
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

    public void raiseLinear()
    {
        leftLinear.setPosition(0.8);
        rightLinear.setPosition(0.8);
    }

    public void lowerLinear()
    {
        leftLinear.setPosition(0.3);
        rightLinear.setPosition(0.3);
    }

    public void changeLinear(double amount)
    {
        leftLinear.setPosition(amount);
        rightLinear.setPosition(amount);
    }

    public void LogValues(Telemetry telemetry)
    {
        telemetry.addLine("Slide Debug Output");
        telemetry.addData("Slide Motor Position", spoolMotor.getCurrentPosition());
        telemetry.addData("Min", minTicks);
        telemetry.addData("Max", maxTicks);
        telemetry.addData("Damp Value", dampValue);
    }
}

