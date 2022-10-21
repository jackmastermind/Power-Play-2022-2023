package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DiffSwerve {
    public DcMotor leftTop, leftBottom, rightTop, rightBottom;
    public DcMotor[] motors;

    public static final double TICKS_TO_DEGREES = 0; // 360 / ticks per rotation
    public static final double POD_GEAR_RATIO = 0;
    public static final double POD_ROTATION_TO_WHEEL_RATIO = 0;

    public void initialize(HardwareMap hardwareMap) {
        leftTop = hardwareMap.get(DcMotor.class, "leftTop");
        leftBottom = hardwareMap.get(DcMotor.class, "leftBottom");
        rightTop = hardwareMap.get(DcMotor.class, "rightTop");
        rightBottom = hardwareMap.get(DcMotor.class, "rightBottom");
        motors = new DcMotor[] {leftTop, leftBottom, rightTop, rightBottom};
        //DO SOMETHING TO ZERO THE PODS HERE
        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public double getLeftPodRotation() {
        double netTicks = leftTop.getCurrentPosition() + leftBottom.getCurrentPosition() / 2.0;

        return (netTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO) % (360);
    }

    public double getRightPodRotation()
    {
        double netTicks = rightTop.getCurrentPosition() + rightBottom.getCurrentPosition() / 2.0;

        return (netTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO) % (360);
    }

    public double getLeftPodPosition()
    {
        double netTicks = leftBottom.getCurrentPosition() - leftTop.getCurrentPosition();

        return netTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO * POD_ROTATION_TO_WHEEL_RATIO;
    }

    public double getRightPodPosition()
    {
        double netTicks = rightBottom.getCurrentPosition() - rightTop.getCurrentPosition();

        return netTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO * POD_ROTATION_TO_WHEEL_RATIO;
    }

    public void setLeftPower(double power)
    {
        leftBottom.setPower(-power);
        leftTop.setPower(power);
    }

    public void setRightPower(double power)
    {
        rightBottom.setPower(-power);
        rightTop.setPower(power);
    }

    public void setLeftAngularPower(double power)
    {
        leftTop.setPower(power);
        leftBottom.setPower(power);
    }

    public void setRightAngularPower(double power)
    {
        rightTop.setPower(power);
        rightBottom.setPower(power);
    }

    public void setPower(double power)
    {
        setLeftPower(power);
        setRightPower(power);
    }

    public void setAngularPower(double power)
    {
        setLeftAngularPower(power);
        setRightAngularPower(power);
    }
}
