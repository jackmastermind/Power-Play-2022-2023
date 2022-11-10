package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Class for storing methods used for the DiffDrive OpMode
 *
 * @author Logan Wood
 * @author Jack Thompson
 */
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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //region Get Position and Rotation Methods
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
    //endregion

    //region Jack's SetPower Methods (I think)
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

    //endregion

    //region Number Manipulation Methods

    //Proportional constant (counters current error)
    double Kp = 1;
    //Integral constant (counters cumulated error)
    double Ki = 1;
    //Derivative constant (fights oscillation)
    double Kd = 1;

    double value = 0;
    double lastError = 0;

    /**
     * Uses PID Controller to generate a correction value given an error value.
     * @param error Difference between desired and current value
     * @return Correction value
     */
    public double GetPIDValue(double error){
        double proportion = Kp * error;
        double integral = Ki * error;
        double derivative = Kd * (error - lastError);
        lastError = error;

        value = proportion + integral + derivative;

        return value;
    }

    /**
     * A function used for when motor power exceeds 1. Scaled both motor values appropriately.
     * @param pow1 The power value of one motor
     * @param pow2 The power value of another motor
     * @return An array of the two motor power values values scaled down within 0-1
     */
    public double[] NormalizeScale(double pow1, double pow2){
        //Check if any wheel power value is greater than one
        if(pow1>1 || pow2>1){
            double i = Math.max(Math.abs(pow1), Math.abs(pow2)); //Get the abs of the greater power
            double normPow[] = {pow1/i, pow2/i}; //Create a new array of the scaled down powers
            return normPow;
        }
        else{
            double regPow[] = {pow1, pow2}; //Return same values if none exceed 1
            return regPow;
        }
    }

    public double StickMagnitude(double a, double b){
        return Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
    }

    public double getStickAngle(Gamepad gamepad1)
    {
        double degrees = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180;
        if (degrees < 0)
        {
            degrees += 360;
        }
        return degrees;
    }

    //endregion

    //region Set Power Methods
    /**
     * Used to call both functions for setting the motor powers of each pod
     */
    private void SetPowers(Gamepad gamepad1){
        SetPod1Powers(gamepad1);
        SetPod2Powers(gamepad1);
    }

    public double getLeftRotationalError(double targetDegrees)
    {
        return (targetDegrees - getLeftPodRotation()) % 360;
    }

    public double getRightRotationalError(double targetDegrees)
    {
        return (targetDegrees - getRightPodRotation()) % 360;
    }

    /**
     * Set the power of each motor for Pod 1
     */
    public void SetPod1Powers(Gamepad gamepad1){
        double inputAngle = getStickAngle(gamepad1);

        double e = getLeftRotationalError(Math.toDegrees(inputAngle));
        //diff.GetPIDValue(e)
        double inputMagnitude = StickMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y);

        double m1 = inputMagnitude + GetPIDValue(e);
        double m2 = -inputMagnitude + GetPIDValue(e);

        double[] pows = NormalizeScale(m1, m2);

        double m1Power = pows[0];
        double m2Power = pows[1];

        leftTop.setPower(m1Power);
        leftBottom.setPower(m2Power);
    }

    /**
     * Set the power of each motor for Pod 1
     */
    private void SetPod2Powers(Gamepad gamepad1){

        //The "error amount" for the desired POD should be the only variable needed for this part.
        //We could just get it from a get function (if there is one) or add it as an input to
        //this function.

        //Motor1 Power = (normalized) Magnitude of joystick + Angle Correction Value
        //Motor2 Power = (normalized) -Magnitude of joystick + Angle Correction Value

        double error = 0; //REPLACE THIS WITH JACK'S THING!
        double inputMagnitude = StickMagnitude(Math.abs(gamepad1.right_stick_x), Math.abs(gamepad1.right_stick_y));

        //Un-normalized amount
        double P3 = inputMagnitude + GetPIDValue(error);
        double P4 = -inputMagnitude + GetPIDValue(error);

        //TODO: ADD ROTATIONAL STUFF!

        //Normalized amount
        double Motor1Pow = NormalizeScale(P3, P4)[0];
        double Motor2Pow = NormalizeScale(P3, P4)[1];;


        //SET MOTOR POWER
        //masterHardware.frontLeft.setPower(M1LPower);
        //masterHardware.frontRight.setPower(M1RPower);
        //masterHardware.backLeft.setPower(M2LPower);
        //masterHardware.backRight.setPower(M2RPower);
    }
    //endregion
}
