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
    //motor spec page: https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double TICKS_PER_MOTOR_ROTATION = 537.7;
    public static final double TICKS_TO_DEGREES = 360.0 / TICKS_PER_MOTOR_ROTATION;
    public static final double POD_GEAR_RATIO = 17.0 / 68.0;
    public static final double POD_ROTATION_TO_WHEEL_RATIO = 68.0 / 15;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 4;
    //BIG GEAR: 68 teeth
    //LITTLE GEAR: 15
    //EXTERNAL GEAR: 17

    //Proportional constant (counters current error)
    double Kp = 6;
    //Integral constant (counters cumulated error)
    double Ki = 1;
    //Derivative constant (fights oscillation)
    double Kd = 1;

    double integral = 0;

    public double lastError = 0;

    public void initialize(HardwareMap hardwareMap) {
        leftTop = hardwareMap.get(DcMotor.class, "leftTop");
        leftBottom = hardwareMap.get(DcMotor.class, "leftBottom");
        rightTop = hardwareMap.get(DcMotor.class, "rightTop");
        rightBottom = hardwareMap.get(DcMotor.class, "rightBottom");
        motors = new DcMotor[] {leftTop, leftBottom, rightTop, rightBottom};
        //TODO: SOMETHING TO ZERO THE PODS HERE

        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //region Get Position and Rotation Methods
    private double getPodAngle(DcMotor topMotor, DcMotor bottomMotor)
    {
        double netTicks = (topMotor.getCurrentPosition() + bottomMotor.getCurrentPosition()) / 2.0;

        return (netTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO) % (360);
    }

    public double getLeftPodAngle() {
        return getPodAngle(leftTop, leftBottom);
    }

    public double getRightPodAngle()
    {
        return getPodAngle(rightTop, rightBottom);
    }

    private double getPodPosition(DcMotor topMotor, DcMotor bottomMotor)
    {
        double netTicks = topMotor.getCurrentPosition() - bottomMotor.getCurrentPosition();

        return netTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO * POD_ROTATION_TO_WHEEL_RATIO;
    }

    public double getLeftPodPosition()
    {
        return getPodPosition(leftTop, leftBottom);
    }

    public double getRightPodPosition()
    {
        return getPodPosition(rightTop, rightBottom);
    }

    private double getAngularError(double targetDegrees, double angle)
    {
        double output = (targetDegrees - angle) % 360;
        if (output > 180)
        {
            output -= 360;
        }

        return output / 180;
    }

    public double getLeftAngularError(double targetDegrees)
    {
        return getAngularError(targetDegrees, getLeftPodAngle());
    }

    public double getRightAngularError(double targetDegrees)
    {
        return getAngularError(targetDegrees, getRightPodAngle());
    }
    //endregion

    //region Programmatically Set Power & Angle Methods
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

    private void setPodAngle(double angle, double power, DcMotor topMotor, DcMotor bottomMotor) throws InterruptedException {
        double currentAngle = getPodAngle(topMotor, bottomMotor);

        double clockwiseDistance = (angle - currentAngle) % 360;
        if (clockwiseDistance < 0)
        {
            clockwiseDistance += 360;
        }

        double anticlockwiseDistance = (currentAngle - angle) % 360;
        if (anticlockwiseDistance < 0)
        {
            anticlockwiseDistance += 360;
            anticlockwiseDistance *= -1;
        }

        //Take the shorter absolute distance, defaulting to clockwise if they're the same.
        double degreeChange = Math.abs(clockwiseDistance) > Math.abs(anticlockwiseDistance) ? anticlockwiseDistance: clockwiseDistance;

        System.out.println("clockwiseDistance: " + clockwiseDistance);
        System.out.println("anticlockwiseDistance: " + anticlockwiseDistance);
        System.out.println("degreeChange: " + degreeChange);
        double tickChange = degreeChange / TICKS_TO_DEGREES / POD_GEAR_RATIO;

        topMotor.setTargetPosition(((int) Math.round(tickChange)) + topMotor.getCurrentPosition());
        bottomMotor.setTargetPosition(((int) Math.round(tickChange)) + bottomMotor.getCurrentPosition());

        topMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topMotor.setPower(power);
        bottomMotor.setPower(power);

        while (topMotor.isBusy() && bottomMotor.isBusy())
        {
            Thread.sleep(100);
        }

        topMotor.setPower(0);
        bottomMotor.setPower(0);

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLeftAngle(double angle, double power) throws InterruptedException {
        setPodAngle(angle, power, leftTop, leftBottom);
    }

    public void setRightAngle(double angle, double power) throws InterruptedException {
        setPodAngle(angle, power, rightTop, rightBottom);
    }

    public void driveInches(double inches, double power) throws InterruptedException {
        int tickDiff = (int) Math.round(inches / POD_GEAR_RATIO / POD_ROTATION_TO_WHEEL_RATIO / WHEEL_CIRCUMFERENCE * 537.7);
        System.out.println("tickDiff: " + tickDiff);
        leftTop.setTargetPosition(leftTop.getCurrentPosition() + tickDiff);
        rightTop.setTargetPosition(rightTop.getCurrentPosition() + tickDiff);

        leftBottom.setTargetPosition(leftBottom.getCurrentPosition() - tickDiff);
        rightBottom.setTargetPosition(rightBottom.getCurrentPosition() - tickDiff);

        for (DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftTop.setPower(power);
        rightTop.setPower(power);

        leftBottom.setPower(-power);
        rightBottom.setPower(-power);

        while (leftTop.isBusy() || rightTop.isBusy() || leftBottom.isBusy() || rightBottom.isBusy()) {
            Thread.sleep(100);
        }

        setPower(0);

        for (DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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

    /**
     * Uses PID Controller to generate a correction value given an error value.
     * @param error Difference between desired and current value
     * @return Correction value
     */
    public double GetPIDValue(double error, double dt){
        double proportion = Kp * error;
        integral += Ki * error * dt;
        double derivative = Kd * (error - lastError);
        lastError = error;

        return proportion + integral + derivative;
    }

    /**
     * A function used for when motor power exceeds 1. Scaled both motor values appropriately.
     * @param pow1 The power value of one motor
     * @param pow2 The power value of another motor
     * @return An array of the two motor power values values scaled down within 0-1
     */
    public double[] NormalizeScale(double pow1, double pow2){
        //Check if any wheel power value is greater than one
        if(Math.abs(pow1)>1 || Math.abs(pow2)>1){
            double i = Math.max(Math.abs(pow1), Math.abs(pow2)); //Get the abs of the greater power
            return new double[]{pow1/i, pow2/i};
        }
        else{
            return new double[]{pow1, pow2};
        }
    }

    public double StickMagnitude(double a, double b){
        return Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
    }

    public double getStickAngle(Gamepad gamepad1)
    {
        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0)
        {
            return 0;
        }

        double degrees = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) / Math.PI * 180;
        degrees += 180;
        degrees %= 360;

        return degrees;
    }

    public double adjustedAngle(double angle){
        double output = angle;

        if (Math.abs(output) > 90) {
            if (output < 0)
            {
                output += 180;
            }
            else {
                output -= 180;
            }
        }

        return output;
    }

    //endregion

    //region Gamepad Set Power Methods
    /**
     * Used to call both functions for setting the motor powers of each pod
     */
    private void SetPowers(Gamepad gamepad1, double dt){
        SetPod1Powers(gamepad1, dt);
        SetPod2Powers(gamepad1, dt);
    }

    private void SetPodPowers(Gamepad gamepad1, DcMotor topMotor, DcMotor bottomMotor, boolean isLeft, double dt)
    {
        double inputAngle = adjustedAngle(getStickAngle(gamepad1));
        // Invert the magnitude if the robot needs to be driving backward
        double magnitudeInverter = getStickAngle(gamepad1) == inputAngle ? 1: -1;
        double e;

        if (isLeft) {
            e = getLeftAngularError(inputAngle); //Get the error amount (desired value - current value)
        }
        else {
            e = getRightAngularError(inputAngle);
        }

        double inputMagnitude = magnitudeInverter * StickMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y); //Get the input magnitude

        double m1;
        double m2;

        //Changes the motor values depending on if this is the left or right wheel.
        int rightStickMultiplier = isLeft ? 1 : -1;

        //Add the different acceleration amounts for the Wheel, POD, and tank turning into one value.
        //Since tank rotation requires powering both WHEELS separately, we set the power of each motor separately for each left and right pod.
        m1 = -inputMagnitude + GetPIDValue(e, dt) + (gamepad1.right_stick_x * rightStickMultiplier);
        m2 = inputMagnitude + GetPIDValue(e, dt) - (gamepad1.right_stick_x * rightStickMultiplier);


        //Scale the acceleration amount to between -1 to 1
        double[] pows = NormalizeScale(m1, m2);

        double m1Power = pows[0];
        double m2Power = pows[1];

        //TODO: delete this (for testing right now)
        m1Power *= 0.6;
        m2Power *= 0.6;

        topMotor.setPower(m1Power);
        bottomMotor.setPower(m2Power);
    }

    /**
     * Set the power of each motor for Pod 1
     */
    public void SetPod1Powers(Gamepad gamepad1, double dt){
        SetPodPowers(gamepad1, leftTop, leftBottom, true, dt);
    }

    /**
     * Set the power of each motor for Pod 2
     */
    private void SetPod2Powers(Gamepad gamepad1, double dt){
        SetPodPowers(gamepad1, rightTop, rightBottom, false, dt);
    }
    //endregion
}
