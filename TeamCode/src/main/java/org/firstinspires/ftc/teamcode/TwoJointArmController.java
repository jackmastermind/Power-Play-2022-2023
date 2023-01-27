package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TwoJointArmController {

    public DcMotor motorJoint1, motorJoint2;
    public DcMotor[] motors;

    double j1_desiredPosition; //Joint 1 desired position
    double j2_desiredPosition;

    double j1_Speed = 527.7/3;
    double j2_Speed = 527.7/3;

    //==== PID VARIABLES =====
    //Proportional constant (counters current error)
    double Kp = 0.04;
    //Integral constant (counters cumulated error)
    double Ki = 0;
    //Derivative constant (fights oscillation)
    double Kd = 0;

    double integral = 0;
    public double lastError = 0;
    //=========================

    public void Initialize(HardwareMap hardwareMap) {
        motorJoint1 = hardwareMap.get(DcMotor.class, "shoulder");
        motorJoint2 = hardwareMap.get(DcMotor.class, "elbow");

        motors = new DcMotor[] {motorJoint1, motorJoint2};

        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Set desired position to current position
        j1_desiredPosition = motorJoint1.getCurrentPosition();
        j2_desiredPosition = motorJoint2.getCurrentPosition();
    }

    //Get joint angle

    //Update desired position with input
    void UpdateDesiredPosition(Gamepad gamepad1, double dt){
        UpdateDesiredPosition(gamepad1, dt, null);
    }

    void UpdateDesiredPosition(Gamepad gamepad1, double dt, Telemetry telemetry)
    {
        double input1 = gamepad1.left_stick_y;
        double input2 = gamepad1.right_stick_y;

        j1_desiredPosition += input1 * j1_Speed * dt;
        j2_desiredPosition += input2 * j2_Speed * dt;

        if (telemetry != null)
        {
            telemetry.addLine();
            telemetry.addData("j1_desiredPosition", j1_desiredPosition);
            telemetry.addData("j2_desiredPosition", j2_desiredPosition);
        }
    }

    void SetPower(Gamepad gamepad1, double dt){
        SetPower(gamepad1, dt, null);
    }

    void SetPower(Gamepad gamepad1, double dt, Telemetry telemetry)
    {
        UpdateDesiredPosition(gamepad1, dt, telemetry);

        double error1 = j1_desiredPosition - motorJoint1.getCurrentPosition();
        double error2 = j2_desiredPosition - motorJoint2.getCurrentPosition();

        if (telemetry != null)
        {
            telemetry.addLine();
            telemetry.addData("error1", error1);
            telemetry.addData("error2", error2);
        }

        motorJoint1.setPower(GetPIDValue(error1, dt, telemetry));
        motorJoint2.setPower(GetPIDValue(error2, dt, telemetry));
    }

    /**
     * Uses PID Controller to generate a correction value given an error value.
     * @param error Difference between desired and current value
     * @return Correction value
     */
    public double GetPIDValue(double error, double dt){
        return GetPIDValue(error, dt, null);
    }

    public double GetPIDValue(double error, double dt, Telemetry telemetry){
        double proportion = Kp * error;
        integral += Ki * error * dt;
        double derivative = Kd * (error - lastError);
        lastError = error;

        if (telemetry != null)
        {
            telemetry.addLine();
            telemetry.addData("PIDValue", proportion + integral + derivative);
        }

        return proportion + integral + derivative;
    }

}
