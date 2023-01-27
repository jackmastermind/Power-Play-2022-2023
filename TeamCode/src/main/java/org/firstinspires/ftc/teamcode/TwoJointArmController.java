package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoJointArmController {

    public DcMotor motorJoint1, motorJoint2;
    public DcMotor[] motors;

    double j1_desiredPosition; //Joint 1 desired position
    double j2_desiredPosition;

    double j1_Speed = 527.7/4;
    double j2_Speed = 527.7/4;

    //==== PID VARIABLES =====
    //Proportional constant (counters current error)
    double Kp = 6;
    //Integral constant (counters cumulated error)
    double Ki = 1;
    //Derivative constant (fights oscillation)
    double Kd = 1;

    double integral = 0;
    public double lastError = 0;
    //=========================

    public void Initialize(HardwareMap hardwareMap) {
        motorJoint1 = hardwareMap.get(DcMotor.class, "motorJoint1");
        motorJoint2 = hardwareMap.get(DcMotor.class, "motorJoint2");

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
        double input1 = gamepad1.left_stick_y;
        double input2 = gamepad1.right_stick_y;

        j1_desiredPosition += j1_Speed*dt;
        j2_desiredPosition += j2_Speed*dt;
    }

    void SetPower(Gamepad gamepad1, double dt, double speed){
        UpdateDesiredPosition(gamepad1, dt);

        double error1 = j1_desiredPosition - motorJoint1.getCurrentPosition();
        double error2 = j2_desiredPosition - motorJoint2.getCurrentPosition();

        motorJoint1.setPower(GetPIDValue(error1, dt));
        motorJoint2.setPower(GetPIDValue(error2, dt));
    }

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

}
