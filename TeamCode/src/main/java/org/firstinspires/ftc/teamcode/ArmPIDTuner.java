package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="Arm PID Tuner")
public class ArmPIDTuner extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final TwoJointArmController arm = new TwoJointArmController();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        arm.Initialize(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lastRuntime = runtime.time();

        while (opModeIsActive()) {
            
            double tuneSpeed = 0.001;
            
            if (gamepad1.dpad_up) {
                arm.Kp += tuneSpeed;
            }
            else if (gamepad1.dpad_down)
            {
                arm.Kp -= tuneSpeed;
            }
            if (gamepad1.dpad_right)
            {
                arm.Kd += tuneSpeed;
            }
            else if (gamepad1.dpad_left)
            {
                arm.Kd -= tuneSpeed;
            }
            if (gamepad1.right_bumper)
            {
                arm.Ki += tuneSpeed;
            }
            else if (gamepad1.left_bumper)
            {
                arm.Ki -= tuneSpeed;
            }

            double dt = runtime.time() - lastRuntime;
            arm.SetPower(gamepad1, dt, telemetry);
            lastRuntime = runtime.time();

            telemetry.addData("Kp", arm.Kp);
            telemetry.addData("Ki", arm.Ki);
            telemetry.addData("Kd", arm.Kd);
            telemetry.addLine();
            telemetry.addData("shoulderPow", arm.motorJoint1.getPower());
            telemetry.addData("shoulderPos", arm.motorJoint1.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("elbowPow", arm.motorJoint2.getPower());
            telemetry.addData("elbowPos", arm.motorJoint2.getCurrentPosition());
            telemetry.update();

        }
    }
}

