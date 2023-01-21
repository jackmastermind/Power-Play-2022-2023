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
    private final MecanumMap_Master master = new MecanumMap_Master();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        double armTarget = 0;

        master.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lastRuntime = runtime.time();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger > 0.5)
            {
                telemetry.addData("tweaking", "elbow");

                if (gamepad1.dpad_up) {
                    master.elbowKp += 0.001;
                }
                else if (gamepad1.dpad_down)
                {
                    master.elbowKp -= 0.001;
                }

                if (gamepad1.dpad_right)
                {
                    master.elbowKd += 0.001;
                }
                else if (gamepad1.dpad_left)
                {
                    master.elbowKd -= 0.001;
                }

                if (gamepad1.right_bumper)
                {
                    master.elbowKi += 0.001;
                }
                else if (gamepad1.left_bumper)
                {
                    master.elbowKi -= 0.001;
                }
            }
            else {
                telemetry.addData("tweaking", "shoulder");

                if (gamepad1.dpad_up) {
                    master.shoulderKp += 0.001;
                }
                else if (gamepad1.dpad_down)
                {
                    master.shoulderKp -= 0.001;
                }

                if (gamepad1.dpad_right)
                {
                    master.shoulderKd += 0.001;
                }
                else if (gamepad1.dpad_left)
                {
                    master.shoulderKd -= 0.001;
                }

                if (gamepad1.right_bumper)
                {
                    master.shoulderKi += 0.001;
                }
                else if (gamepad1.left_bumper)
                {
                    master.shoulderKi -= 0.001;
                }
            }


            double armSpeed = -0.003;
            double armInput = gamepad2.left_stick_y;

            armTarget += armSpeed * armInput;
            armTarget = Math.min(1, Math.max(0, armTarget));

            double dt = runtime.time() - lastRuntime;

            master.moveArmTowardTarget(armTarget, dt);

            telemetry.addData("shoulderKp", master.shoulderKp);
            telemetry.addData("shoulderKi", master.shoulderKi);
            telemetry.addData("shoulderKd", master.shoulderKd);
            telemetry.addLine();
            telemetry.addData("elbowKp", master.elbowKp);
            telemetry.addData("elbowKi", master.elbowKi);
            telemetry.addData("elbowKd", master.elbowKd);
            telemetry.addLine();
            telemetry.addData("shoulderPow", master.shoulderJoint.getPower());
            telemetry.addData("shoulderPos", master.shoulderJoint.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("elbowPow", master.elbowJoint.getPower());
            telemetry.addData("elbowPos", master.elbowJoint.getCurrentPosition());
            telemetry.update();

        }
    }
}

