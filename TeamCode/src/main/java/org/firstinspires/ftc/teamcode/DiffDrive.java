package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Diff Drive")
public class DiffDrive extends LinearOpMode
{
    DiffMap_Master master = new DiffMap_Master();
    DiffSwerve diff = master.diff;

    public void runOpMode() throws InterruptedException {
        master.init(hardwareMap, true);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();

        double lastRuntime = runtime.time();

        while (opModeIsActive()) {

            //Create a deltaTime variable from the current runtime - last runtime
            double deltaTime = runtime.time() - lastRuntime;

            if (gamepad1.dpad_up) {
                diff.Kp += 0.1;
            }
            else if (gamepad1.dpad_down)
            {
                diff.Kp -= 0.1;
            }

            if (gamepad1.dpad_right)
            {
                diff.Kd += 0.1;
            }
            else if (gamepad1.dpad_left)
            {
                diff.Kd -= 0.1;
            }

            if (gamepad1.right_bumper)
            {
                diff.Ki += 0.1;
            }
            else if (gamepad1.left_bumper)
            {
                diff.Ki -= 0.1;
            }


            //PUT STUFF HERE
            diff.SetPod1Powers(gamepad1, deltaTime);

            //region TELEMETRY
            telemetry.addData("leftTop", diff.leftTop.getPower());
            telemetry.addData("leftBottom", diff.leftBottom.getPower());
            telemetry.addData("leftTop ticks", diff.leftTop.getCurrentPosition());
            telemetry.addData("leftBottom ticks", diff.leftBottom.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("leftAngle", diff.getLeftPodAngle());
            telemetry.addLine();
            telemetry.addData("lastError", diff.lastError);
            telemetry.addData("PID Value", diff.GetPIDValue(diff.getLeftAngularError(diff.getStickAngle(gamepad1)), deltaTime));
            telemetry.addLine();
            telemetry.addData("stickAngle", diff.getStickAngle(gamepad1));
            telemetry.addData("stickMagnitude", diff.StickMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y));
            telemetry.addLine();
            telemetry.addData("Kp", diff.Kp);
            telemetry.addData("Ki", diff.Ki);
            telemetry.addData("Kd", diff.Kd);

            telemetry.update();
            //endregion

            //Set last runtime current runtime
            lastRuntime = runtime.time();
        }
    }
}

