package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Diff Drive")
public class DiffDrive extends LinearOpMode
{
    DiffMap_Master master = new DiffMap_Master();

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
                master.diff.Kp += 0.1;
            }
            else if (gamepad1.dpad_down)
            {
                master.diff.Kp -= 0.1;
            }

            if (gamepad1.dpad_right)
            {
                master.diff.Kd += 0.1;
            }
            else if (gamepad1.dpad_left)
            {
                master.diff.Kd -= 0.1;
            }

            if (gamepad1.right_bumper)
            {
                master.diff.Ki += 0.1;
            }
            else if (gamepad1.left_bumper)
            {
                master.diff.Ki -= 0.1;
            }


            //PUT STUFF HERE
            master.diff.SetPod1Powers(gamepad1, deltaTime);

            //region TELEMETRY
            telemetry.addData("leftTop", master.diff.leftTop.getPower());
            telemetry.addData("leftBottom", master.diff.leftBottom.getPower());
            telemetry.addData("leftTop ticks", master.diff.leftTop.getCurrentPosition());
            telemetry.addData("leftBottom ticks", master.diff.leftBottom.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("leftAngle", master.diff.getLeftPodAngle());
            telemetry.addLine();
            telemetry.addData("lastError", master.diff.lastError);
            telemetry.addData("PID Value", master.diff.GetPIDValue(master.diff.getLeftAngularError(master.diff.getStickAngle(gamepad1)), deltaTime));
            telemetry.addLine();
            telemetry.addData("stickAngle", master.diff.getStickAngle(gamepad1));
            telemetry.addData("stickMagnitude", master.diff.StickMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y));
            telemetry.addLine();
            telemetry.addData("Kp", master.diff.Kp);
            telemetry.addData("Ki", master.diff.Ki);
            telemetry.addData("Kd", master.diff.Kd);

            telemetry.update();
            //endregion

            //Set last runtime current runtime
            lastRuntime = runtime.time();
        }
    }
}

