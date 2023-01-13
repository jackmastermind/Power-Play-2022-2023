package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Diff Drive")
public class DiffDrive extends LinearOpMode
{
    DiffMap_Master master = new DiffMap_Master();

    public void runOpMode() throws InterruptedException {
        boolean chassisOnly = false;
        boolean chassisModeChosen = false;
        double wristTarget = 0.9;
        telemetry.setAutoClear(false);

        telemetry.addLine("Press gamepad1.a to initialize attachments OR gamepad1.b to enter chassisOnly mode:");
        telemetry.update();

        while (opModeInInit())
        {
            if (gamepad1.a) {
                chassisModeChosen = true;
                break;
            }
            if (gamepad1.b)
            {
                chassisOnly = true;
                chassisModeChosen = true;
                break;
            }
        }

        if (!chassisModeChosen)
        {
            telemetry.addLine("warning: chassisOnly mode not chosen; defaulting to false");
            telemetry.update();
        }

        telemetry.addData("chassisOnly", chassisOnly);
        telemetry.update();

        master.init(hardwareMap, chassisOnly);
        ElapsedTime runtime = new ElapsedTime();

        if (!chassisOnly)
        {
            master.wrist.setPosition(wristTarget);
            master.claw.setPosition(0);
        }

        waitForStart();
        runtime.reset();
        telemetry.setAutoClear(true);

        double lastRuntime = runtime.time();

        while (opModeIsActive()) {

            //region controlling DiffSwerve
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

            master.diff.SetPod1Powers(gamepad1, deltaTime);
            master.diff.SetPod2Powers(gamepad1, deltaTime);
            //
            //endregion

            //region controlling attachments
            if (!chassisOnly) {
                double susanSpeed = 0.5;
                double slideSpeed = 0.4;
                double wristSpeed = -0.003;

                double susanInput = gamepad2.right_trigger - gamepad2.left_trigger;
                double spoolInput = gamepad2.left_stick_y;
                double wristInput = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);

                double susanPower = Math.pow(susanInput * susanSpeed, 3);
                double slidePower = Math.pow(spoolInput * slideSpeed, 3);

                wristTarget += wristSpeed * wristInput;
                wristTarget = Math.max(0, Math.min(wristTarget, 1)); //Clamp wristTarget within [0, 1]


                master.susan.setPower(susanPower);
                master.slide.setPower(slidePower);
                master.wrist.setPosition(wristTarget);
            }
            //endregion

            //region DiffSwerve Telemetry
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
            //endregion

            //region attachment Telemetry
            if (!chassisOnly)
            {
                telemetry.addData("susan power", master.susan.getPower());
                telemetry.addData("slide power", master.slide.getPower());
                telemetry.addData("wrist position", master.wrist.getPosition());
                telemetry.addData("claw position", master.claw.getPosition());
            }
            //endregion

            telemetry.update();
            lastRuntime = runtime.time();  //Set last runtime to current runtime
        }
    }
}

