package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends LinearOpMode
{
    MecanumMap_Master master = new MecanumMap_Master();

    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();

        boolean clawOpen = true;
        boolean aDown = false;
        boolean armModeChosen = false;
        boolean useArmPID = false;
        double wristTarget = 0.9;
        double armTarget = 0.0; //Only used for PID mode

        telemetry.setAutoClear(false);
        /*telemetry.addLine("ARM MODE: Press gamepad1.a to use PID arm mode OR gamepad1.b to use manual controls");
        telemetry.update();

        while (opModeInInit())
        {
            if (gamepad1.a)
            {
                armModeChosen = true;
                useArmPID = true;
                break;
            }
            if (gamepad1.b)
            {
                armModeChosen = true;
                break;
            }
        }*/

        if (!armModeChosen)
        {
            telemetry.addLine("warning: useArmPID mode not chosen; defaulting to false");
            telemetry.update();
        }

        telemetry.addData("useArmPID", useArmPID);
        telemetry.update();

        master.init(hardwareMap);
        MotorRecorder recorder = new MotorRecorder(runtime, master, 0.01, telemetry);

        master.clawWrist.setPosition(wristTarget);
        master.clawServo.setPosition(0);

        waitForStart();
        telemetry.setAutoClear(true);
        runtime.reset();
        double lastRuntime = runtime.time();

        while (opModeIsActive()) {

            //region  ------------------------------- Gamepad 1 -------------------------------
            double powerMultiplier = 0.5;
            if (gamepad1.left_bumper) {
                powerMultiplier = 0.25;
            }

            double inputLX = gamepad1.left_stick_x;
            double inputLY = gamepad1.left_stick_y;
            double inputRX = gamepad1.right_stick_x;

            double normalization = Math.max(Math.abs(inputLX) + Math.abs(inputLY) + Math.abs(inputRX), 1.0);

            double flPower = Math.pow(((-inputLX + inputLY) - inputRX)/ normalization, 3) * powerMultiplier;
            double blPower = Math.pow(((inputLX + inputLY) - inputRX)/ normalization, 3) * powerMultiplier;

            double frPower = Math.pow(((inputLX + inputLY) + inputRX)/ normalization, 3) * powerMultiplier;
            double brPower = Math.pow(((-inputLX + inputLY) + inputRX)/ normalization, 3) * powerMultiplier;

            if (gamepad1.right_trigger >= 0.75)
            {
                telemetry.addData("DUMPING MODE", "active");
                String filepath = master.filenameSelect(this);
                if (filepath != null)
                {
                    telemetry.addData("DUMPING MODE", "dumping...");
                    recorder.dumpData(filepath);
                    telemetry.addData("DUMPING MODE", "complete!");
                }
            }
            //endregion

            //region  ------------------------------- Gamepad 2 -------------------------------
            double susanSpeed = 1;
            double wristSpeed = -0.003;

            double susanInput = gamepad2.right_trigger - gamepad2.left_trigger;
            double wristInput = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);

            double susanPower = Math.pow(susanInput * susanSpeed, 3);
            double shoulderPower = 0, elbowPower = 0;

            wristTarget += wristSpeed * wristInput;
            wristTarget = Math.min(1, Math.max(0, wristTarget));

            if (useArmPID)
            {
                double armSpeed = -0.003;
                double armInput = gamepad2.left_stick_y;

                armTarget += armSpeed * armInput;
                armTarget = Math.min(1, Math.max(0, armTarget));
            }
            else
            {
                double shoulderSpeed = 0.2;
                double elbowSpeed = 0.8;

                double shoulderInput = gamepad2.left_stick_y;
                double elbowInput = gamepad2.right_stick_y;

                shoulderPower = Math.pow(shoulderSpeed * shoulderInput, 3);
                elbowPower = Math.pow(elbowSpeed * elbowInput, 3);
            }

            //endregion

            //region ----------------------------- Setting Power -----------------------------
            master.frontLeft.setPower(flPower);
            master.frontRight.setPower(frPower);
            master.backLeft.setPower(blPower);
            master.backRight.setPower(brPower);

            master.susan.setPower(susanPower);

            master.clawWrist.setPosition(wristTarget);

            if (useArmPID)
            {
                double deltaTime = runtime.time() - lastRuntime;
                master.moveArmTowardTarget(armTarget, deltaTime);
            }
            else
            {
                master.shoulderJoint.setPower(shoulderPower);
                master.elbowJoint.setPower(elbowPower);
            }
            lastRuntime = runtime.time();

            if (gamepad2.a && !aDown)
            {
                aDown = true;
                if (clawOpen)
                {
                    master.clawServo.setPosition(0.25);
                    clawOpen = false;
                }
                else
                {
                    master.clawServo.setPosition(0);
                    clawOpen = true;
                }
            }
            else if (!gamepad2.a){
                aDown = false;
            }

            //endregion

            recorder.updateData();

            //region ----------------------------- Telemetry ---------------------------------
            telemetry.addData("fl", flPower);
            telemetry.addData("fr", frPower);
            telemetry.addData("bl", blPower);
            telemetry.addData("br", brPower);
            telemetry.addLine();
            telemetry.addData("susan", susanPower);
            telemetry.addData("armTarget", armTarget);
            telemetry.addData("shoulder", master.shoulderJoint.getPower());
            telemetry.addData("elbow", master.elbowJoint.getPower());
            telemetry.addLine();
            telemetry.addData("claw position", master.clawServo.getPosition());
            telemetry.addData("wristTarget", wristTarget);
            telemetry.addData("wrist position", master.clawWrist.getPosition());
            telemetry.addLine();
            telemetry.addData("[DEBUG] aDown?", aDown);
            telemetry.addData("[DEBUG] clawOpen?", clawOpen);
            telemetry.update();
            //endregion
        }
    }
}
