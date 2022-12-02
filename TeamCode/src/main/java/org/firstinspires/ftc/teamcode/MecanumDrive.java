package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends LinearOpMode
{
    HardwareMap_Master masterHardware = new HardwareMap_Master();

    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();
        boolean clawOpen = true;
        boolean aDown = false;
        masterHardware.init(hardwareMap);
        double wristTarget = masterHardware.clawWrist.getPosition();
        MotorRecorder recorder = new MotorRecorder(runtime, masterHardware, 0.01, telemetry);

        masterHardware.clawWrist.setPosition(wristTarget);
        masterHardware.clawServo.setPosition(0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //region  ------------------------------- Gamepad 1 -------------------------------
            double powerMultiplier = 1;
            if (gamepad1.left_bumper) {
                powerMultiplier = 0.5;
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
                String filepath = masterHardware.filenameSelect(this);
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
            double spoolSpeed = 1;
            double armSpeed   = 0.5;
            double wristSpeed = -Math.pow(0.01, 1/3.0);

            double susanInput = gamepad2.right_trigger - gamepad2.left_trigger;
            double spoolInput = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);
            double armInput   = gamepad2.left_stick_y;
            double wristInput = gamepad2.right_stick_y;

            double susanPower = susanInput * susanSpeed;
            double spoolPower = spoolInput * spoolSpeed;
            double armPower   = armInput * armSpeed;

            wristTarget += Math.pow(wristSpeed * wristInput, 3);

            if (wristTarget > 1)
            {
                wristTarget = 1;
            }
            else if (wristTarget < 0)
            {
                wristTarget = 0;
            }

            //endregion

            //region ----------------------------- Setting Power -----------------------------
            masterHardware.frontLeft.setPower(flPower);
            masterHardware.frontRight.setPower(frPower);
            //TODO: currently front wheel only because of mechanical problems.

            // masterHardware.backLeft.setPower(blPower);
            // masterHardware.backRight.setPower(brPower);

            masterHardware.susan.setPower(susanPower);
            masterHardware.spool.setPower(spoolPower);
            masterHardware.arm.setPower(armPower);

            masterHardware.clawWrist.setPosition(wristTarget);

            if (gamepad2.a && !aDown)
            {
                aDown = true;
                if (clawOpen)
                {
                    masterHardware.clawServo.setPosition(0.4);
                    clawOpen = false;
                }
                else
                {
                    masterHardware.clawServo.setPosition(0);
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
            //telemetry.addData("bl", blPower);
            //telemetry.addData("br", brPower);
            telemetry.addLine();
            telemetry.addData("susan", susanPower);
            telemetry.addData("spool", spoolPower);
            telemetry.addData("arm", armPower);
            telemetry.addLine();
            telemetry.addData("claw position", masterHardware.clawServo.getPosition());
            telemetry.addData("wristTarget", wristTarget);
            telemetry.addData("wrist position", masterHardware.clawWrist.getPosition());
            telemetry.addLine();
            telemetry.addData("[DEBUG] aDown?", aDown);
            telemetry.addData("[DEBUG] clawOpen?", clawOpen);
            telemetry.update();
            //endregion
        }
    }
}
