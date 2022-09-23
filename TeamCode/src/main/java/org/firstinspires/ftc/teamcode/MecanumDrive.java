package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends LinearOpMode
{
    HardwareMap_Master masterHardware = new HardwareMap_Master();


    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();
        masterHardware.init(hardwareMap);
        MotorRecorder recorder = new MotorRecorder(runtime, masterHardware, 0.01, telemetry);

        //RESET MOTOR ENCODER THING BRUH
        masterHardware.spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            masterHardware.spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double powerMultiplier = 1;
            if (gamepad1.left_bumper) {
                powerMultiplier = 0.5;
            }

            double inputLX = gamepad1.left_stick_x;
            double inputLY = gamepad1.left_stick_y;
            double inputRX = gamepad1.right_stick_x;

            double d = Math.max(Math.abs(inputLX)+Math.abs(inputLY), 1.0);

            double flPower = Math.pow(((-inputLX + inputLY) - inputRX)/d, 3) * powerMultiplier;
            double blPower = Math.pow(((inputLX + inputLY) - inputRX)/d, 3) * powerMultiplier;

            double frPower = Math.pow(((inputLX + inputLY) + inputRX)/d, 3) * powerMultiplier;
            double brPower = Math.pow(((-inputLX + inputLY) + inputRX)/d, 3) * powerMultiplier;

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

            //---------------------- Gamepad 2 -------------------------------------
            double collectorSpeed = 0.1;
            double spoolSpeed = 0.6;
            double armSpeed = 0.2;
            double carouselSpeed = 0.25;

            double inputRT2 = gamepad2.right_trigger;
            double inputLT2 = gamepad2.left_trigger;
            int inputDPad2 = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);
            int inputBumpers2 = (gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0);
            double inputLY2 = gamepad2.left_stick_y;

            double colPower = Math.pow(inputRT2 - inputLT2, 3) * collectorSpeed;
            double spoolPower = inputDPad2 * spoolSpeed;

            double armPower = inputLY2 * armSpeed;
            double carouselPower = inputBumpers2 * carouselSpeed;

            masterHardware.frontLeft.setPower(flPower);
            masterHardware.frontRight.setPower(frPower);
            masterHardware.backLeft.setPower(blPower);
            masterHardware.backRight.setPower(brPower);
            masterHardware.collector.setPower(colPower);

            //This part is to restrict the spool motor from turning too much and breaking the string
            if(gamepad2.y){
                masterHardware.spool.setPower(spoolPower);
            }
            else {
                if (masterHardware.spool.getCurrentPosition() >= 2700)
                    if (spoolPower > 0)
                        masterHardware.spool.setPower(0);
                    else
                        masterHardware.spool.setPower(spoolPower);
                if (masterHardware.spool.getCurrentPosition() <= 0)
                    if (spoolPower < 0)
                        masterHardware.spool.setPower(0);
                    else
                        masterHardware.spool.setPower(spoolPower);
                else
                    masterHardware.spool.setPower(spoolPower);
            }

            masterHardware.arm.setPower(armPower);
            masterHardware.carousel.setPower(carouselPower);

            recorder.updateData();

            telemetry.addData("Spool Position ", masterHardware.spool.getCurrentPosition());
            telemetry.addData("X Axis", inputLX);
            telemetry.addData("Y Axis", inputLY);
            telemetry.addData("fl", flPower);
            telemetry.addData("fr", frPower);
            telemetry.addData("bl", blPower);
            telemetry.addData("br", brPower);

            telemetry.addData("collector", colPower);
            telemetry.addData("spool", spoolPower);
            telemetry.addData("arm", armPower);
            telemetry.addData("carousel", carouselPower);

            telemetry.update();
        }
    }
}
