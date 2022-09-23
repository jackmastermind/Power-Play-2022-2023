package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Diff Drive")
public class DiffDrive extends LinearOpMode
{
    HardwareMap_Master masterHardware = new HardwareMap_Master();

    double M1LPower = 0;
    double M1RPower = 0;
    double M2LPower = 0;
    double M2RPower = 0;

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

            //Get Input
            double inputLY = gamepad1.left_stick_y;
            double inputRX = gamepad1.right_stick_x;

            //Determine motor power from input using math!
            double inputMotorLeft = inputLY + inputRX;
            double inputMotorRight = -inputLY + inputRX;

            //If any of the motor power vectors exceed 1, scale BOTH vectors down proportionaly.
            if(Math.abs(inputMotorLeft) > 1 || Math.abs(inputMotorRight) > 1){
                //Get the amount which excess from the vector which exceeds 1
                double i = Math.max(Math.abs(inputMotorLeft), Math.abs(inputMotorRight));

                //Set motor power variables
                M1LPower = inputMotorLeft/i;
                M1RPower = inputMotorRight/i;
                M2LPower = inputMotorLeft/i;
                M2RPower = inputMotorRight/i;
            }
            else{
                //Set motor power variables without scale
                M1LPower = inputMotorLeft;
                M1RPower = inputMotorRight;
                M2LPower = inputMotorLeft;
                M2RPower = inputMotorRight;
            }

            //RECORDING STUFF BUTTONS
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

            //SET MOTOR POWER
            masterHardware.frontLeft.setPower(M1LPower);
            masterHardware.frontRight.setPower(M1RPower);
            masterHardware.backLeft.setPower(M2LPower);
            masterHardware.backRight.setPower(M2RPower);

            recorder.updateData();

            telemetry.addData("Spool Position ", masterHardware.spool.getCurrentPosition());
            telemetry.addData("Right X Axis", inputRX);
            telemetry.addData("Left Y Axis", inputLY);
            telemetry.addData("Motor 1 Left", M1LPower);
            telemetry.addData("Motor 1 Rigt", M1RPower);
            telemetry.addData("Motor 2 Left", M2LPower);
            telemetry.addData("Motor 2 Right", M2RPower);

            /*
            telemetry.addData("collector", colPower);
            telemetry.addData("spool", spoolPower);
            telemetry.addData("arm", armPower);
            telemetry.addData("carousel", carouselPower);
            */

            telemetry.update();
        }
    }
}
