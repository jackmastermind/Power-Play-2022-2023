package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Diff Drive")
public class DiffDrive extends LinearOpMode
{
    //HardwareMap_Master masterHardware = new HardwareMap_Master();

    double M1LPower = 0;
    double M1RPower = 0;
    double M2LPower = 0;
    double M2RPower = 0;

    DiffSwerve diff = new DiffSwerve();

    public void runOpMode() throws InterruptedException {
        diff.initialize(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();
        //masterHardware.init(hardwareMap);
        //MotorRecorder recorder = new MotorRecorder(runtime, masterHardware, 0.01, telemetry);

        //RESET MOTOR ENCODER THING BRUH
        //masterHardware.spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            //RECORDING STUFF BUTTONS
            if (gamepad1.right_trigger >= 0.75)
            {
                telemetry.addData("DUMPING MODE", "active");
                //String filepath = masterHardware.filenameSelect(this);
                /*if (filepath != null)
                {
                    telemetry.addData("DUMPING MODE", "dumping...");
                    recorder.dumpData(filepath);
                    telemetry.addData("DUMPING MODE", "complete!");
                }*/
            }

            //region Set Motor Power and Telemetry
            //SET MOTOR POWER
            /*masterHardware.frontLeft.setPower(M1LPower);
            masterHardware.frontRight.setPower(M1RPower);
            masterHardware.backLeft.setPower(M2LPower);
            masterHardware.backRight.setPower(M2RPower);

            recorder.updateData();*/

            //telemetry.addData("Spool Position ", masterHardware.spool.getCurrentPosition());
            //telemetry.addData("Right X Axis", inputRX);
            //telemetry.addData("Left Y Axis", inputLY);
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

