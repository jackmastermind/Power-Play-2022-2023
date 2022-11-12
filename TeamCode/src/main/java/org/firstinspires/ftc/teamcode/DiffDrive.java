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

        while (opModeIsActive()) {

            //PUT STUFF HERE
            diff.SetPod1Powers(gamepad1);

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
            telemetry.addData("PID Value", diff.GetPIDValue(diff.getLeftAngularError(diff.getStickAngle(gamepad1))));
            telemetry.addLine();
            telemetry.addData("stickAngle", diff.getStickAngle(gamepad1));
            telemetry.addData("stickMagnitude", diff.StickMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y));

            telemetry.update();
            //endregion
        }
    }






    /*
    //Proportional constant (counters current error)
    double Kp = 1;
    //Integral constant (counters cumulated error)
    double Ki = 1;
    //Derivative constant (fights oscillation)
    double Kd = 1;

    double value = 0;
    double lastError = 0;

    /**
     * Uses PID Controller to generate a correction value given an error value.
     * @param error Difference between desired and current value
     * @return Correction value
     *
    private double GetPIDValue(double error){
        double proportion = Kp * error;
        double integral = Ki * error;
        double derivative = Kd * (error - lastError);
        lastError = error;

        value = proportion + integral + derivative;

        return value;
    }

    /**
     * A function used for when motor power exceeds 1. Scaled both motor values appropriately.
     * @param pow1 The power value of one motor
     * @param pow2 The power value of another motor
     * @return An array of the two motor power values values scaled down within 0-1
     *
    private double[] NormalizeScale(double pow1, double pow2){
        //Check if any wheel power value is greater than one
        if(pow1>1 || pow2>1){
            double i = Math.max(Math.abs(pow1), Math.abs(pow2)); //Get the abs of the greater power
            double normPow[] = {pow1/i, pow2/i}; //Create a new array of the scaled down powers
            return normPow;
        }
        else{
            double regPow[] = {pow1, pow2}; //Return same values if none exceed 1
            return regPow;
        }
    }

    double StickMagnitude(double a, double b){
        return Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
    }


    /**
     * Used to call both functions for setting the motor powers of each pod
     *
    private void SetPowers(){
        SetPod1Powers();
        SetPod2Powers();
    }

    /**
     * Set the power of each motor for Pod 1
     *
    private void SetPod1Powers(){

        //The "error amount" for the desired POD should be the only variable needed for this part.
        //We could just get it from a get function (if there is one) or add it as an input to
        //this function.

        //Motor1 Power = (normalized) Magnitude of joystick + Angle Correction Value
        //Motor2 Power = (normalized) -Magnitude of joystick + Angle Correction Value

        double error = 0; //REPLACE THIS WITH JACK'S THING!
        double inputMagnitude = StickMagnitude(Math.abs(gamepad1.right_stick_x), Math.abs(gamepad1.right_stick_y));

        //Un-normalized amount
        double P1 = inputMagnitude + GetPIDValue(error);
        double P2 = -inputMagnitude + GetPIDValue(error);

        //TODO: ADD ROTATIONAL STUFF!

        //Normalized amount
        double Motor1Pow = NormalizeScale(P1, P2)[0];
        double Motor2Pow = NormalizeScale(P1, P2)[1];;


        //SET MOTOR POWER
        //masterHardware.frontLeft.setPower(M1LPower);
        //masterHardware.frontRight.setPower(M1RPower);
        //masterHardware.backLeft.setPower(M2LPower);
        //masterHardware.backRight.setPower(M2RPower);
    }

    /**
     * Set the power of each motor for Pod 1
     *
    private void SetPod2Powers(){

        //The "error amount" for the desired POD should be the only variable needed for this part.
        //We could just get it from a get function (if there is one) or add it as an input to
        //this function.

        //Motor1 Power = (normalized) Magnitude of joystick + Angle Correction Value
        //Motor2 Power = (normalized) -Magnitude of joystick + Angle Correction Value

        double error = 0; //REPLACE THIS WITH JACK'S THING!
        double inputMagnitude = StickMagnitude(Math.abs(gamepad1.right_stick_x), Math.abs(gamepad1.right_stick_y));

        //Un-normalized amount
        double P3 = inputMagnitude + GetPIDValue(error);
        double P4 = -inputMagnitude + GetPIDValue(error);

        //TODO: ADD ROTATIONAL STUFF!

        //Normalized amount
        double Motor1Pow = NormalizeScale(P3, P4)[0];
        double Motor2Pow = NormalizeScale(P3, P4)[1];;


        //SET MOTOR POWER
        //masterHardware.frontLeft.setPower(M1LPower);
        //masterHardware.frontRight.setPower(M1RPower);
        //masterHardware.backLeft.setPower(M2LPower);
        //masterHardware.backRight.setPower(M2RPower);
    }
    */
}

