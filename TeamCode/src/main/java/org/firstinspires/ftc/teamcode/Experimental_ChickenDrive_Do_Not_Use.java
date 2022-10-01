package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="Experimental_ChickenDrive_Do_Not_Use")
public class Experimental_ChickenDrive_Do_Not_Use extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //INITIALIZATION CODE

        DcMotor MotorLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        DcMotor MotorRight = hardwareMap.get(DcMotor.class, "driveRight");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // CHICKEN CODE BELOW HERE

        float stickXPos = 0;
        float stickYPos = 0;

        float mlPower = 0;
        float mrPower = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE

            stickXPos = gamepad1.right_stick_x;
            stickYPos = gamepad1.right_stick_y;

            mlPower = stickYPos;
            mrPower = stickYPos;

            if (stickXPos < 0) {
                mlPower = mlPower + stickXPos;
                mrPower = mrPower - stickXPos;
            }

            if (stickXPos > 0) {
                mlPower = mlPower - stickXPos;
                mrPower = mrPower + stickXPos;
            }

            if (mlPower > 1) {
                mlPower = 1;
            }

            if (mrPower > 1) {
                mrPower = 1;
            }

            if (mlPower < -1) {
                mlPower = -1;
            }

            if (mrPower < -1) {
                mrPower = -1;
            }

            MotorLeft.setPower(mlPower);
            MotorRight.setPower(mrPower);

            telemetry.addData("Status", "Running");
            telemetry.update();


        }
    }
}

