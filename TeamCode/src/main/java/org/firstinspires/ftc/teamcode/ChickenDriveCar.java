package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="ChickenDriveCar")
public class ChickenDriveCar extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorLeft;
    private DcMotor MotorRight;
    private Servo servo;
    private boolean eggBeingDropped = false;

    class EggDropThread extends Thread
    {
        @Override
        public void run() {
            eggBeingDropped = true;
            servo.setPosition(180);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            finally {
                servo.setPosition(0);
            }
            eggBeingDropped = false;
        }
    }

    @Override
    public void runOpMode() {

        //INITIALIZATION CODE

        MotorLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        MotorRight = hardwareMap.get(DcMotor.class, "driveRight");
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0);

        MotorRight.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // CHICKEN CODE BELOW HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            if (gamepad1.right_bumper && !eggBeingDropped)
            {
                EggDropThread eggDropThread = new EggDropThread();
                eggDropThread.start();
            }

            double forwardInput = Math.pow(gamepad1.left_stick_y, 3);
            double turningInput = Math.pow(gamepad1.right_stick_x, 3);

            double mlPower = (forwardInput - turningInput) * 0.5;
            double mrPower = (forwardInput + turningInput) * 0.5;

            MotorLeft.setPower(mlPower);
            MotorRight.setPower(mrPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Wheel Power", mlPower);
            telemetry.addData("Right Wheel Power", mrPower);

            telemetry.update();


        }
    }
}

