package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="ChickenDrive")
public class ChickenDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //INITIALIZATION CODE

        DcMotor MotorLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        DcMotor MotorRight = hardwareMap.get(DcMotor.class, "driveRight");
        MotorRight.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // CHICKEN CODE BELOW HERE


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE

            MotorLeft.setPower(gamepad1.left_stick_y * 0.5);
            MotorRight.setPower(gamepad1.right_stick_y * 0.5);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Wheel Power", gamepad1.left_stick_y);
            telemetry.addData("Right Wheel Power", gamepad1.right_stick_y);

            telemetry.update();


        }
    }
}

