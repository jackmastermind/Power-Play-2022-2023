package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="ChickenDriveTank")
@Disabled
public class ChickenDriveTank extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

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
            double mlPower = Math.pow(gamepad1.left_stick_y, 3);
            double mrPower = Math.pow(gamepad1.right_stick_y, 3);

            MotorLeft.setPower(mlPower);
            MotorRight.setPower(mrPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Wheel Power", mlPower);
            telemetry.addData("Right Wheel Power", mrPower);

            telemetry.update();


        }
    }
}

