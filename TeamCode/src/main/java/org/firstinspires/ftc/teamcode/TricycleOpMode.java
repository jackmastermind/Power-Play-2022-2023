package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Opmode for simple tricycle
 */

@TeleOp(name="TricycleOpMode")
public class TricycleOpMode extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        DcMotor left = hardwareMap.get(DcMotor.class, "left");
        DcMotor right = hardwareMap.get(DcMotor.class, "right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        //RUN ONE TIME CODE HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            if(gamepad1.right_trigger > 0) //half-speed mode
            {
                left.setPower(0.5*(gamepad1.left_stick_y-gamepad1.left_stick_x));
                right.setPower(0.5*(gamepad1.left_stick_y+gamepad1.left_stick_x));
                telemetry.addLine("u slo");
            }
            else //normal speed
            {
                left.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                right.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                telemetry.addLine("u quik");
            }

            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}

