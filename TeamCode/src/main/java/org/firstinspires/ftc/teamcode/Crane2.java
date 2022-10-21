package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="Crane2")
@Disabled
public class Crane2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE

        DcMotor Base = hardwareMap.get(DcMotor.class,"Base");
        DcMotor SlideMotor1 = hardwareMap.get(DcMotor.class, "slideOne");
        DcMotor SlideMotor2 = hardwareMap.get(DcMotor.class, "slideTwo");
        Servo Hand = hardwareMap.get(Servo.class, "Hand");

        /*
            Y
         X     B
            A

        Correspond XYAB buttons to slide/hand movements
        up...    gamepad1.y
        down...  gamepad1.a;
        open...  gamepad1.x;
        close... gamepad1.b;
        */

        // variables to be used later
        double slidePower = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // RUN ONE TIME CODE HERE
        //  run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            // Rotate base via bumpers

            Base.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            // Uses the y/a buttons to control the linear slide
            if (gamepad1.y && !gamepad1.a) {
                slidePower = 1;
            } else if (!gamepad1.y && gamepad1.a) {
                slidePower = -1;
            } else {
                slidePower = 0;
            }
            SlideMotor1.setPower(slidePower);
            SlideMotor2.setPower(slidePower);

            // Uses the x/b buttons to control the hand
            if (gamepad1.x && !gamepad1.b){
                Hand.setPosition(1);
            }
            else if (!gamepad1.x && gamepad1.b){
                Hand.setPosition(0);
            }

            //LOOPING CODE HERE
            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}

