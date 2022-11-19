package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Test")
public class ServoTest extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE

            if (gamepad1.a)
            {
                clawServo.setPosition(0);
            }
            else if (gamepad1.b)
            {
                clawServo.setPosition(1);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}

