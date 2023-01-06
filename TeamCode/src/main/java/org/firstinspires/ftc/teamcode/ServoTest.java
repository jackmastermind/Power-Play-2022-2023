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
        Servo servo = hardwareMap.get(Servo.class, "servo");
        double servoTarget = servo.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE

            servoTarget += gamepad1.left_stick_y * -0.003;

            if (gamepad1.a)
            {
                servoTarget = 0;
            }
            else if (gamepad1.b)
            {
                servoTarget = 1;
            }

            // Clamp servoTarget between 0 and 1
            servoTarget = Math.max(servoTarget, 0);
            servoTarget = Math.min(servoTarget, 1);

            servo.setPosition(servoTarget);
            telemetry.addData("Status", "Running");
            telemetry.addData("servoTarget", servoTarget);
            telemetry.addData("Servo position", servo.getPosition());
            telemetry.update();

            //test comment
        }
    }
}

