package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TestColorSensor")
public class TestColorSensor extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            String colorSensed;
            if (colorSensor.red() >= colorSensor.blue() && colorSensor.red() >= colorSensor.green())
            {
                colorSensed = "red";
            }
            else if (colorSensor.green() >= colorSensor.blue())
            {
                colorSensed = "green";
            }
            else {
                colorSensed = "blue";
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("color sensed", colorSensed);
            telemetry.update();

            //test comment
        }
    }
}

