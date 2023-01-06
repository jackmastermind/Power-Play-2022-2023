package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="GpsMap")
@Disabled
public class GpsMap extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE

        // Variables

        double robotX = 0;
        double robotY = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE eee

            /*
            I want to make a sort of map that displays the position of the robot on a grid
            at all times. This was an idea brought up at the start of Robotics this year.
             */



            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}
