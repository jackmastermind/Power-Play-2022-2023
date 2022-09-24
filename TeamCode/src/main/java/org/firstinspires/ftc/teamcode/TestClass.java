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

@TeleOp(name="TestClass")
public class TestClass extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //INITIALIZATION CODE
        DcMotor Motor1 = hardwareMap.get(DcMotor.class, "driveFL");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE
        Motor1.setPower(0.3);
        Thread.sleep(1000);
        Motor1.setPower(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}

