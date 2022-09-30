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
        DcMotor MotorFL = hardwareMap.get(DcMotor.class, "driveFL");
        DcMotor MotorFR = hardwareMap.get(DcMotor.class, "driveFR");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE
        int bedtime = 1000;

        // FL motor moves forward at ~1/3 speed for three seconds

        // FL motor moves backwards at full speed for
        // one second, defined by the bedtime variable
        MotorFL.setPower(0.3);
        Thread.sleep(3000);
        MotorFL.setPower(-1);
        Thread.sleep(bedtime);
        MotorFL.setPower(0);

        // FR motor moves forward at half speed for one and a half seconds
        MotorFR.setPower(0.5);
        Thread.sleep(1500);
        MotorFR.setPower(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}

