package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="EncoderReadouts")
public class EncoderReadouts extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final MecanumMap_Master master = new MecanumMap_Master();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        master.init(hardwareMap);
        //RUN ONE TIME CODE HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            telemetry.addLine("--- encoder readouts ---");
            telemetry.addData("frontLeft", master.frontLeft.getCurrentPosition());
            telemetry.addData("frontRight", master.frontRight.getCurrentPosition());
            telemetry.addData("backLeft", master.backLeft.getCurrentPosition());
            telemetry.addData("backRight", master.backRight.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("shoulder", master.shoulderJoint.getCurrentPosition());
            telemetry.addData("elbow", master.elbowJoint.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("clawServo", master.clawServo.getPosition());
            telemetry.addData("clawWrist", master.clawWrist.getPosition());
            telemetry.addLine();
            telemetry.addData("Status", "Running");
            telemetry.update();

            //test comment
        }
    }
}

