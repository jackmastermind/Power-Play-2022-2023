package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="DiscFlingyJank")
public class DiscFlingyJank extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //INITIALIZATION CODE
        DcMotor bigBoy = hardwareMap.get(DcMotor.class, "bigBoy");

        DcMotor chainBoy = hardwareMap.get(DcMotor.class, "chainBoy");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE
        bigBoy.setTargetPosition(200);
        bigBoy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bigBoy.setPower(0.2);
        while(bigBoy.isBusy()){
            Thread.sleep(100);
        }
        bigBoy.setPower(0);
        bigBoy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            telemetry.addData("Status", "Running");
            telemetry.update();
            chainBoy.setPower(gamepad1.left_stick_y);

            //test comment
        }
    }
}

