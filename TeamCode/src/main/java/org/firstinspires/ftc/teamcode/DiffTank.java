package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@TeleOp(name="DiffTank")
@Disabled
public class DiffTank extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        DcMotor m1 = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor m2 = hardwareMap.get(DcMotor.class, "motor2");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //RUN ONE TIME CODE HERE

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //LOOPING CODE HERE
            double m1Power = gamepad1.left_stick_y;
            double m2Power = gamepad1.right_stick_y;

            m1.setPower(m1Power);
            m2.setPower(m2Power);

            telemetry.addData("Status", "Running");

            telemetry.addData("m1Power", m1Power);
            telemetry.addData("m1Ticks:", m1.getCurrentPosition());

            telemetry.addData("m2Power", m2Power);
            telemetry.addData("m2Ticks", m2.getCurrentPosition());
            telemetry.update();
        }
    }
}

