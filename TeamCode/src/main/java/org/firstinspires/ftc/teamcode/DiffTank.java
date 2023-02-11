package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DiffSwerve.POD_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.DiffSwerve.TICKS_TO_DEGREES;

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
        DcMotor m1 = hardwareMap.get(DcMotor.class, "leftTop");
        DcMotor m2 = hardwareMap.get(DcMotor.class, "leftBottom");

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
            telemetry.addLine();
            telemetry.addData("m1Power", m1Power);
            telemetry.addData("m1Ticks", m1.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("m2Power", m2Power);
            telemetry.addData("m2T?icks", m2.getCurrentPosition());
            telemetry.addLine();
            int totalTicks = m1.getCurrentPosition() + m2.getCurrentPosition();
            double degrees = (totalTicks * TICKS_TO_DEGREES * POD_GEAR_RATIO / 2) % 360;
            telemetry.addData("totalTicks", totalTicks);
            telemetry.addData("degrees", degrees);
            telemetry.update();
        }
    }
}

