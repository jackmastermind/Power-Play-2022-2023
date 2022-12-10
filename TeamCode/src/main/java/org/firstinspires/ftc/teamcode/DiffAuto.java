package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@Autonomous(name="DiffAuto")
public class DiffAuto extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final DiffSwerve diff = new DiffSwerve();

    @Override
    public void runOpMode() throws InterruptedException {
        //INITIALIZATION CODE
        diff.initialize(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        TelemetryOut telemetryThread = new TelemetryOut();
        telemetryThread.start();
        //Intended behavior: turn right 90 degrees x 4, then turn left 90 degrees.
        diff.setLeftAngle(90, 0.2);
        Thread.sleep(1000);
        diff.setLeftAngle(180, 0.2);
        Thread.sleep(1000);
        diff.setLeftAngle(270, 0.2);
        Thread.sleep(1000);
        diff.setLeftAngle(0, 0.2);
        Thread.sleep(1000);
        diff.setLeftAngle(270, 0.2);
        //RUN ONE TIME CODE HERE
        telemetryThread.interrupt();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("status", "complete!");
            telemetry.addData("topTicks", diff.leftTop.getCurrentPosition());
            telemetry.addData("bottomTicks", diff.leftBottom.getCurrentPosition());
            telemetry.addData("Pod Angle", diff.getLeftPodAngle());
            telemetry.addLine();
            telemetry.addData("Top Target Position", diff.leftBottom.getTargetPosition());
            telemetry.addData("Bottom Target Position", diff.leftBottom.getTargetPosition());
            telemetry.update();
        }
    }

    class TelemetryOut extends Thread {
        @Override
        public void run() {
            while (opModeIsActive())
            {
                telemetry.addData("status", "running");
                telemetry.addData("topTicks", diff.leftTop.getCurrentPosition());
                telemetry.addData("bottomTicks", diff.leftBottom.getCurrentPosition());
                telemetry.addData("Pod Angle", diff.getLeftPodAngle());
                telemetry.addLine();
                telemetry.addData("Top Target Position", diff.leftBottom.getTargetPosition());
                telemetry.addData("Bottom Target Position", diff.leftBottom.getTargetPosition());
                telemetry.update();
            }
        }
    }
}

