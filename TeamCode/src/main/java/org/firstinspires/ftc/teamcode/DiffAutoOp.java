package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@Autonomous(name="DiffAutoOp")
@Disabled
public class DiffAutoOp extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final DiffSwerve diff = new DiffSwerve();
    private final double TILE_WIDTH = 0;
    private final double DRIVE_SPEED = 0.5;

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

        diff.driveInches(TILE_WIDTH, DRIVE_SPEED);
        String coneSense = "red"; // --> Detect cone here
        diff.driveInches(TILE_WIDTH, DRIVE_SPEED);
        diff.rotateChassis(45, DRIVE_SPEED);
        // --> Raise & drop preload cone

        for (int i = 0; i < 2; i++) {
            diff.rotateChassis(-135, -DRIVE_SPEED);
            diff.driveInches(TILE_WIDTH, DRIVE_SPEED);
            // --> Grab cone from stack [go lower every time]
            diff.driveInches(-TILE_WIDTH, -DRIVE_SPEED);
            diff.rotateChassis(135, DRIVE_SPEED);
            /// --> Raise  & drop come
        }

        diff.rotateChassis(45, DRIVE_SPEED);

        switch (coneSense) {
            case "red":
                diff.driveInches(TILE_WIDTH, DRIVE_SPEED);
                break;

            case "green":
                //pass
                break;

            case "blue":
                diff.driveInches(-TILE_WIDTH, -DRIVE_SPEED);
                break;
        }
    }

    class TelemetryOut extends Thread {
        @Override
        public void run() {
            while (opModeIsActive())
            {
                // PUT TELEMETRY CODE HERE
            }
        }
    }
}