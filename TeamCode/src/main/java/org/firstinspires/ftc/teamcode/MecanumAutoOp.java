package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Use this as a template for making new opmodes.
 */

@Autonomous(name="MecanumAutoOp")
public class MecanumAutoOp extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Robot robot = new Robot();
    private final double TILE_WIDTH = 0;
    private final double DRIVE_SPEED = 0.5;
    
    @Override
    public void runOpMode() {
        //INITIALIZATION CODE
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        TelemetryOut telemetryThread = new TelemetryOut();
        telemetryThread.start();

        robot.moveEncoder(TILE_WIDTH, DRIVE_SPEED);
        String coneSense = "red"; // --> Detect cone here
        robot.moveEncoder(TILE_WIDTH, DRIVE_SPEED);
        robot.turnEncoder(45, DRIVE_SPEED);
        // --> Raise & drop preload cone

        for (int i = 0; i < 2; i++) {
            robot.turnEncoder(-135, -DRIVE_SPEED);
            robot.moveEncoder(TILE_WIDTH, DRIVE_SPEED);
            // --> Grab cone from stack [go lower every time]
            robot.moveEncoder(-TILE_WIDTH, -DRIVE_SPEED);
            robot.turnEncoder(135, DRIVE_SPEED);
            /// --> Raise  & drop come
        }

        robot.turnEncoder(45, DRIVE_SPEED);

        switch (coneSense) {
            case "red":
                robot.moveEncoder(TILE_WIDTH, DRIVE_SPEED);
                break;

            case "green":
                //pass
                break;

            case "blue":
                robot.moveEncoder(-TILE_WIDTH, -DRIVE_SPEED);
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

