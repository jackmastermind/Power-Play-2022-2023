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
    
    @Override
    public void runOpMode() {
        //region INITIALIZATION CODE
        robot.init(hardwareMap);
        robot.clawServo.setPosition(Robot.CLAW_CLOSED_POSITION);
        robot.clawWrist.setPosition(0);
        //endregion
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        TelemetryOut telemetryThread = new TelemetryOut();
        telemetryThread.start();

        // 1. Go forward & detect signal cone.
        robot.moveEncoder(Robot.TILE_WIDTH, Robot.AUTO_DRIVE_SPEED);
        String coneSense = "red"; // --> Detect cone here

        // 2. Forward & turn to face high pole.
        robot.moveEncoder(Robot.TILE_WIDTH, Robot.AUTO_DRIVE_SPEED);
        robot.turnEncoder(45, Robot.AUTO_DRIVE_SPEED);

        // 3. Raise & drop preload cone on high pole.
        Robot.runMotorToPosition(robot.spool, Robot.SLIDE_HIGH_POSITION, Robot.AUTO_DRIVE_SPEED);
        robot.clawServo.setPosition(Robot.CLAW_OPEN_POSITION);

        // 4. Grab & drop 2 cones on high pole.
        for (int i = 0; i < 2; i++) {
            // A. Move to stack
            robot.turnEncoder(-135, -Robot.AUTO_DRIVE_SPEED);
            robot.moveEncoder(Robot.TILE_WIDTH, Robot.AUTO_DRIVE_SPEED);

            // B. Grab cone from stack
            Robot.runMotorToPosition(robot.spool, Robot.SLIDE_GRAB_CONE_POSITION, Robot.AUTO_DRIVE_SPEED);
            robot.clawServo.setPosition(Robot.CLAW_CLOSED_POSITION);

            // C. Return to high pole.
            robot.moveEncoder(-Robot.TILE_WIDTH, -Robot.AUTO_DRIVE_SPEED);
            robot.turnEncoder(135, Robot.AUTO_DRIVE_SPEED);

            // D. Raise & drop cone on high pole.
            Robot.runMotorToPosition(robot.spool, Robot.SLIDE_HIGH_POSITION, Robot.AUTO_DRIVE_SPEED);
            robot.clawServo.setPosition(Robot.CLAW_CLOSED_POSITION);
        }

        // 5. Straighten out and park in the appropriate location.
        robot.turnEncoder(45, Robot.AUTO_DRIVE_SPEED);
        switch (coneSense) {
            case "red":
                robot.moveEncoder(Robot.TILE_WIDTH, Robot.AUTO_DRIVE_SPEED);
                break;

            case "green":
                //pass
                break;

            case "blue":
                robot.moveEncoder(-Robot.TILE_WIDTH, -Robot.AUTO_DRIVE_SPEED);
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

