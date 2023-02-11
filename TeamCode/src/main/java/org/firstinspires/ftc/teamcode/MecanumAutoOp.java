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
@Disabled
public class MecanumAutoOp extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final MecanumMap_Master master = new MecanumMap_Master();
    
    @Override
    public void runOpMode() {
        //region INITIALIZATION CODE
        master.init(hardwareMap);
        master.closeClaw();
        master.clawWrist.setPosition(0);
        //endregion
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        TelemetryOut telemetryThread = new TelemetryOut();
        telemetryThread.start();

        // 1. Go forward & detect signal cone.
        master.moveTiles(1);
        String coneSense = "red"; // --> Detect cone here

        // 2. Forward & turn to face high pole.
        master.moveTiles(1);
        master.turnEncoder(45, MecanumMap_Master.AUTO_DRIVE_SPEED);

        // 3. Raise & drop preload cone on high pole.
        // RAISE ARM
        master.openClaw();
        // 4. Grab & drop 2 cones on high pole.
        for (int i = 0; i < 2; i++) {
            // A. Move to stack
            master.turnEncoder(-135, -MecanumMap_Master.AUTO_DRIVE_SPEED);
            master.moveTiles(1);

            // B. Grab cone from stack
            // LOWER ARM
            master.closeClaw();
            // C. Return to high pole.
            master.moveTiles(-1);
            master.turnEncoder(135, MecanumMap_Master.AUTO_DRIVE_SPEED);

            // D. Raise & drop cone on high pole.
            //RAISE ARM
            master.openClaw();
        }

        // 5. Straighten out and park in the appropriate location.
        master.turnEncoder(45, MecanumMap_Master.AUTO_DRIVE_SPEED);
        switch (coneSense) {
            case "red":
                master.moveEncoder(MecanumMap_Master.TILE_WIDTH, MecanumMap_Master.AUTO_DRIVE_SPEED);
                break;

            case "green":
                //pass
                break;

            case "blue":
                master.moveEncoder(-MecanumMap_Master.TILE_WIDTH, -MecanumMap_Master.AUTO_DRIVE_SPEED);
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

