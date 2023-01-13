package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Type;
import java.util.ArrayList;


/**
 * A class that records the movement of the robot during an opmode and stores that information in a
 * file, to be replayed by MotorPlayback
 * <p>
 * After construction, all that's needed is to call updateData() in the while(opModeIsActive()) loop
 * to record the robot's movement as an ArrayList of MotorSnapshots, and to call dumpData() when
 * you want to save that ArrayList to a file. Once the data is dumped, a MotorPlayback object will
 * be able to access and replay it.
 *
 * @author Jack Thompson, Bubbert Innovations #18351
 * @see MotorPlayback
 * @see MotorSnapshot
 */

public class MotorRecorder {
    private static final Type dumpType = new TypeToken<ArrayList<MotorSnapshot>>() {}.getType();
    private final ArrayList<MotorSnapshot> data;  //ArrayList of MotorSnapshots
    private final ElapsedTime runtime;            //Runtime of the OpMode
    private final Context context;                //The app context that grants file directory access
    private final DcMotor[] motors;               //Gets power levels from these motors
    private final Servo[] servos;                 //Gets servo position from these servos
    private final double interval;                //How often should it take a snapshot?
    private double lastSnapshotTime;        //The time at which the last snapshot was taken
    private Telemetry errorLog;             //Optional telemetry to output file writing errors to.

    /**
     * Construct a MotorRecorder with a HardwareMap_Master.
     *
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param master   A HardwareMap_Master instance to retrieve motor information from.
     * @param interval How frequently to take snapshots
     */
    public MotorRecorder(ElapsedTime runtime, HardwareMap_Master master, double interval) {
        this(runtime, master.hwMap, master.motors, master.servos, interval);
    }

    /**
     * Construct a MotorRecorder manually, with a list of motors & servos.
     *
     * @param runtime The ElapsedTime instance of the current opmode.
     * @param hardwareMap HardwareMap to get appContext information for file storage.
     * @param motors Motors to record.
     * @param servos Servos to record.
     * @param interval How frequently to take snapshots.
     */
    public MotorRecorder(ElapsedTime runtime, HardwareMap hardwareMap, DcMotor[] motors,
                         Servo[] servos, double interval) {
        this.data = new ArrayList<>();
        this.runtime = runtime;
        this.context = hardwareMap.appContext;
        this.motors = motors;
        this.servos = servos;
        this.interval = interval;
        this.lastSnapshotTime = 0;
    }

    /**
     *
     * Construct a MotorRecorder manually with a telemetry object to log errors to.
     *
     * @param runtime The ElapsedTime instance of the current opmode.
     * @param hardwareMap HardwareMap to get appContext information for file storage.
     * @param motors Motors to record.
     * @param servos Servos to record.
     * @param interval How frequently to take snapshots.
     * @param errorLog A Telemetry instance that errors in file-writing will be logged to.
     */
    public MotorRecorder(ElapsedTime runtime, HardwareMap hardwareMap, DcMotor[] motors,
                         Servo[] servos, double interval, Telemetry errorLog)
    {
        this(runtime, hardwareMap, motors, servos, interval);
        this.errorLog = errorLog;
    }

    /**     * Construct a MotorRecorder with a telemetry object to log errors to.
     *
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param master   A HardwareMap_Master instance to retrieve motor information from.
     * @param interval How frequently to take snapshots
     * @param errorLog A Telemetry instance that errors in file-writing will be logged to.
     */
    public MotorRecorder(ElapsedTime runtime, HardwareMap_Master master, double interval,
                         Telemetry errorLog) {
        this(runtime, master, interval);
        this.errorLog = errorLog;
    }

    /**
     * Take a new MotorSnapshot if enough time (>= interval) has passed since the last one.
     */
    public void updateData() {
        if (runtime.time() - lastSnapshotTime >= interval) {
            lastSnapshotTime = runtime.time();
            takeSnapshot();
        }
    }

    //Adds a MotorSnapshot to the data list.
    private void takeSnapshot() {
        data.add(new MotorSnapshot(runtime.time(), motors, servos));
    }

    /**
     * Dumps data to a file at filePath.
     *
     * @param filePath The name of the .ser file to write to (no need for a full directory)
     */
    public void dumpData(String filePath) {
        FileEditor.writeJSON(data, context, dumpType, filePath, errorLog);
    }

}
