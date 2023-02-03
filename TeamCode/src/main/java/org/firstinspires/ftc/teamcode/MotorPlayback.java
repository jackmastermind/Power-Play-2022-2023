package org.firstinspires.ftc.teamcode;

import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Type;
import java.util.ArrayList;

/**
 * A class that takes a collection of MotorSnapshots and recreates the movement of the robot from
 * those snapshots.
 * <p>
 * You can imagine a MotorRecorder as a camera, and each MotorSnapshot as a picture. A MotorPlayback
 * is a film projector, running through each picture to recreate movement. There are two main ways
 * of doing this once the MotorPlayback has been appropriately constructed: calling playAll(), or
 * calling continuePlaying() in a loop that repeats until the MotorPlayback is done. Use the former
 * method if you just want to play back the whole recorded opmode as a single method call; use the
 * latter if you want to be able to run other code (telemetry readings, motor corrections, etc.)
 * while the robot is moving.
 *
 * @author Jack Thompson, Bubbert Innovations #18351
 * @see MotorRecorder
 * @see MotorSnapshot
 */
public class MotorPlayback {
    private static final Type dataType = new TypeToken<ArrayList<MotorSnapshot>>() {}.getType();
    private final ArrayList<MotorSnapshot> data;  //The snapshots recorded by a MotorRecorder
    private final ElapsedTime runtime;            //Runtime of the current OpMode
    private final DcMotor[] motors;               //Motors used in the current HardwareMap_Master config
    private final Servo[] servos;                 //Servos used in the current HardwareMap_Master config
    private int dataIndex;                        //The index of the next snapshot.
    private MotorSnapshot nextSnapshot;           //The next snapshot to play.
    private boolean finished;                     //Has the MotorPlayback run through all its data?

    private final double Kp = 1.0 / 537.7;

    /**
     * Construct a new MotorPlayback object with a HardwareMap_Master.
     *
     * @param filePath The name of the recording file you are trying to access (no need for a full
     *                 file directory).
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param master   A HardwareMap_Master instance to retrieve motor information from.
     */
    public MotorPlayback(String filePath, ElapsedTime runtime, HardwareMap_Master master) {
        this(filePath, runtime, master, null);
    }

    /**
     * Construct a new MotorPlayback manually.
     *
     * @param filePath The name of the recording file you are trying to access (no need for a full
     *                 file directory).
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param hardwareMap HardwareMap to retrieve appContext for file reading.
     * @param motors Array of motors to playback. These must be in the same order that the
     *               MotorRecorder was constructed with.
     * @param servos Array of servos to playback. These must be in the same order that the
     *               MotorRecorder was constructed with.
     */
    public MotorPlayback(String filePath, ElapsedTime runtime, HardwareMap hardwareMap,
                         DcMotor[] motors, Servo[] servos) {
        this(filePath, runtime, hardwareMap, motors, servos, null);
    }

    /**
     * Construct a MotorPlayback object with a Telemetry parameter to output errors to.
     *
     * @param filePath The name of the recording file you are trying to access (no need for a full
     *                 file directory).
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param master   A HardwareMap_Master instance to retrieve motor information from.
     * @param errorLog The Telemetry instance of the current opmode.
     */
    public MotorPlayback(String filePath, ElapsedTime runtime, HardwareMap_Master master,
                         Telemetry errorLog) {
        this(filePath, runtime, master.hwMap, master.motors, master.servos, errorLog);
    }

    /**
     * Construct a new MotorPlayback manually.
     *
     * @param filePath The name of the recording file you are trying to access (no need for a full
     *                 file directory).
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param hardwareMap HardwareMap to retrieve appContext for file reading.
     * @param motors Array of motors to playback. These must be in the same order that the
     *               MotorRecorder was constructed with.
     * @param servos Array of servos to playback. These must be in the same order that the
     *               MotorRecorder was constructed with.
     * @param errorLog The Telemetry instance of the current opmode.
     */
    public MotorPlayback(String filePath, ElapsedTime runtime, HardwareMap hardwareMap,
                         DcMotor[] motors, Servo[] servos, Telemetry errorLog)
    {
        this.data = FileEditor.readJSON(hardwareMap, dataType, filePath, errorLog);
        this.runtime = runtime;
        this.motors = motors;
        this.servos = servos;
        assert data != null;
        this.nextSnapshot = data.get(dataIndex);
    }

    /**
     * Construct a MotorPlayback object directly from an ArrayList of MotorSnapshots, rather than
     * accessing them from a file. Unnecessary for most client programmers.
     *
     * @param data     An ArrayList of MotorSnapshots taken by a MotorRecorder.
     * @param runtime  The ElapsedTime instance of the current opmode.
     * @param master   A HardwareMap_Master instance to retrieve motor information from.
     */
    public MotorPlayback(ArrayList<MotorSnapshot> data, ElapsedTime runtime,
                         HardwareMap_Master master) {
        this.data = data;
        this.runtime = runtime;
        this.motors = master.motors;
        this.servos = master.servos;
        this.dataIndex = 0;
        this.nextSnapshot = data.get(dataIndex);
    }

    /**
     * Play back the next MotorSnapshot in the data if the appropriate timestamp has been reached.
     * Use this in a loop instead of playAll() if you want to be executing other code while the
     * playback is running.
     */
    public void continuePlaying() {
        continuePlaying(0);
    }

    // Play the next MotorSnapshot if the runtime plus a time offset has reached the appropriate timestamp.
    // The purpose of the offset is for playing snippets of a recording, rather than the whole thing.
    private void continuePlaying(double timeOffset)
    {
        if (runtime.time() + timeOffset >= nextSnapshot.getTimestamp()) {
            playNext();
        }
    }

    /**
     * Play back all of the recorded data in its entirety.
     */
    public void playAll(LinearOpMode opMode) {
        while (!finished && opMode.opModeIsActive()) {
            continuePlaying();
        }
    }

    public void playAll(LinearOpMode opMode, boolean trimEnds)
    {
        play(0, Double.MAX_VALUE, trimEnds, opMode);
    }

    /**
     * Play a snippet of the recording, from a certain timestamp to another.
     * @param from Initial timestamp in the recording
     * @param to Final timestamp to play to
     * @param opMode The opmode in which this is being run, to check opModeIsActive()
     */
    public void play(double from, double to, boolean trimEnds, LinearOpMode opMode) throws IllegalArgumentException
    {
        found:  //This block just ensures that if 'from' is an invalid timestamp, an exception will be thrown
        {
            //Go through the data, set the current snapshot to the one just after 'from'
            for (int i = 0; i < data.size(); i++) {
                MotorSnapshot snapshot = data.get(i);
                if (snapshot.getTimestamp() >= from) {
                    // If trimEnds is on and the snapshot is zero, don't stop, keep going
                    if ( !(trimEnds && snapshot.isZero()) ) {
                        dataIndex = i;
                        nextSnapshot = snapshot;
                        from = snapshot.getTimestamp();
                        break found;
                    }
                }
            }
            // This exception will only be thrown if the above loop is not broken - either it's out of bounds,
            // or, if trimEnds is on, it could be that there are no nonzero snapshots after the starting point
            throw new IllegalArgumentException(String.format("Starting timestamp %f not in recording", from));
        }
        for (int i = data.size() - 1; i >= dataIndex; i--)
        {
            MotorSnapshot snapshot = data.get(i);
            if (snapshot.getTimestamp() <= to)
            {
                if ( !(trimEnds && snapshot.isZero()) ) {
                    to = snapshot.getTimestamp();
                }
            }
        }

        double timeOffset = from - runtime.time();

        while (!finished && runtime.time() + timeOffset <= to && opMode.opModeIsActive()) {
            continuePlaying(timeOffset);
        }

    }

    //Set all motors to the power level of the current snapshot & set all servos to the position of
    //the current snapshot
    private void playNext() {
        playNextEncoder();

        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(nextSnapshot.getServoPositions()[i]);
        }

        //Move to the a new snapshot
        if (dataIndex < data.size() - 1) {
            finished = false;
            dataIndex++;
            nextSnapshot = data.get(dataIndex);
        }
        else {
            finished = true;
        }
    }

    private void playNextPower()
    {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(nextSnapshot.getPowerLevels()[i]);
        }
    }

    private void playNextEncoder()
    {
        for (int i = 0; i < motors.length; i++) {
            int error = nextSnapshot.getEncoderPositions()[i] - motors[i].getCurrentPosition();

            motors[i].setPower(Kp * error);
        }
    }

    /**
     * Check if all of the data has been played back.
     *
     * @return This MotorPlayback's finished field.
     */
    public boolean isFinished() {
        return finished;
    }
}
