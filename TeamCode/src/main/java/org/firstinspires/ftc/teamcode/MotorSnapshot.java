package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.Serializable;


/**
 * A "still frame" of the robot's movement at a given point in time.
 * <p>
 * The MotorSnapshot class is essentially a container for three pieces of data that make the
 * other motor recording classes work: the power levels of the motors at a given point in time, the
 * servo positions at that same point in time, and the timestamp at which those measurements were
 * taken. You'll rarely, if ever, have to construct one of these on your own -- the MotorRecorder
 * and MotorPlayback classes should be able to handle it.
 *
 * @author Jack Thompson, Bubbert Innovations #18351
 * @see MotorRecorder
 * @see MotorPlayback
 * @see Serializable
 */
public class MotorSnapshot implements Serializable {

    private final double timestamp;                //Holds the time of construction.
    private final double[] powerLevels;            //Holds the getPower() value of a group of motors.
    private final double[] servoPositions;         //Holds the getPosition() value of a group of servos.

    /**
     * Construct a new MotorSnapshot with a given timestamp, and motors & servos to access.
     *
     * @param timestamp The time of construction in the opmode runtime.
     * @param motors    The motors to get the power levels from.
     * @param servos    The servos to get the positions from.
     */
    public MotorSnapshot(double timestamp, DcMotor[] motors, Servo[] servos) {
        this.timestamp = timestamp;
        this.powerLevels = new double[motors.length];
        this.servoPositions = new double[servos.length];

        //Loops through motors, adding each power level to the powerLevels array.
        for (int i = 0; i < motors.length; i++) {
            this.powerLevels[i] = motors[i].getPower();
        }

        //Loops through servos, adding each servo position to the servoPositions array.
        for (int i = 0; i < servos.length; i++)
        {
            servoPositions[i] = servos[i].getPosition();
        }
    }

    /**
     * Construct a new MotorSnapshot with a given timestamp, but with powerLevels & servoPositions
     * passed directly, instead of passing DcMotors & Servos then internally calling accessor
     * methods.
     *
     * @param timestamp   The time of construction in the opmode runtime.
     * @param powerLevels The power levels of the motors, passed directly instead of accessed via
     *                    getPower().
     * @param servoPositions The positions of the servos, passed directly instead of accessed via
     *                       getPosition().
     */
    public MotorSnapshot(double timestamp, double[] powerLevels, double[] servoPositions) {
        this.timestamp = timestamp;
        this.powerLevels = powerLevels;
        this.servoPositions = servoPositions;
    }

    /**
     * Return the runtime timestamp of this MotorSnapshot's construction.
     *
     * @return The timestamp field of this MotorSnapshot.
     */
    public double getTimestamp() {
        return timestamp;
    }

    /**
     * Return the power levels recorded at this MotorSnapshot's construction.
     *
     * @return The powerLevels field of this MotorSnapshot.
     */
    public double[] getPowerLevels() {
        return powerLevels;
    }

    /**
     * Return the servo positions recorded at this MotorSnapshot's construction.
     *
     * @return The servoPositions field of this MotorSnapshot.
     */
    public double[] getServoPositions() {
        return servoPositions;
    }

    /**
     * @return true if all the servo positions and power levels are zero, false otherwise
     */
    public boolean isZero()
    {
        for (double level: powerLevels)
        {
            if (level != 0)
            {
                return false;
            }
        }
        for (double position: servoPositions)
        {
            if (position != 0)
            {
                return false;
            }
        }

        return true;
    }
}
