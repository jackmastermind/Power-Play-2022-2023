package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// For Qr Code Reader
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


/**
 * The Robot class is a child of HardwareMap_Master, and provides several functions that control
 * the whole robot at once, rather than individual motors or sensors at a time. You declare and
 * initialize it the same way you would a HardwareMap_Master instance.
 *
 * @author Jack Thompson, Bubbert Innovations #18351
 */
public class Robot extends HardwareMap_Master {

    private BNO055IMU imu;          //Internal Motion Unit, built into REV Control Hub

    // For Qr Code Reader
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int qr = 0;
    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    // End Qr Code Reader Stuff

    @Override
    public void init(HardwareMap ahwMap) {
        init(ahwMap, false);
    }



    @Override
    public void init(HardwareMap ahwMap, boolean chassisOnly) {
        super.init(ahwMap, chassisOnly);
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());
        imu.startAccelerationIntegration(new Position(), new Velocity(DistanceUnit.METER, 0, 0, 0, 0), 10 );

        // For Qr Code Reader
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(ahwMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    /**
     * Move a specified distance, using the internal motion unit to sense position.
     *
     * @param inches  The number of inches to move.
     * @param power   The power of the motors while moving.
     * @see BNO055IMU
     * @see DcMotor
     */
    public void moveIMU(double inches, double power) {
        setPower(power);
        double[] startPosition = getXYZPosition();

        if (inches > 0) {
            while (getXYZDistance(startPosition) < inches) {
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                    break;
                }
            }
        }
        else {
            while (getXYZDistance(startPosition) > inches) {
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                    break;
                }
            }
        }

        setPower(0);
    }

    /**
     * Move a specified distance, using drive motor encoders.
     *
     * @param inches  The number of inches to move.
     * @param power   The power of the motors while moving.
     * @see DcMotor
     */
    public void moveEncoder(double inches, double power) {
        //Gets the proper target position with some math & robot specs inherited from hwmap master.
        int target = (int) Math.round((inches / WHEEL_CIRCUMFERENCE_INCHES) * TICKS_PER_ROTATION);

        //Resets encoder ticks, sets the position, and gets the robot moving to the position.
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPosition(target);
        setPower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait for the robot to stop moving
        while (frontLeft.isBusy()) {
            try {
                Thread.sleep(100);
            }
            catch (InterruptedException e) {
                break;
            }
        }

        //Stop the robot, reset encoder ticks, change run mode back to normal.
        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Rotate the robot a certain number of degrees according to the internal motion unit.
     *
     * @param degrees The number of degrees to turn.
     * @param power   The power of the motors while moving.
     * @see DcMotor
     * @see BNO055IMU
     *
     */
    public void turnIMU(float degrees, double power) {
        //According to IMU specs, Z is the vertical axis, zo Z rotation is what we want
        float targetHeading = degrees - (getXYZRotation()[2] % 360);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        if (degrees > 0) {
            while (getXYZRotation()[2] < targetHeading) {
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                    break;
                }
            }
        }
        else {
            while (getXYZRotation()[2] > degrees) {
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                    break;
                }
            }
        }

        setPower(0);
    }

    /**
     * Rotate the robot a certain number of degrees according to the motor encoders.
     *
     * @param degrees The number of degrees to turn.
     * @param power   The power of the motors while moving.
     * @see DcMotor
     */
    public void turnEncoder(float degrees, double power) {
        int target = (int) Math.round((Math.PI * ROBOT_DIAMETER_INCHES / 360)   //Inches per degree
                                      * degrees                                 //times degrees
                                      * (1 / WHEEL_CIRCUMFERENCE_INCHES)        //times rotations per inch
                                      * TICKS_PER_ROTATION);                    //times ticks per rotation
                                                                                //equals ticks.
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontRight.setTargetPosition(-target);
        backRight.setTargetPosition(-target);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait for the robot to stop moving
        while (frontLeft.isBusy()) {
            try {
                Thread.sleep(100);
            }
            catch (InterruptedException e) {
                break;
            }
        }

        //Stop the robot, reset encoder ticks, change run mode back to normal.
        setPower(0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Get the current rotation of the robot, according to the IMU rotation sensor. Unlikely to be
     *  needed by client programmers.
     *
     * @return A float array of the degrees of rotation, in {x, y, z} format.
     * @see BNO055IMU
     * @see Orientation
     */
    public float[] getXYZRotation() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ,
                                                            AngleUnit.DEGREES);

        return new float[] {orientation.firstAngle, orientation.secondAngle,
                            orientation.thirdAngle};
    }

    /**
     * Get the current position of the robot, using a double-integration of the accelerometer. Due
     * to that process, it could be very inaccurate, but testing will tell. Unlikely to be needed by
     * client programmers.
     *
     * @return A double array of the robot's coordinates, in {x, y, z} format.
     * @see BNO055IMU
     * @see Position
     */
    public double[] getXYZPosition() {
        Position position = imu.getPosition().toUnit(DistanceUnit.INCH);

        return new double[] {position.x, position.y, position.z};
    }

    /** Return the distance from the starting position to the current one, using getXYZPosition()
     *  and the Pythagorean theorem.
     *
     * @param startCoords A double array to represent starting coordinates, in {x, y, z} format.
     * @return The distance between the starting position and the robot's current position.
     */
    public double getXYZDistance(double[] startCoords) {
        double[] coords = getXYZPosition();
        double output = 0;
        for (int i = 0; i < startCoords.length; i++) {
            output += Math.pow(coords[i] - startCoords[i], 2);
        }

        return Math.sqrt(output);
    }

    /** Set the power of all drive motors.
     *
     * @param power The specified power level.
     * @see DcMotor
     */
    public void setPower(double power) {
        for (DcMotor motor: motors) {
            motor.setPower(power);
        }
    }

    /** Set the mode of all drive motors.
     *
     * @param mode The specified DcMotor.RunMode.
     * @see DcMotor
     */
    public void setMode(DcMotor.RunMode mode) {
        for (DcMotor motor: motors) {
            motor.setMode(mode);
        }
    }

    /** Set the direction of all drive motors.
     *
     * @param direction The specified DcMotor.Direction.
     * @see DcMotor
     */
    public void setDirection(DcMotor.Direction direction) {
        for (DcMotor motor: motors) {
            motor.setDirection(direction);
        }
    }

    /** Set the target position of all drive motors.
     *
     * @param position The specified position, in encoder ticks.
     * @see DcMotor
     */
    public void setTargetPosition(int position) {
        for (DcMotor motor: motors) {
            motor.setTargetPosition(position);
        }
    }


    // For Qr Code Reader
    /**
     * Finds all qr codes, returns the id of the first one. For our purposes, this will be
     * the integers 1, 2, or 3.
     * @param aprilTagDetectionPipeline
     * @return AprilTag identification
     */
    public int GetQrCode(AprilTagDetectionPipeline aprilTagDetectionPipeline){
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        return detections.get(0).id;

    }
}
