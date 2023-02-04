package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Master Drive")
@SuppressWarnings("FieldCanBeLocal")
public class MasterDrive extends LinearOpMode
{
    private final MecanumMap_Master mecanum = new MecanumMap_Master();
    private final ElapsedTime runtime = new ElapsedTime();

    private SlideController slide;
    private ClawController clawController;
    private SusanController susanController;

    private DcMotor[] motors;
    private Servo[] servos;

    //MOTOR VARIABLES
    double armSpeed = 1;
    double clawSpeed = 0.003;
    double susanSpeed = 1;

    //TRACKER VARIABLES
    boolean aDown = false;

    public void runOpMode()
    {
        //region INITIALIZE OBJECTS
        mecanum.init(hardwareMap);

        slide           = new SlideController(hardwareMap);
        clawController  = new ClawController(hardwareMap);
        susanController = new SusanController(hardwareMap);

        motors = new DcMotor[] {mecanum.frontLeft, mecanum.frontRight,
                                mecanum.backLeft, mecanum.backRight,
                                slide.spoolMotor, susanController.susan};
        servos = new Servo[]   {clawController.claw, clawController.wrist};

        MotorRecorder recorder = new MotorRecorder(runtime, hardwareMap, motors,
                                                   servos, 0.1, telemetry);
        //endregion

        waitForStart();
        telemetry.setAutoClear(true);
        runtime.reset();

        while(opModeIsActive())
        {
            //region RECORDING DUMP
            if (gamepad1.right_trigger >= 0.75)
            {
                telemetry.addData("DUMPING MODE", "active");
                String filepath = mecanum.filenameSelect(this);
                if (filepath != null)
                {
                    telemetry.addData("DUMPING MODE", "dumping...");
                    recorder.dumpData(filepath);
                    telemetry.addData("DUMPING MODE", "complete!");
                }
            }
            //endregion

            //region DRIVING & OPERATION
            mecanum.drive             (gamepad1);
            slide.MoveSlide           (gamepad2.left_stick_y, armSpeed);
            clawController.moveWrist  (gamepad2.right_stick_y, clawSpeed);
            susanController.moveSusan (gamepad2.right_trigger - gamepad2.left_trigger,
                                       susanSpeed);

            if (gamepad2.a && !aDown)
            {
               clawController.toggleClaw();
            }
            else if (!gamepad2.a){
                aDown = false;
            }
            //endregion

            //region TELEMETRY
            telemetry.addLine("DRIVE MOTORS");
            mecanum.LogValues(telemetry);

            telemetry.addLine("OPERATOR MOTORS");
            clawController  .LogValues(telemetry);
            slide           .LogValues(telemetry);
            susanController .LogValues(telemetry);
            telemetry.update();
            //endregion
        }
    }
}
