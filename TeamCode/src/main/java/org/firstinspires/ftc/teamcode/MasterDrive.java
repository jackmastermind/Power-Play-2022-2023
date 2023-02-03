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
    private SlideController slide;
    private ClawController clawController;
    private SusanController susanController;

    private DcMotor[] motors;
    private Servo[] servos;

    //MOTOR VARIABLES
    double armSpeed = 1;
    double clawSpeed = 0.003;
    double susanSpeed = 1;

    boolean aDown = false;


    private final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        //INITIALIZE OBJECTS
        mecanum.init(hardwareMap);
        slide = new SlideController(hardwareMap);
        clawController = new ClawController(hardwareMap);
        susanController = new SusanController(hardwareMap);

        motors = new DcMotor[]{mecanum.frontLeft, mecanum.frontRight,
                mecanum.backLeft, mecanum.backRight, slide.spoolMotor, susanController.susan};
        servos = new Servo[]{clawController.claw, clawController.wrist};

        MotorRecorder recorder = new MotorRecorder(runtime, hardwareMap, motors,
                servos, 0.1, telemetry);

        slide.ignoreMinMax = true; //Todo: Remove this once min max are determined
        slide.raiseLinear();

        waitForStart();
        telemetry.setAutoClear(true);
        runtime.reset();

        double lastRuntime = runtime.time();

        while(opModeIsActive())
        {
            double deltaTime = runtime.time() - lastRuntime;

            //region DRIVE SECTION
            mecanum.drive(gamepad1);
            //endregion

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

            //region OPERATING SECTION

            //Arm
            slide.MoveSlide(gamepad2.left_stick_y, armSpeed);

            //Wrist
            clawController.moveWrist(gamepad2.right_stick_y, clawSpeed);

            //Claw
            if (gamepad2.a && !aDown)
            {
               clawController.toggleClaw();
            }
            else if (!gamepad2.a){
                aDown = false;
            }

            //Susan
            susanController.moveSusan(gamepad2.right_trigger - gamepad2.left_trigger,
                    susanSpeed);
            //endregion

            //region TELEMETRY
            telemetry.addLine("DRIVE MOTORS");
            mecanum.LogValues(telemetry);
            telemetry.addLine("OPERATOR MOTORS");
            clawController.LogValues(telemetry);
            slide.LogValues(telemetry);
            susanController.LogValues(telemetry);
            telemetry.update();
            //endregion

            lastRuntime = runtime.time();  //Set last runtime to current runtime
        }
    }
}
