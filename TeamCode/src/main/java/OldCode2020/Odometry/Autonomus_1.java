package OldCode2020.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Autonomous(name="SkyStone Bot Rocks Autonomus", group="Autonomous")
public class Autonomus_1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftRearDrive = null;

    private DcMotor armDrive = null;
    private Servo armExtender = null;
    private Servo claw = null;

    private int maxArmEncoder = 375;
    private double maxArmPower = 0.6;

    private double maxDriveSpeed = 1.0;
    private double strafeSpeed = 1.0;
    private double maxExtender = 0.5;
    private double minExtender = 0.0;
    private double extenderPower = 0.5;
    private double clawClosed = 0.0;
    private double clawOpen = 1.0;
    private double clawPower = 0.3;
    private boolean clawIsOpen = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        armDrive = hardwareMap.get(DcMotor.class, "lift");
        armExtender = hardwareMap.get(Servo.class, "extender");
        claw = hardwareMap.get(Servo.class, "claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        armDrive.setDirection(DcMotor.Direction.FORWARD);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setTargetPosition(0);
        armDrive.setPower(maxArmPower);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(clawOpen);


        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setTargetPosition(0);
        leftDrive.setPower(maxArmPower);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Goes forward at start
        leftDrive.setTargetPosition(399);
        leftDrive.setPower(maxDriveSpeed);
        while (leftDrive.isBusy()) {
            rightDrive.setPower(maxDriveSpeed);
            leftRearDrive.setPower(maxDriveSpeed);
            rightRearDrive.setPower(maxDriveSpeed);
        }

        // Strafe to the left?
        leftDrive.setTargetPosition(500);
        leftDrive.setPower(maxDriveSpeed);
        while (leftDrive.isBusy()) {
            rightDrive.setPower(maxDriveSpeed);
            leftRearDrive.setPower(-1.0*maxDriveSpeed);
            rightRearDrive.setPower(-1.0*maxDriveSpeed);
        }

        armExtender.setPosition(maxExtender);
        while (armExtender.getPosition()<maxExtender) {
            // wait for extender
        }

        claw.setPosition(clawClosed);
        while (claw.getPosition()>clawClosed) {
            // wait for claw
        }

        armDrive.setTargetPosition(maxArmEncoder * -1);
        armDrive.setPower(maxArmPower);
        armExtender.setPosition(minExtender);

        // Strafe to the right?
        leftDrive.setTargetPosition(500);
        leftDrive.setPower(maxDriveSpeed);
        while (leftDrive.isBusy()) {
            rightDrive.setPower(maxDriveSpeed);
            leftRearDrive.setPower(-1.0*maxDriveSpeed);
            rightRearDrive.setPower(-1.0*maxDriveSpeed);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }

    }

    public void retractArm(boolean extend, boolean wait) {
        armExtender.setPosition(maxExtender);
        while (armExtender.getPosition()<maxExtender) {
            // wait for extender
        }

    }

    public void openClaw(boolean open) {
        clawIsOpen = !clawIsOpen;
        if (clawIsOpen) {
            claw.setPosition(clawOpen);
        } else {
            claw.setPosition(clawClosed);
        }
    }
}