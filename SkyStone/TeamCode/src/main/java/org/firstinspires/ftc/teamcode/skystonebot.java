package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SkyStone Bot Rocks Mechanum", group="Linear Opmode")

public class skystonebot extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftRearDrive = null;

    private Servo armDrive = null;
    private Servo armExtender = null;
    private Servo claw = null;

    private int maxArmEncoder = 375;
    private double maxArmPower = 0.6;

    private double maxDriveSpeed = 0.25;
    private double strafeSpeed = 0.4;
    private double maxExtender = 1.0;
    private double minExtender = 0.0;
    private double extenderPower = 0.5;
    private double clawClosed = 0.0;
    private double clawOpen = 1.0;
    private double clawPower = 0.3;
    private boolean clawIsOpen = true;

    private boolean armSwitch = false;
    private int driveMode = 3; // 1 = arcade, 2 = tank 3 = strafe
    private boolean clawSwitch = false;
    private boolean extenderSwitch = false;

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
        if (armSwitch) armDrive = hardwareMap.get(Servo.class, "lift");
        if (extenderSwitch) armExtender = hardwareMap.get(Servo.class, "extender");
        if (clawSwitch) claw = hardwareMap.get(Servo.class, "claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);



        if (clawSwitch) claw.setPosition(clawOpen);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower = 0.4;
            double rightPower = 0.4;
            double leftRearPower = 0.4;
            double rightRearPower = 0.4;

            // Arcade Mode uses left stick to go forward, and to turn.
            if (driveMode == 1) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                leftPower = Range.clip(drive + turn, -1.0 * maxDriveSpeed, maxDriveSpeed);
                rightPower = Range.clip(drive - turn, -1.0 * maxDriveSpeed, maxDriveSpeed);
                leftRearPower = leftPower;
                rightRearPower = rightPower;
            }

            // Tank Mode uses one stick to control each wheel.
            if (driveMode == 2) {
                leftPower = -gamepad1.left_stick_y;
                rightPower = -gamepad1.right_stick_y;
                leftRearPower = leftPower;
                rightRearPower = rightPower;
            }

            // Strafe Mode uses the left stick to move forward and turn, and the right stick to strafe sideways.
            if (driveMode == 3) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                double strafe = gamepad1.right_stick_x;
                leftPower = Range.clip(drive + turn + strafe, -1.0 * maxDriveSpeed, maxDriveSpeed);
                rightPower = Range.clip(drive - turn + strafe, -1.0 * 0.48, maxDriveSpeed);
                leftRearPower = Range.clip(drive + turn - strafe, -1.0 * maxDriveSpeed, maxDriveSpeed);
                rightRearPower = Range.clip(drive - turn - strafe, -1.0 * 0.485, maxDriveSpeed);
            }

            if (armSwitch) {
                if (gamepad1.a) {
                    armDrive.setPosition(1.0);
                } else if (gamepad1.y) {
                    armDrive.setPosition(0.0);
                } else armDrive.setPosition(armDrive.getPosition());
            }


            if (extenderSwitch) {
                if (gamepad1.left_bumper) {
                    armExtender.setPosition(maxExtender);
                } else if (gamepad1.right_bumper) {
                    armExtender.setPosition(minExtender);
                } else armExtender.setPosition(armExtender.getPosition());
            }

            if (clawSwitch) {
                if (gamepad1.b) {
                    clawIsOpen = !clawIsOpen;
                    if (clawIsOpen) {
                        claw.setPosition(clawOpen);
                    } else {
                        claw.setPosition(clawClosed);
                    }
                }
            }

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)",  leftRearPower, rightRearPower);
            if (armSwitch) telemetry.addData("Lift Motors", "liftPosition (%f)", armDrive.getPosition());
            if (extenderSwitch) telemetry.addData("Extender Servo", " extenderPosition (%f)", armExtender.getPosition());
            if (clawSwitch) telemetry.addData("Claw", "State: " + clawIsOpen);
            telemetry.update();
        }

    }
}
