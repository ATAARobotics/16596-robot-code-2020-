package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SkyStone 2 Bot Rocks", group="TeleOp")

public class SkyStone2Bot extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;

    private int driveMode = 3; // 1 = arcade, 2 = tank 3 = strafe

    private boolean armSwitch = false;
    private boolean clawSwitch = false;
    private boolean extenderSwitch = false;
    private boolean toothSwitch = true;

    @Override
    public void runOpMode() {
        robotui = new RobotInterface(hardwareMap, telemetry, armSwitch, clawSwitch, extenderSwitch, toothSwitch);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower = 0.0;
            double rightPower = 0.0;
            double leftRearPower = 0.0;
            double rightRearPower = 0.0;

            // Arcade Mode uses left stick to go forward, and to turn.
            if (driveMode == 1) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                leftPower = Range.clip(drive + turn, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                rightPower = Range.clip(drive - turn, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
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
                leftPower = Range.clip(drive + turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                rightPower = Range.clip(drive - turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                leftRearPower = Range.clip(drive + turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                rightRearPower = Range.clip(drive - turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            }

            if (armSwitch) {
                if (gamepad1.y) {
                    robotui.liftArm();
                } else if (gamepad1.a) {
                    robotui.lowerArm();
                } else robotui.stopArm();
            }

            if (extenderSwitch) {
                if (gamepad1.left_bumper) {
                    robotui.extendArm();
                } else if (gamepad1.right_bumper) {
                    robotui.retractArm();
                } else robotui.stopExtender();
            }

            if (clawSwitch) {
                if (gamepad1.b) {
                    robotui.openClaw();
                }
            }

            if (toothSwitch) {
                if (gamepad1.x) {
                    robotui.deployTooth();
                }
            }

            // Send calculated power to wheels
            robotui.drive(leftPower,rightPower, leftRearPower, rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Rear Motors", "left (%.2f), right (%.2f)",  leftRearPower, rightRearPower);
            if (armSwitch) telemetry.addData("Lift Motors", "liftPosition (%o) liftPower (%f)", robotui.getCurrentArmPosition(), robotui.getCurrentArmPower());
            if (extenderSwitch) telemetry.addData("Extender Servo", " extenderPosition (%f) ", robotui.getExtenderPosition());
            if (clawSwitch) telemetry.addData("Claw", "State: " + robotui.isClawIsOpen());
            if (toothSwitch) telemetry.addData("Tooth", "State: " + robotui.isToothDeployed());
            telemetry.update();
        }

    }
}
