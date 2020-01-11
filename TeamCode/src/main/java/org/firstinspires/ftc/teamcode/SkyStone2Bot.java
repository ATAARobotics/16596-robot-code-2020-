package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="SkyStone 2 Bot Rocks", group="TeleOp")

public class SkyStone2Bot extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;

    private int driveMode = 3; // 1 = arcade, 2 = tank 3 = strafe

    private boolean armSwitch = true;
    private boolean clawSwitch = true;
    private boolean extenderSwitch = true;
    private boolean toothSwitch = true;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousB = false;
        boolean previousX = false;
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
                double turn = gamepad1.right_stick_x;
                double strafe = -gamepad1.left_stick_x;
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

            if (gamepad2.a) {
                robotui.parker();
            }
            if (gamepad2.x) {
                robotui.blockpush();
            }
            if (gamepad2.y) {
                robotui.unblockpush();
            }

            if (extenderSwitch) {
                if (gamepad1.left_bumper) {
                    robotui.retractArm();
                } else if (gamepad1.right_bumper) {
                    robotui.extendArm();
                } else robotui.stopExtender();
            }

            if (clawSwitch) {
                if (!gamepad1.b && previousB) {
                    robotui.openClaw();
                }
                previousB = gamepad1.b;
            }

            if (toothSwitch) {
                if (!gamepad1.x && previousX) {
                    robotui.deployTooth();
                }
                previousX = gamepad1.x;
            }

            // Send calculated power to wheels
            robotui.drive(leftPower,rightPower, leftRearPower, rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData( "Heading", "%d", )
            telemetry.addData("Motors", "Front left (%.2f), Front right (%.2f), Rear left (%.2f), Rear right (%.2f)", leftPower, rightPower, leftRearPower, rightRearPower);
            if (armSwitch) telemetry.addData("Lift Motors", "liftPosition (%d) liftmax (%d) liftmin (%d) liftPower (%f)", robotui.getCurrentArmPosition(), robotui.MAX_ARM_POSITION, robotui.MIN_ARM_POSITION, robotui.getCurrentArmPower());
            if (extenderSwitch) telemetry.addData("Extender Servo", " extenderPosition (%f) ", robotui.getExtenderPosition());
            if (clawSwitch)  telemetry.addData("Claw", "State: " + robotui.isClawIsOpen() + " Position " + robotui.getClawPosition());
            if (toothSwitch) telemetry.addData("Tooth", "State: " + robotui.isToothDeployed());
            telemetry.addData("Color sensor", "red %d, green %d, blue %d", robotui.getColorSensor().red(), robotui.getColorSensor().green(), robotui.getColorSensor().blue());

            telemetry.update();
        }
    }
}
