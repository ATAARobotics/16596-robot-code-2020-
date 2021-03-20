package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Ultimate Goal Bot Rocks", group="TeleOp")

public class UltimateGoalBot extends LinearOpMode {
    private RobotInterface robotui = null;

    private boolean armSwitch = true;
    private boolean clawSwitch = true;
    private boolean extenderSwitch = true;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean previousB = false;
        boolean previousX = false;
        boolean previousLeftBumper = false;
        boolean previousRightBumper = false;
        boolean extendy = false;
        robotui = new RobotInterface(hardwareMap, telemetry, armSwitch, clawSwitch, extenderSwitch);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower = 0.0;
            double rightPower = 0.0;
            double leftRearPower = 0.0;
            double rightRearPower = 0.0;

            double drive = -1.0 * gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = -1.0 * gamepad1.left_stick_x;
            leftPower = Range.clip(drive + turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            rightPower = Range.clip(drive - turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            leftRearPower = Range.clip(drive + turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            rightRearPower = Range.clip(drive - turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());

            if (armSwitch) {
                if (gamepad2.left_bumper) {
                    robotui.liftArm();
                } else if (gamepad2.right_bumper) {
                    robotui.lowerArm();
                } else robotui.stopArm();
            }

            if (extenderSwitch) {
                if (gamepad2.x) { //left_bumper
                  //  robotui.retractArm();
                //} else if (gamepad2.x) {
                    robotui.extendArm();
                } //else robotui.stopExtender();
            }

            if (clawSwitch) {
                if (!gamepad2.b && previousB) {
                    robotui.openClaw();
                }
                previousB = gamepad2.b;
            }
            if (gamepad1.left_bumper && previousLeftBumper) {
                robotui.slow();
            }
            previousLeftBumper = gamepad1.left_bumper;
            if (gamepad1.right_bumper && previousRightBumper) {
                robotui.turbo();
            }
            previousRightBumper = gamepad1.right_bumper;

            // Send calculated power to wheels
            robotui.drive(leftPower, rightPower, leftRearPower, rightRearPower);
        }

    }
}