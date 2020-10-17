package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Ultimate Goal Bot Rocks", group="TeleOp")

public class UltimateGoalBot extends LinearOpMode {
    private RobotInterface robotui = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new RobotInterface(hardwareMap, telemetry);


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
            leftPower = Range.clip(     drive + turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            rightPower = Range.clip(drive - turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            leftRearPower = Range.clip(drive + turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            rightRearPower = Range.clip(drive - turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());

            // Send calculated power to wheels
            robotui.drive(leftPower, rightPower, leftRearPower, rightRearPower);
        }

    }
}