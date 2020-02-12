package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Foundation with Pause", group="Autonomous")
public class AutoFoundationBluePause extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;
    private double location = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new RobotInterface(hardwareMap, telemetry, true, true, true, true);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robotui.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robotui.getCalibrationStatus());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Center on foundation
        robotui.strafe(-0.5,9);

        // backwards 32"
        robotui.drive(0.5, 0.5, -34);

        // Grab foundation
        robotui.deployTooth();

        // adjust for lifting wheel on tooth deploy
        robotui.drive(0.5, 0.5, -1);

        // forwards 32" (plus compensation for slipping)
        robotui.drive(0.5,0.5,42);

        // Let go of foundation
        robotui.deployTooth();

        // wait 5 seconds
        sleep(5000);

        // strafe past the foundation

        robotui.strafe(0.5,34);



        // back away from wall (4")
        robotui.drive(0.5,1.0,-4);

        // lower arm
        robotui.lowerArm(1);

        // strafe under bridge
        robotui.strafeToLine(0.3);

        // Stop arm from going down
        robotui.stopArm();
        // Park under bridge

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }
}
