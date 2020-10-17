package OldCode2020.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="TurnTest", group="Autonomous")
public class TurnTest extends LinearOpMode {

    private RobotInterface robotui = null;

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

        robotui.turn(90);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        }

    }
}
