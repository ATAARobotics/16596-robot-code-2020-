package OldCode2020.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="SkyStone Bot Rocks Drive Test", group="Autonomous")
public class DriveSpeedTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotui = new RobotInterface(hardwareMap, telemetry, false, false, false, false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robotui.driveTest(50.0);

    }
}
