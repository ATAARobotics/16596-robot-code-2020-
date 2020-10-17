package OldCode2020.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="SkyStone Bot Rocks Auto 2", group="Autonomous")
public class Autonomus_2 extends LinearOpMode{
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private RobotInterface robotui = null;
        private double location = 0.0;

        @Override
        public void runOpMode() throws InterruptedException {
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                robotui = new RobotInterface(hardwareMap, telemetry, false, false, false, false);

                // Wait for the game to start (driver presses PLAY)
                waitForStart();
                runtime.reset();

                robotui.liftArm();
                // forward 20"
                robotui.drive(1.0, 1.0, 20);
                // scan+strafe for skystone
                robotui.sensorStart();
                robotui.resetEncoders();
                while (!robotui.skyStoneFound()) {
                        robotui.strafe(-1.0);
                }
                robotui.sensorStop();

                // Pick up the block
                robotui.openClaw();
                robotui.lowerArm();
                robotui.openClaw();
                robotui.liftArm();

                location = robotui.getEncoderDistance();

                // backward 5"
                robotui.drive(1.0, 1.0, -5);

                // Strafe 76" under the bridge
                robotui.strafe(1.0, 76 - location);

                // release block
                robotui.openClaw();

                // park under bridge
                robotui.strafe(-1.0, 36);

                // run until the end of the match (driver presses STOP)
                while (opModeIsActive()) {

                }
        }

}
