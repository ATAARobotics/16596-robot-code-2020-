package OldCode2020.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="SkyStone Bot Rocks Auto Simple Blue", group="Autonomous")

public class Autonomous_Simple extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;
    private double location = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robotui = new RobotInterface(hardwareMap, telemetry, true, true, true, true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // forward 25"
        robotui.drive(1.0, 1.0, 25);
        // scan+strafe for skystone

        // Pick up the block
        runtime.reset();
        while(robotui.getCurrentArmPosition() < robotui.MAX_ARM_POSITION - 10) {
            robotui.lowerArm();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw", "State: " + robotui.isClawIsOpen() + " Position " + robotui.getClawPosition());
            telemetry.addData("Lift Motors", "liftPosition (%o) liftmax (%o) liftmin (%o) liftPower (%f)", robotui.getCurrentArmPosition(), robotui.MAX_ARM_POSITION, robotui.MIN_ARM_POSITION, robotui.getCurrentArmPower());
            telemetry.update();
            if(runtime.time() > 5.0) break;
        }
        robotui.stopArm();
        robotui.openClaw();
        runtime.reset();
        while(robotui.getCurrentArmPosition() > robotui.MAX_ARM_POSITION - 17) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw", "State: " + robotui.isClawIsOpen() + " Position " + robotui.getClawPosition());
            robotui.liftArm();
            telemetry.addData("Lift Motors", "liftPosition (%d) liftmax (%d) liftmin (%d) liftPower (%f)", robotui.getCurrentArmPosition(), robotui.MAX_ARM_POSITION, robotui.MIN_ARM_POSITION, robotui.getCurrentArmPower());
            telemetry.update();
            if(runtime.time() > 5.0) break;
        }
        robotui.stopArm();


        // backward 15"
        robotui.drive(1.0, 1.0, -15);

        // turn -10 degrees
        robotui.turn(-10);

        // Strafe 76" under the bridge
        robotui.strafe(1.0, 85);

        // release block
        robotui.openClaw();

        // park under bridge
        robotui.strafe(-1.0, 36);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }
}
