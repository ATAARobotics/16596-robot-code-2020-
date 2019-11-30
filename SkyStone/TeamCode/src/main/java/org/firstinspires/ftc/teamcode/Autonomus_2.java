package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SkyStone Bot Rocks Auto 2", group="Autonomous")
public class Autonomus_2 extends LinearOpMode{
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private RobotInterface robotui = null;

        @Override
        public void runOpMode() {
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                robotui = new RobotInterface(hardwareMap, telemetry,false,false,false, false);

                // Wait for the game to start (driver presses PLAY)
                waitForStart();
                runtime.reset();

                // forward 24"
                robotui.drive(1.0, 1.0, 24);
                // scan+strafe for skystone
                // TODO
                // pickup block
                // TODO
                // life arm to max
//                robotui.liftArm();
                // backward 5"
                robotui.drive(1.0, 1.0, -5);
                // Strafe 76"
                robotui.turn(-20);
                robotui.strafe(1.0,76);


                // Turn -90 degrees
//                robotui.turn(80);
                // forward 76"
//                robotui.drive(1.0,1.0, 76);
                // Turn 90 degrees
//                robotui.turn(-90);
                // release block
                // TODO

                // run until the end of the match (driver presses STOP)
                while (opModeIsActive()) {

                }
        }


}
