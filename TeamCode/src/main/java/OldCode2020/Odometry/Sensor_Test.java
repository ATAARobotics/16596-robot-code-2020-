package OldCode2020.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Sensors Test", group="TeleOp")

public class Sensor_Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;

    @Override
    public void runOpMode() {
        robotui = new RobotInterface(hardwareMap, telemetry, false, false, false, false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

         //run until the end of the match (driver presses STOP)
//        robotui.sensorStart();
        while (opModeIsActive()) {
//            telemetry.addData("Color sensor red", robotui.getColorSensor().red());
//            telemetry.addData("Color sensor green", robotui.getColorSensor().green());
//            telemetry.addData("Color sensor blue", robotui.getColorSensor().blue());//
//            telemetry.addData("Color sensor alpha", robotui.getColorSensor().alpha());
//            telemetry.addData("Color sensor argb", robotui.getColorSensor().argb());
//            telemetry.addData( "Detected SkyStone", robotui.skyStoneFound());
            telemetry.addData("Distance", robotui.getDistance());
            telemetry.update();

        }
//        robotui.sensorStop();
    }
}
