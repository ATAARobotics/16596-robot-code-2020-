package OldCode2020.Odometry;

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
        boolean previousBlocky = false;
        boolean previousLeftBumper = false;
        boolean previousRightBumper = false;
        robotui = new RobotInterface(hardwareMap, telemetry, armSwitch, clawSwitch, extenderSwitch, toothSwitch);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
/*
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(robotui,75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
*/
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
                double drive = -1.0 * gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = -1.0 * gamepad1.left_stick_x;
                leftPower = Range.clip(drive + turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                rightPower = Range.clip(drive - turn + strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                leftRearPower = Range.clip(drive + turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
                rightRearPower = Range.clip(drive - turn - strafe, -1.0 * robotui.getMaxDriveSpeed(), robotui.getMaxDriveSpeed());
            }

            if (armSwitch) {
                if (gamepad2.y) {
                    robotui.liftArm();
                } else if (gamepad2.a) {
                    robotui.lowerArm();
                } else robotui.stopArm();
            }

            if (gamepad2.right_trigger >= 0.5 && gamepad2.left_trigger >= 0.5) {
                robotui.parker();
            }
            if (!gamepad2.x && previousBlocky) {
                robotui.blockpush();
            }
            previousBlocky = gamepad2.x;


            if (extenderSwitch) {
                if (gamepad2.left_bumper) {
                    robotui.retractArm();
                } else if (gamepad2.right_bumper) {
                    robotui.extendArm();
                } else robotui.stopExtender();
            }

            if (clawSwitch) {
                if (!gamepad2.b && previousB) {
                    robotui.openClaw();
                }
                previousB = gamepad2.b;
            }

            if (toothSwitch) {
                if (!gamepad1.x && previousX) {
                    robotui.deployTooth();
                }
                previousX = gamepad1.x;
            }
            if (gamepad1.left_bumper && previousLeftBumper) {
                robotui.slow();
            }
            previousLeftBumper = gamepad1.left_bumper;
            if (gamepad1.right_bumper && previousRightBumper) {
                robotui.turbo();
            }
            previousRightBumper = gamepad1.right_bumper;
           /* if (gamepad1.right_trigger >= 0.5) {
                robotui.turboOn();
            } else {
                robotui.turboOff();
            }
*/
            // Send calculated power to wheels
            robotui.drive(leftPower, rightPower, leftRearPower, rightRearPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Accel & Gyro", "%.4f : %.4f : %.4f : %.4f", robotui.getAcceleration().xAccel, robotui.getAcceleration().yAccel, robotui.getAcceleration().zAccel, robotui.getHeading());
//            telemetry.addData("Reported Velocity", "X:%.4f Y:%.4f Z:%.4f", robotui.getVelocity().xVeloc, robotui.getVelocity().yVeloc, robotui.getVelocity().zVeloc);
//            telemetry.addData("Reported Position", "X:%.4f Y:%.4f Z:%.4f", robotui.getPosition().x, robotui.getPosition().y, robotui.getPosition().z);
//            telemetry.addData("Calculated Velocity", "X:%.4f Y:%.4f Z:%.4f", globalPositionUpdate.returnXSpeed(), globalPositionUpdate.returnYSpeed(), globalPositionUpdate.returnZSpeed());
//            telemetry.addData("Calculated Position", "X:%.4f Y:%.4f Z:%.4f", globalPositionUpdate.returnXCoordinate(), globalPositionUpdate.returnYCoordinate(), globalPositionUpdate.returnZCoordinate());
            telemetry.addData("Motors", "Front left (%.2f), Front right (%.2f), Rear left (%.2f), Rear right (%.2f)", leftPower, rightPower, leftRearPower, rightRearPower);
            if (armSwitch)
                telemetry.addData("Lift Motors", "liftPosition (%d) liftmax (%d) liftmin (%d) liftPower (%.2f)", robotui.getCurrentArmPosition(), robotui.MAX_ARM_POSITION, robotui.MIN_ARM_POSITION, robotui.getCurrentArmPower());
            if (extenderSwitch)
                telemetry.addData("Extender Servo", " extenderPosition (%.2f) ", robotui.getExtenderPosition());
            if (clawSwitch)
                telemetry.addData("Claw", "State: " + robotui.isClawIsOpen() + " Position " + robotui.getClawPosition());
            if (toothSwitch) telemetry.addData("Tooth", "State: " + robotui.isToothDeployed());
            telemetry.addData("Color sensor", "red %d, green %d, blue %d", robotui.getColorSensor().red(), robotui.getColorSensor().green(), robotui.getColorSensor().blue());

            telemetry.update();
        }

//        positionThread.stop();
    }
}
