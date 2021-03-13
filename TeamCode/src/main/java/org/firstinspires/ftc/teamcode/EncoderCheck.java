package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    @Autonomous
        public class EncoderCheck extends LinearOpMode{
        private RobotInterface robotui = null;

        @Override
        public void runOpMode() {
            robotui = new RobotInterface(hardwareMap, telemetry, false, false, false);



            waitForStart();

            robotui.resetEncoders();


            while (opModeIsActive())
            {
                telemetry.addLine("Left Front Motor " + Double.toString(robotui.returnValues(RobotInterface.MOTORLIST.LEFTFRONTMOTOR)));
                telemetry.addLine("Right Front Motor " + Double.toString(robotui.returnValues(RobotInterface.MOTORLIST.RIGHTFRONTMOTOR)));
                telemetry.addLine("Left Rear Motor " + Double.toString(robotui.returnValues(RobotInterface.MOTORLIST.LEFTREARMOTOR)));
                telemetry.addLine("Right Rear Motor " + Double.toString(robotui.returnValues(RobotInterface.MOTORLIST.RIGHTREARMOTOR)));
                telemetry.update();


            }
        }
}
