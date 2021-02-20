package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.RobotInterface;

@Autonomous(name="voodooAuto", group="Autonomous")
public class voodooAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new RobotInterface(hardwareMap, telemetry);

//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
      //  phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

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

        // turn to the bridge

        robotui.drive(0.2,0.2, 48);
        robotui.strafe(0.5, 24);

        //1 46 2 70 4 94 overall square 24 add 2 for compensation




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }
}
