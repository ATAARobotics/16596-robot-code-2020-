package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Red Block", group="Autonomous")
public class AutoBlockRedCamera extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;
    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new RobotInterface(hardwareMap, telemetry, true, true, true, true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

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

        // drive forwards until distance is correct
        robotui.driveToBlock(0.5);

        // extend arm
    //    robotui.extendArm(true);
    //    sleep(2000);
    //    robotui.stopExtender();
        // open claw
    //    robotui.openClaw();
        // lower arm
    //    robotui.lowerArm(2);
        // close claw
    //    robotui.openClaw();
        // lift arm
    //    robotui.liftArm(true);

        // block pusher
        robotui.blockpush();

        // drive backwards 10 inches
        robotui.drive(0.5,0.5,-10.0);

        // strafe under the bridge
        robotui.strafeToLine(-0.3);
        robotui.strafe(-0.5,10.0);

        // drop block
      //  robotui.openClaw();
        robotui.unblockpush();

        // strafe to park under the bridge
        robotui.strafeToLine(0.3);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

    }
}
