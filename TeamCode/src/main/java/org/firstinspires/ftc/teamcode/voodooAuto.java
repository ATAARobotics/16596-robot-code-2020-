package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.RobotInterface;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.opencv.imgproc.Imgproc.RETR_CCOMP;

@Autonomous(name="voodooAuto", group="Autonomous")
public class voodooAuto extends LinearOpMode {
    // Declare OpMode members.
    // OpenCvInternalCamera2 phoneCam;
    // RingAnalysisPipeline pipeline;
    private ElapsedTime runtime = new ElapsedTime();
    private RobotInterface robotui = null;


    @Override
    public void runOpMode() throws InterruptedException {


        robotui = new RobotInterface(hardwareMap, telemetry, true, true, true);
        //   int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //   phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        //   phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            // @Override
            //  public void onOpened()
            {
                //         phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                //       pipeline = new RingAnalysisPipeline();
                //     phoneCam.setPipeline(pipeline);
            }
            //    });

//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            //  phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !robotui.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            // int[] rauri = new int[1];
            // ArrayList<RingOrientationExample.RingAnalysisPipeline.AnalyzedRing> rings;
            // int amountOfZero = 0;
            // int amountOfOne = 0;
            // int amountOfFour = 0;
            // for (int i = 0; i < rauri.length; i++) {
            //   rings = pipeline.getDetectedRings();
            // rauri[i] = robotui.startingField(rings);
          /*  if (rauri[i] == 0){
                amountOfZero++;
            }
            else if (rauri[i] == 1){
                amountOfOne++;
            }
            else if (rauri[i] == 4){
                amountOfFour++;
            } else continue;
        }*/


            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", robotui.getCalibrationStatus());
            //telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

        /*if (amountOfZero > amountOfOne){
            if (amountOfZero > amountOfFour){
                telemetry.addLine("Zero");
                telemetry.update();
                robotui.drive(0.2, 0.2, 10);
                robotui.strafe(0.5, 20);
                robotui.drive(0.2, 0.2, 70);
                robotui.openClaw();

            }
            else{
                telemetry.addLine("Four");
                telemetry.update();
                robotui.drive(0.2, 0.2, 10);
                robotui.strafe(0.2, 48);
                robotui.drive(0.2,0.2, 150);
                robotui.drive(0.2,0.2,-50);
                telemetry.addLine("Four");

            }
        }
        else if (amountOfOne > amountOfFour){
            telemetry.addLine("One");
            telemetry.update();
            robotui.drive(0.2, 0.2, 100);
            robotui.openClaw();
            robotui.drive(0.2, 0.2, -35);


        }
        else{
            telemetry.addLine("Four");
            telemetry.update();
            robotui.drive(0.2, 0.2, 10);
            robotui.strafe(0.2, 48);
            robotui.drive(0.2,0.2, 150);
            robotui.drive(0.2,0.2,-50);

        }*/
            robotui.drive(0.2, 0.2, 10);
            robotui.strafe(0.5, 20);
            robotui.drive(0.2, 0.2, 140);
            robotui.openClaw();
            robotui.drive(0.2, 0.2, -45);



            // turn to the bridge


            //1 46 2 70 4 94 overall square 24 add 2 for compensation


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.update();
            }

        }

    }
}
