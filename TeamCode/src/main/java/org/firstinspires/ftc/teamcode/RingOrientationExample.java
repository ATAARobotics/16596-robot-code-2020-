package org.firstinspires.ftc.teamcode;


        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        import org.openftc.easyopencv.OpenCvInternalCamera2;
        import org.openftc.easyopencv.OpenCvPipeline;

        import java.util.ArrayList;
        import java.util.Arrays;
        import java.util.List;

        import static org.opencv.imgproc.Imgproc.RETR_CCOMP;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple rings, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@Autonomous
public class RingOrientationExample extends LinearOpMode
{
    OpenCvInternalCamera2 phoneCam;
    RingAnalysisPipeline pipeline;
    private final double MAXSLOPE = 1.0;
    private final double MIDSLOPE = 0.7;
    private RobotInterface robotui = null;


    @Override
    public void runOpMode()
    {
        robotui = new RobotInterface(hardwareMap, telemetry, false, false, false);


        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new RingAnalysisPipeline();
                phoneCam.setPipeline(pipeline);
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
     //   telemetry.setMsTransmissionInterval(20);
        waitForStart();

        int[] rauri = new int[1];
        ArrayList<RingAnalysisPipeline.AnalyzedRing> rings;
        int amountOfZero = 0;
        int amountOfOne = 0;
        int amountOfFour = 0;
        for (int i = 0; i < rauri.length; i++) {
            rings = pipeline.getDetectedRings();
            rauri[i] = startingField(rings);
            if (rauri[i] == 0){
                amountOfZero++;
            }
            else if (rauri[i] == 1){
                amountOfOne++;
            }
            else if (rauri[i] == 4){
                amountOfFour++;
            } else continue;
        }

        if (amountOfZero > amountOfOne){
            if (amountOfZero > amountOfFour){
                telemetry.addLine("Zero");
            }
            else{
                telemetry.addLine("Four");
            }
        }
        else if (amountOfOne > amountOfFour){
            telemetry.addLine("One");
        }
        else{
            telemetry.addLine("Four");
        }


        while (opModeIsActive())
        {
            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(20);



            telemetry.update();









        }
    }

    private int startingField(ArrayList<RingAnalysisPipeline.AnalyzedRing> rings) {
        for (int i = 0; i < rings.size(); i++) {
            telemetry.addLine(Double.toString(rings.get(i).slope));
            if (rings.get(i).slope > MIDSLOPE && rings.get(i).slope < MAXSLOPE){
                return 4;
            }
            else if (rings.get(i).slope <= MIDSLOPE){
                return 1;
            }

        }
        return 0;

    }

    static class RingAnalysisPipeline extends OpenCvPipeline
    {
        /*
         * Our working image buffers
         */
        Mat hsvMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 75;
        static final int CR_CHAN_MASK_THRESHOLD = 225;
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;


        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        /*
         * Colors
         */
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_CHAN_IDX = 2;
        static final int CR_CHAN_IDX = 1;

        static class AnalyzedRing
        {

            double slope;
        }



        ArrayList<AnalyzedRing> internalRingList = new ArrayList<>();
        volatile ArrayList<AnalyzedRing> clientRingList = new ArrayList<>();

        /*
         * Some stuff to handle returning our various buffers
         */
        enum Stage
        {
            FINAL,
            Cb,
            MASK,
            MASK_NR,
            CONTOURS;
        }

        Stage[] stages = Stage.values();

        // Keep track of what stage the viewport is showing
        int stageNum = 0;

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int nextStageNum = stageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageNum = nextStageNum;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            // We'll be updating this with new data below
            internalRingList.clear();

            /*
             * Run the image processing
             */
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            clientRingList = new ArrayList<>(internalRingList);

            /*
             * Decide which buffer to send to the viewport
             */
            switch (stages[stageNum])
            {
                case Cb:
                {
                    return hsvMat;
                }

                case FINAL:
                {
                    return input;
                }

                case MASK:
                {
                    return thresholdMat;
                }

                case MASK_NR:
                {
                    return morphedThreshold;
                }

                case CONTOURS:
                {
                    return contoursOnPlainImageMat;
                }
            }

            return input;
        }

        public ArrayList<AnalyzedRing> getDetectedRings()
        {
            return clientRingList;
        }

        ArrayList<MatOfPoint> findContours(Mat input)
        {
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contours = new ArrayList<>();

            Mat blurredImage = new Mat();
            Mat hsvImage = new Mat();
            Mat mask = new Mat();
            Mat morphOutput = new Mat();
            Mat hierarchy = new Mat();
            Mat maskedImage = new Mat();
            int HUETARGET = 105;
            int HUESTART = HUETARGET - 7;
            int HUESTOP = HUETARGET + 7;
            int SATURATIONSTART = 160;
            int SATURATIONSTOP = 255;
            int VALUESTART = 100;
            int VALUESTOP = 255;
            // remember: H ranges 0-180, S and V range 0-255
            Scalar minValues = new Scalar(HUESTART, SATURATIONSTART, VALUESTART);
            Scalar maxValues = new Scalar(HUESTOP, SATURATIONSTOP, VALUESTOP);

// remove some noise
            Imgproc.blur(input, blurredImage, new Size(7, 7));
            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
            // morphological operators
            Core.inRange(hsvImage, minValues, maxValues, mask);
            // dilate with large element, erode with small ones
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

            Imgproc.erode(mask, morphOutput, erodeElement);
            Imgproc.erode(morphOutput, morphOutput, erodeElement);

            Imgproc.dilate(morphOutput, morphOutput, dilateElement);
            Imgproc.dilate(morphOutput, morphOutput, dilateElement);
            // find contours
            Imgproc.findContours(morphOutput, contours, hierarchy, RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

// if any contour exist...
            if (hierarchy.size().height > 0 && hierarchy.size().width > 0)
            {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0])
                {
                    //Imgproc.drawContours(input, contours, idx, new Scalar(250, 0, 0));
                    Imgproc.drawContours(input, contours, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

                }
            }


            return contours;

        }

        void morphMask(Mat input, Mat output)
        {
            /*
             * Apply some erosion and dilation for noise reduction
             */

            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input)
        {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            AnalyzedRing contourDrawer = new AnalyzedRing();

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);


            // The angle OpenCV gives us can be ambiguous, so look at the shape of
            // the rectangle to fix that.
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
            {
                rotRectAngle += 90;
            }

            // Figure out the slope of a line which would run through the middle, lengthwise
            // (Slope as in m from 'Y = mx + b')
            contourDrawer.slope = rotatedRectFitToContour.size.height / rotatedRectFitToContour.size.width;


            internalRingList.add(contourDrawer);





        }

        static class ContourRegionAnalysis
        {
            /*
             * This class holds the results of analyzeContourRegion()
             */

            double hullArea;
            double contourArea;
            double density;
            List<MatOfPoint> listHolderOfMatOfPoint;
        }

        static ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints)
        {
            // drawContours() requires a LIST of contours (there's no singular drawContour()
            // method), so we have to make a list, even though we're only going to use a single
            // position in it...
            MatOfPoint matOfPoint = new MatOfPoint();
            matOfPoint.fromList(contourPoints);
            List<MatOfPoint> listHolderOfMatOfPoint = Arrays.asList(matOfPoint);

            // Compute the convex hull of the contour
            MatOfInt hullMatOfInt = new MatOfInt();
            Imgproc.convexHull(matOfPoint, hullMatOfInt);

            // Was the convex hull calculation successful?
            if(hullMatOfInt.toArray().length > 0)
            {
                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
                }

                ContourRegionAnalysis analysis = new ContourRegionAnalysis();
                analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;

                // Compute the hull area
                analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));

                // Compute the original contour area
                analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));

                // Compute the contour density. This is the ratio of the contour area to the
                // area of the convex hull formed by the contour
                analysis.density = analysis.contourArea / analysis.hullArea;

                return analysis;
            }
            else
            {
                return null;
            }
        }

        static Point computeDisplacementForSecondPointOfStoneOrientationLine(RotatedRect rect, double unambiguousAngle)
        {
            // Note: we return a point, but really it's not a point in space, we're
            // simply using it to hold X & Y displacement values from the middle point
            // of the bounding rect.
            Point point = new Point();

            // Figure out the length of the short side of the rect
            double shortSideLen = Math.min(rect.size.width, rect.size.height);

            // We draw a line that's 3/4 of the length of the short side of the rect
            double lineLength = shortSideLen * .75;

            // The line is to be drawn at 90 deg relative to the midline running through
            // the rect lengthwise
            point.x = (int) (lineLength * Math.cos(Math.toRadians(unambiguousAngle+90)));
            point.y = (int) (lineLength * Math.sin(Math.toRadians(unambiguousAngle+90)));

            return point;
        }

        static void drawTagText(RotatedRect rect, String text, Mat mat)
        {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x-50,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1); // Font thickness
        }

        static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }



    }




}
/*
Zero
Detect zero rings

Move to A = 11 inches right of robot.
Shoot while moving, maybe stop to shoot.
Move 24 inches right of robot.
Drop wobble once white line detected.
Back up a centimetre so it doesn't touch the wobble
Park

One
Detect one ring

Move to B = 11 inches right of robot.
Shoot while moving, maybe stop to shoot.
Move left 11 inches
Forward to white line
Move right 11 inches
Drop wobble once red line detected. (Maybe move slightly forward depending where wobble is dropped.
Move back till detect white line
Park

Four
Detect four rings

Move to C = 11 inches right of robot
Shoot while moving, maybe stop to shoot.
Move 24 inches right of robot
Drop wobble once red line is detected for the 3rd time. (Maybe move slightly forward depending where wobble is dropped.
Move back till detect white line
Park


Distances to square: 47 70.5 94
Half a tile: 11


 */