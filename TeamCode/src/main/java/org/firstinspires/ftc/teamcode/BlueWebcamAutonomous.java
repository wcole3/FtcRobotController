/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Blue Webcam Autonomous")
public class BlueWebcamAutonomous extends LinearOpMode
{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    OpenCvWebcam webcam;
    private final int webcam_width = 320;
    private final int webcam_height = 240;

    // this is the threshold +/- for the color scalar in the pipeline
    private final int threshold_size = 20;

    boolean is_streaming = true;

    /**
     * Bc we're almost out of time we're going to try a very simple algorithm:
     *
     * We are going to collect frames and find the centroid of the blue pixels and determine
     * if it is straight ahead, left, or right
     *
     * We will count 30 frames to hopefully average out any noise
     */
    private int FRAMES_TO_COLLECT = 30;
    private int frames_collected = 0;
    // define the left right and center regions as rects
    private Rect left = new Rect(0, 0, webcam_width/3, webcam_height);
    private Rect center = new Rect(webcam_width/3, 0, webcam_width/3, webcam_height);
    private Rect right = new Rect(2*webcam_width/3, 0, webcam_width/3, webcam_height);

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo wristServo = null;

    private Servo intakeServo = null;
    private Servo hugServo = null;

    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;

    /**
     * Constants
     */

    private final int ARM_TICKS_PER_INPUT = 5;
    // The min motor encoder position observed from trial and error
    private int MIN_ARM_POS = 10;
    // The max motor encoder position observed from trial and error
    private int MAX_ARM_POS = 3000;

    private final double MAX_CLAW_POS = 0.55;
    private final double MIN_CLAW_POS = 0.19;


    private double lastArmPos = 0.0;

    // This also sets the starting position
    private double clawPos = 0.4;
    private double intakePos = 0.26;
    private double wristPos = 0.0;
    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        GamePiecePipeline pipeline = new GamePiecePipeline(new Scalar(0, 86, 161)); // blue is 4, 47, 86 0056a1
        //red is supposedly 187, 61, 67; d0413f
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(webcam_width, webcam_height, OpenCvCameraRotation.UPSIDE_DOWN );
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 *
                 * Currently do nothing
                 */
            }
        });

        //telemetry.addLine("Waiting for start");
        //telemetry.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftArmMotor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "right_arm_motor");

        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        hugServo = hardwareMap.get(Servo.class,"hug_servo");
        intakeServo = hardwareMap.get(Servo.class,"intakeServo");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        // Arm motors
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        // must set position before switching mode
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the min arm position to whereever it starts
        MIN_ARM_POS = (int)(leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition())/2;

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive() && is_streaming)
        {
            /*
             * Send some stats to the telemetry
             */
            // get the vote from the pipeline
            VOTE vote = pipeline.getVote();
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Vote:", vote.toString());
            telemetry.update();

            /*
             * We want to stop the stream after we've determined which direction to go
             */
            if(frames_collected >= FRAMES_TO_COLLECT && is_streaming)
            {
                is_streaming = false;
            }
        }
        double axial   = 0.0;  // forward and reverse
        double lateral =  0.0; // strafe side to side
        double yaw     =  0.0;  // turn robot

        double[] powers = setMotorPowers(axial, lateral, yaw);
        // get the final vote
        VOTE vote = pipeline.getVote();
        telemetry.addData("Vote:", vote.toString());
        telemetry.update();
        // TODO do second

        // once we get here, we have decided which direction to travel
        // The movement
        runtime.reset();


        if(vote == VOTE.LEFT){
            // -------Step 1:  Drive forward  ------
            powers = setMotorPowers(0.25, 0.0, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 5.4) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------END STEP 1 ----------------------------

            // ---------- Strafe Left ----------------------------
            powers = setMotorPowers(0.0, -0.25, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------------- END STEP 2---------------------
        }
        else if(vote == VOTE.RIGHT){
            // -------Step 1:  Drive forward  ------
            powers = setMotorPowers(0.25, 0.0, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 5.4) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------END STEP 1 ----------------------------

            // ---------- Strafe Right ----------------------------
            powers = setMotorPowers(0.0, 0.25, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------------- END STEP 2---------------------
        }else{
            // default to center, lowest risk
            // -------Step 1:  Drive forward  ------
            powers = setMotorPowers(0.25, 0.0, 0.0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 6.25) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            powers = setMotorPowers(0.0, 0.0, 0.0);
            // -------------END STEP 1 ----------------------------
            //TODO end movement
        }
    }
    public double[] setMotorPowers(double axial, double lateral, double yaw){
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        return new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
    }
    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class GamePiecePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        Scalar color = new Scalar(0, 0, 0);
        // A value that defines how "close" a color has to be
        private final Scalar threshold = new Scalar(threshold_size, threshold_size, threshold_size);
        private Scalar lower_bound;
        private Scalar upper_bound;

        private Mat bckImage = null;

        int votes_left = 0;
        int votes_center = 0;
        int votes_right = 0;

        public GamePiecePipeline(Scalar color) {
            super();
            // convert the input rgb color to hsv
            Mat rgb = new Mat(1, 1, CvType.CV_8UC3, color);
            Mat hsv = new Mat();
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
            Scalar hsv_color = new Scalar(hsv.get(0, 0)[0], hsv.get(0, 0)[1], hsv.get(0, 0)[2]);
            rgb.release();
            hsv.release();
            System.out.println(hsv_color.toString());
            setColor(hsv_color);
            // calculate the lower and upper bounds, ensure the values are between 0 and 255
            lower_bound = new Scalar(Math.max(hsv_color.val[0] - threshold.val[0], 0),
                    150 ,
                    150);
            upper_bound = new Scalar(Math.min(hsv_color.val[0] + threshold.val[0], 179),
                    255,
                    255);
        }

        public void setColor(Scalar color) {
            this.color = color;
        }

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            Mat workingMatrix = input.clone();
            // convert the mat to hsv
            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2HSV);
            // Threshold the HSV image, keep only the blue pixels
            Core.inRange(workingMatrix, lower_bound, upper_bound, workingMatrix);
            // mask out the top half of the image
            Imgproc.rectangle(workingMatrix, new Point(0, 0), new Point(webcam_width, webcam_height/2), new Scalar(0, 0, 0), -1);
            // Find the contours of the objects
            List< MatOfPoint> contours = new ArrayList<>();
            List<MatOfPoint> draw_contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(workingMatrix, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            // Calculate all of the centroids
            List<Point> centroids = new ArrayList<>();
            for(MatOfPoint contour : contours) {
                // only take contours with enough points
                if(contour.rows() < 0) continue;
                Moments moments = Imgproc.moments(contour, false);
                if(moments.get_m00() == 0) continue; // dont divide by zero
                Point centroid = new Point();
                centroid.x += moments.get_m10() / moments.get_m00();
                centroid.y += moments.get_m01() / moments.get_m00();
                centroids.add(centroid);
                draw_contours.add(contour);
            }
            // draw the contour in white
            Imgproc.drawContours(input, draw_contours, -1, new Scalar(255, 255, 255), 2);
            if (!centroids.isEmpty()) {
                int k = 2; // Number of clusters. You might want to adjust this based on your specific case
                Point centroid = new Point();
                if (centroids.size() < k) {
                    // can only do kmeans if we have enough points; just use the centroid of the points
                    for (Point point : centroids) {
                        centroid.x += point.x;
                        centroid.y += point.y;
                    }
                    centroid.x /= centroids.size();
                    centroid.y /= centroids.size();

                }else {
                    // Convert list of points to Mat
                    Mat points = new Mat(centroids.size(), 1, CvType.CV_32FC2);
                    for (int i = 0; i < centroids.size(); i++) {
                        points.put(i, 0, new float[]{(float) centroids.get(i).x, (float) centroids.get(i).y});
                    }

                    // Apply K-means clustering
                    Mat labels = new Mat();
                    Mat centers = new Mat();
                    TermCriteria criteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 100, 0.2);

                    Core.kmeans(points, k, labels, criteria, 3, Core.KMEANS_PP_CENTERS, centers);

                    // Calculate compactness for each cluster
                    double[] compactness = new double[k];
                    for (int i = 0; i < labels.rows(); i++) {
                        int label = (int) labels.get(i, 0)[0];
                        Point pt = centroids.get(i);
                        Point center = new Point(centers.get(label, 0));
                        double distance = Math.pow(pt.x - center.x, 2) + Math.pow(pt.y - center.y, 2);
                        compactness[label] += distance;
                    }

                    // Normalize compactness by cluster size to get average distance
                    int[] clusterSizes = new int[k];
                    for (int i = 0; i < labels.rows(); i++) {
                        int label = (int) labels.get(i, 0)[0];
                        clusterSizes[label]++;
                    }
                    for (int i = 0; i < k; i++) {
                        if (clusterSizes[i] > 0) {
                            compactness[i] /= clusterSizes[i];
                        }
                    }

                    // Identify the cluster with the minimum compactness (tightest grouping)
                    int tightestClusterIndex = 0;
                    for (int i = 1; i < k; i++) {
                        if (compactness[i] < compactness[tightestClusterIndex]) {
                            tightestClusterIndex = i;
                        }
                    }
                    // get the centroid of the largest cluster
                    centroid = new Point(centers.get(tightestClusterIndex, 0)[0], centers.get(tightestClusterIndex, 1)[0]);
                    // draw the other two centroids in green
                    for(int i = 0; i < k; i++) {
                        if(i == tightestClusterIndex) continue;
                        Point other_centroid = new Point(centers.get(i, 0)[0], centers.get(i, 1)[0]);
                        Imgproc.circle(input, other_centroid, 5, new Scalar(0, 255, 0), -1);
                    }
                    labels.release();
                    centers.release();
                    points.release();
                }
                // draw the centroid in blue
                Imgproc.circle(input, centroid, 5, new Scalar(0, 0, 255), -1);
                // determine which region the centroid is in
                telemetry.addData("Centroid:", centroid.toString());
                if(left.contains(centroid)) {
                    votes_left++;
                    telemetry.addData("Current Vote:", "Left");
                } else if(center.contains(centroid)) {
                    votes_center++;
                    telemetry.addData("Current Vote:", "Center");
                } else if(right.contains(centroid)) {
                    votes_right++;
                    telemetry.addData("Current Vote:", "Right");
                }else{
                    telemetry.addData("Current Vote:", "none");
                }
                telemetry.update();
            }
            // increment the number of frames collected
            frames_collected++;

            return input;
        }

        public VOTE getVote() {
            if(votes_left > votes_center && votes_left > votes_right) {
                return VOTE.LEFT;
            } else if(votes_right > votes_center && votes_right > votes_left) {
                return VOTE.RIGHT;
            } else {
                return VOTE.CENTER;
            }
        }
    }

    private enum VOTE{
        LEFT,
        CENTER,
        RIGHT
    }
}
