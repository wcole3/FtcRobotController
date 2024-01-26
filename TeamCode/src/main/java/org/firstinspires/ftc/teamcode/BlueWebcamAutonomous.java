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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam_0"), cameraMonitorViewId);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        GamePiecePipeline pipeline = new GamePiecePipeline(new Scalar(0,0, 255));
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
                webcam.startStreaming(webcam_width, webcam_height, OpenCvCameraRotation.UPRIGHT);
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

        telemetry.addLine("Waiting for start");
        telemetry.update();

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
            // TODO change condition to voting
            if(frames_collected >= FRAMES_TO_COLLECT && is_streaming)
            {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
                is_streaming = false;
            }
        }
        // get the final vote
        VOTE vote = pipeline.getVote();
        // once we get here, we have decided which direction to travel
        // TODO do the movement
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

        int votes_left = 0;
        int votes_center = 0;
        int votes_right = 0;

        public GamePiecePipeline(Scalar color) {
            super();
            setColor(color);
            // calculate the lower and upper bounds, ensure the values are between 0 and 255
            lower_bound = new Scalar(Math.max(color.val[0] - threshold.val[0], 0),
                    Math.max(color.val[1] - threshold.val[1], 0),
                    Math.max(color.val[2] - threshold.val[2], 0));
            upper_bound = new Scalar(Math.min(color.val[0] + threshold.val[0], 255),
                    Math.min(color.val[1] + threshold.val[1], 255),
                    Math.min(color.val[2] + threshold.val[2], 255));
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
            // Find the contours of the objects
            List< MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(workingMatrix, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            // find the average centroid of the contours
            Point centroid = new Point(0, 0);
            for(MatOfPoint contour : contours) {
                Moments moments = Imgproc.moments(contour, false);
                centroid.x += moments.get_m10() / moments.get_m00();
                centroid.y += moments.get_m01() / moments.get_m00();
            }
            centroid.x /= contours.size();
            centroid.y /= contours.size();
            // determine which region the centroid is in
            if(left.contains(centroid)) {
                votes_left++;
            } else if(center.contains(centroid)) {
                votes_center++;
            } else if(right.contains(centroid)) {
                votes_right++;
            }
            // increment the number of frames collected
            frames_collected++;

            return input;
        }

        public VOTE getVote() {
            if(votes_left > votes_center && votes_left > votes_right) {
                return VOTE.LEFT;
            } else if(votes_center > votes_left && votes_center > votes_right) {
                return VOTE.CENTER;
            } else {
                return VOTE.RIGHT;
            }
        }
    }

    private enum VOTE{
        LEFT,
        CENTER,
        RIGHT
    }
}
