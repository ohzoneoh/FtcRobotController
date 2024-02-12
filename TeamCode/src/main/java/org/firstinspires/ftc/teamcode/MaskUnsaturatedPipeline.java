package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.inRange;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MaskUnsaturatedPipeline extends OpenCvPipeline {

    Mat temp = new Mat();
    Mat mask = new Mat();
    Scalar down = new Scalar(0, 100, 0);
    Scalar up = new Scalar( 255, 200, 255 );

    @Override
    public Mat processFrame ( Mat input )
    {
        //Imgproc.cvtColor(input, img, Imgproc.COLOR_BGR2HSV);
        //inRange(img, low, high, mask);
        //img2.setTo(thing, mask);
        //return img2;
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_BGR2HSV);

        inRange(temp, down, up, mask);

        return mask;
    }
}
