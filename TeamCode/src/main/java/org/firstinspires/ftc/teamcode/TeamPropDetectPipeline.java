package org.firstinspires.ftc.teamcode;

import static org.opencv.core.Core.inRange;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetectPipeline extends OpenCvPipeline {

    Mat rightPos = new Mat();
    Mat centerPos = new Mat();

    Scalar rightAvg = null;
    Scalar centerAvg = null;

    double centerDistance = 100;
    double rightDistance = 100;

    Scalar green = new Scalar(0, 255, 0);
    Scalar white = new Scalar(225, 225, 225);

    Scalar cc = white;
    Scalar rc = white;

    int position = 0;

    @Override
    public Mat processFrame ( Mat input )
    {
        //Imgproc.cvtColor(input, img, Imgproc.COLOR_BGR2HSV);
        //inRange(img, low, high, mask);
        //img2.setTo(thing, mask);
        //return img2

        centerPos = input.submat(new Rect(new Point(85, 35), new Point(160, 105)));
        rightPos = input.submat(new Rect(new Point(215, 40), new Point(300, 120)));

        centerAvg = Core.mean(centerPos);
        rightAvg = Core.mean(rightPos);

        centerDistance = dist(centerAvg, true);
        rightDistance = dist(rightAvg, true);

        cc = white;
        rc = white;
        position = -1;

        if ( centerDistance < 210)
        {
            cc = green;
            position = 0;
        }
        else if ( rightDistance < 200)
        {
            rc = green;
            position = 1;
        }

        Imgproc.rectangle(
                input,
                new Point(85, 35),
                new Point(160, 105),
                cc, 8);

        Imgproc.rectangle(
                input,
                new Point(215, 40),
                new Point(300, 120),
                rc, 8);

        return input;
    }



    private double dist(Scalar color, boolean target)
    {
        double r = color.val[0];
        double g = color.val[1];
        double b = color.val[2];
        int tr;
        int tg;
        int tb;

        if (target) //TRUE is blue
        {
            tr = 0;
            tg = 0;
            tb = 255;
        }
        else
        {
            tr = 255;
            tg = 0;
            tb = 0;
        }

        return Math.sqrt(Math.pow((r - tr),2) + Math.pow((g - tg), 2) + Math.pow((b - tb), 2));
    }


    public double getCenterDistance()
    {
        return centerDistance;
    }

    public double getRightDistance()
    {
        return rightDistance;
    }

    public int getPos()
    {
        return position;
    }

    public String getStringPos()
    {
        /*
            ---X---
           |        |
           X        X
           |        |
         */


        if (getPos() == -1) {
            return " ---O---\n" +
                    "|        |\n" +
                    "X        O\n" +
                    "|        |";
        }
        else if (getPos() == 0)
        {
            return " ---X---\n" +
                    "|        |\n" +
                    "O        O\n" +
                    "|        |";
        }
        else if (getPos() == 1)
        {
            return " ---O---\n" +
                    "|        |\n" +
                    "O        X\n" +
                    "|        |";
        }
        return "ERROR OH NO";
    }
}




