package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class AimingPipeline extends OpenCvPipeline
{
    private static final Scalar HSV_LOWER = new Scalar(100, 50, 50);
    private static final Scalar HSV_UPPER = new Scalar(130, 255, 255);

    private static final Mat ERODE_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 4));
    private static final Mat DIALATE_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 4));

    private static final int MIN_Y = 0;
    private static final int MAX_Y = 100;

    // width / height ratio
    private static final double MIN_POWERSHOT_BOX_RATIO = 1.2;
    private static final double MAX_POWERSHOT_BOX_RATIO = 8.0;

    private static final double MIN_GOAL_BOX_RATIO = 1.0 / 8.0;
    private static final double MAX_GOAL_BOX_RATIO = 1.0;

    private static final double MIN_POWERSHOT_WIDTH = 2.0;
    private static final double MAX_POWERSHOT_WIDTH = 60.0;

    private static final double MIN_GOAL_WIDTH = 30.0;
    private static final double MAX_GOAL_WIDTH = 160.0;

    private static final double X_SPACING_TOLERANCE = 0.3;

    // max difference in widths between powershots abs(h1 - h2) / (h1 + h2)
    private static final double MAX_WIDTH_RATIO = 0.25;

    // y spacing difference / x spacing
    private static final double MAX_SPACING_RATIO = 0.25;
    private static final double MAX_COMBINED_SPACING_RATIO = 0.4;

    private Mat filteredMat = new Mat();
    private Mat rangeMat = new Mat();
    private Mat denoisedMat = new Mat();
    private Mat contourMat = new Mat();
    private Mat finalMat = new Mat();
    private Mat hierarchy = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();
    private List<RotatedRect> rects = new ArrayList<>();

    private int stageNum = 0;

    private double goalCenterX;
    private double powershot1CenterX;
    private double powershot2CenterX;
    private double powershot3CenterX;

    private Size adjustedRotatedRectSize(RotatedRect rotatedRect) {
        Size size = rotatedRect.size;
        if (rotatedRect.angle < -45) {
            return new Size(size.height, size.width);
        } else {
            return size;
        }
    }

    private Point[] getVertices(RotatedRect rotatedRect) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        return vertices;
    }

    private void drawRotatedRect(Mat out, RotatedRect rotatedRect, Scalar color, int thickness) {
        Point[] vertices = getVertices(rotatedRect);
        List<MatOfPoint> boxContours = new ArrayList<>();
        boxContours.add(new MatOfPoint(vertices));
        Imgproc.drawContours(out, boxContours, -1, color, thickness);
    }

    @Override
    public void onViewportTapped() {
        stageNum++;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contours.clear();
        rects.clear();

        input.copyTo(finalMat);
        input.copyTo(contourMat);

        Imgproc.line(finalMat, new Point(0, MIN_Y), new Point(finalMat.width(), MIN_Y), new Scalar(255.0, 165.0, 0.0), 2);
        Imgproc.line(finalMat, new Point(0, MAX_Y), new Point(finalMat.width(), MAX_Y), new Scalar(255.0, 165.0, 0.0), 2);

        // initial noise reduction, bilateral is good for noise reduction while preserving boundaries
//        Imgproc.bilateralFilter(input, filteredMat, 5, 100, 100);
        input.copyTo(filteredMat);
        // rgb -> hsv
        Imgproc.cvtColor(filteredMat, rangeMat, Imgproc.COLOR_RGB2HSV);
        // hsv filtering
        Core.inRange(rangeMat, HSV_LOWER, HSV_UPPER, rangeMat);

        // erode + dilate remove small areas and fill gaps
        Imgproc.erode(rangeMat, denoisedMat, ERODE_ELEMENT, new Point(-1, -1), 2);
        Imgproc.dilate(denoisedMat, denoisedMat, DIALATE_ELEMENT, new Point(-1, -1), 2);

        // finds contours in the filtered mat
        Imgproc.findContours(denoisedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.drawContours(contourMat, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

        for (MatOfPoint c : contours) {
            // fit rotated rect to contour
            MatOfPoint2f points = new MatOfPoint2f(c.toArray());
            RotatedRect rect = Imgproc.minAreaRect(points);
            points.release();

            Size rectSize = adjustedRotatedRectSize(rect);
            double ratio = rectSize.height / rectSize.width;

            // check if the ratio of height to width are within the defined range and that the width is within range
            if (ratio > MIN_POWERSHOT_BOX_RATIO
                    && ratio < MAX_POWERSHOT_BOX_RATIO
                    && rectSize.width > MIN_POWERSHOT_WIDTH
                    && rectSize.width < MAX_POWERSHOT_WIDTH
                    && rect.center.y >= MIN_Y
                    && rect.center.y <= MAX_Y) {
                drawRotatedRect(finalMat, rect, new Scalar(0.0, 255.0, 0.0), 3);
                rects.add(rect);
            }

            if (ratio > MIN_GOAL_BOX_RATIO
                    && ratio < MAX_GOAL_BOX_RATIO
                    && rectSize.width > MIN_GOAL_WIDTH
                    && rectSize.width < MAX_GOAL_WIDTH
                    && rect.center.y >= MIN_Y
                    && rect.center.y <= MAX_Y) {
                drawRotatedRect(finalMat, rect, new Scalar(255.0, 0.0, 255.0), 3);
                goalCenterX = rect.center.x;
            }

            c.release();
        }

        // sort contours left to right
        rects.sort(Comparator.comparingDouble(a -> a.center.x));

        // too few matches, bail
        if (rects.size() < 3) {
            Mat[] mats = new Mat[] { finalMat, input, filteredMat, rangeMat, denoisedMat, contourMat };
            return mats[stageNum % mats.length];
        }

        // initialize search params
        double minDetectedSpacing = Double.MAX_VALUE;
        RotatedRect match1 = rects.get(0);
        RotatedRect match2 = rects.get(1);
        RotatedRect match3 = rects.get(2);

        // process to check all combinations of three rects that meet certain criteria
        for (int i = 0; i < rects.size(); i++) {
            for (int j = i + 1; j < rects.size(); j++) {
                for (int k = j + 1; k < rects.size(); k++) {
                    RotatedRect rectA = rects.get(i);
                    RotatedRect rectB = rects.get(j);
                    RotatedRect rectC = rects.get(k);

                    // point zero is always the bottom
                    Point bottomA = getVertices(rectA)[0];
                    Point bottomB = getVertices(rectB)[0];
                    Point bottomC = getVertices(rectC)[0];

                    Size sizeA = adjustedRotatedRectSize(rectA);
                    Size sizeB = adjustedRotatedRectSize(rectB);
                    Size sizeC = adjustedRotatedRectSize(rectC);

                    double spacingXAB = rectB.center.x - rectA.center.x;
                    double spacingXBC = rectC.center.x - rectB.center.x;

                    double spacingYAB = bottomB.y - bottomA.y;
                    double spacingYBC = bottomC.y - bottomB.y;

                    double spacingXCombined = spacingXAB + spacingXBC;
                    double spacingYCombined = Math.abs(spacingYAB) + Math.abs(spacingYBC);
                    double spacingYDifference = Math.abs(spacingYAB - spacingYBC);

                    // spacing in the x direction should be similar between each pair
                    // difference in y spacing between each pair should be low relative to the x spacing
                    // the widths of each rect should be similar
                    // the ratio between y and x spacing should be less than a max value
                    // we are looking to find the combination that fits these criteria and minimizes overall spacing
                    if (Math.abs(spacingXAB - spacingXBC) / spacingXCombined < X_SPACING_TOLERANCE
                            && Math.abs(sizeA.width - sizeB.width) / (sizeA.width + sizeB.width) < MAX_WIDTH_RATIO
                            && Math.abs(sizeB.width - sizeC.width) / (sizeB.width + sizeC.width) < MAX_WIDTH_RATIO
                            && Math.abs(sizeA.width - sizeC.width) / (sizeA.width + sizeC.width) < MAX_WIDTH_RATIO
                            && spacingYDifference / spacingXCombined < MAX_SPACING_RATIO
                            && spacingYCombined / spacingXCombined < MAX_COMBINED_SPACING_RATIO
                            && spacingXCombined + spacingYCombined < minDetectedSpacing) {
                        match1 = rectA;
                        match2 = rectB;
                        match3 = rectC;
                        minDetectedSpacing = spacingXCombined + spacingYCombined;
                    }
                }
            }
        }

        drawRotatedRect(finalMat, match1, new Scalar(255.0, 0.0, 0.0), 3);
        drawRotatedRect(finalMat, match2, new Scalar(255.0, 0.0, 0.0), 3);
        drawRotatedRect(finalMat, match3, new Scalar(255.0, 0.0, 0.0), 3);
        if (minDetectedSpacing !=Double.MAX_VALUE){
            powershot1CenterX = match1.center.x;
            powershot2CenterX = match2.center.x;
            powershot3CenterX = match3.center.x;
        }

        Mat[] mats = new Mat[] { finalMat, input, filteredMat, rangeMat, denoisedMat, contourMat };
        return mats[stageNum % mats.length];
    }

    public double getGoalCenterX() {
        return goalCenterX;
    }

    public double[] getPowershotsCenterX() {
        return new double[] { powershot1CenterX, powershot2CenterX, powershot3CenterX };
    }
}
