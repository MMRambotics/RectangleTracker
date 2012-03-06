package mmrambotics.smartdashboard.extesion.camera;

import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;

public class Match {
    public WPIPolygon polygon;
    public WPIPoint[] points;
    public int cX, cY;
    public float dX, dY;
    private int imageWidthPixels, imageHeightPixels;

    private static final double targetWidth  = 24;
    private static final double targetHeight = 18;

    public Match(WPIPolygon p, int imageWidth, int imageHeight) {
        polygon = p;
        imageWidthPixels = imageWidth;
        imageHeightPixels = imageHeight;
        cX = polygon.getX() + polygon.getWidth() / 2;
        cY = polygon.getY() + polygon.getHeight() / 2;
        dX = pixelToRealWorld(cX, imageWidth);
        dY = pixelToRealWorld(cY, imageHeight);
        assignPoints();
    }

    public boolean isSubMatchOf(Match m) {
        return (m.points[0].getX() < points[0].getX() &&
                m.points[0].getY() < points[0].getY() &&
                m.points[2].getX() > points[2].getX() &&
                m.points[2].getY() > points[2].getY());
    }

    /**
     * Gets the distance between the center of a target and the camera lens.
     *
     * @param viewingAngle Viewing angle in degrees of the camera.  43.5 for the
     * Axis M1011 and 47 for the Axis 206.
     * @param heightOfTarget The distance between the center of the target and
     * ground.
     * @param heightOfCamera The height of the camera from the ground in inches.
     * @return The distance in inches.
     */
    public double distanceInInches(double viewingAngle, double heightOfTarget, double heightOfCamera) {
        double side = Math.abs(points[0].getY() - points[3].getY());
        double length = Math.abs(points[0].getX() - points[3].getX());
        double polygonHeight = Math.sqrt(Math.pow(side, 2) + Math.pow(length, 2));

        //System.out.println("Side: " + side + " Length: " + length + " Polygon Height: " + polygonHeight);
        double imageHeight = (targetHeight * imageHeightPixels) / polygonHeight;
        double fovDistance = fovDistance(imageHeight, viewingAngle);

        double opposite = Math.abs(heightOfTarget - heightOfCamera);
        double result   = Math.sqrt(Math.pow(opposite, 2) + Math.pow(fovDistance, 2));
        //System.out.println("Image Height: " + imageHeight + " FOV Distance: " + fovDistance + " Opposite: " + opposite + " Result: " + result);
        return result;
    }

    /**
     * Gets the angle the camera is facing at relative to the target.  0 is
     * straight at the target.
     *
     * @param viewingAngle Viewing angle in degrees of the camera.  43.5 for the
     * Axis M1011 and 47 for the Axis 206.
     * @return Angle in the degrees.  -90 <= 0 <= 90
     */
    public double angleFromTarget(double viewingAngle) {
        /*
        double leftSide  = points[3].getY() - points[0].getY();
        double rightSide = points[2].getY() - points[1].getY();
        leftSide  = imageHeightPixels / leftSide * targetHeight;
        rightSide = imageHeightPixels / rightSide * targetHeight;

        double fovDistanceLeft  = fovDistance(leftSide, viewingAngle);
        double fovDistanceRight = fovDistance(rightSide, viewingAngle);
        double targetTheta = calculateCosineTheta(fovDistanceLeft, fovDistanceRight, targetWidth);
        double sideTheta   = calculateCosineTheta(targetWidth, fovDistanceLeft, fovDistanceRight);

        // Angle in a triangle will add up to 180, subtract from 90 instead of
        // 180 to offset.
        return (90 - (targetTheta / 2) - sideTheta);*/

        double height = Math.sqrt(Math.pow(points[0].getY(), 2) + Math.pow(points[2].getY(), 2));
        double width  = Math.sqrt(Math.pow(points[0].getX(), 2) + Math.pow(points[2].getX(), 2));

        double dimensionRatio = width /height;
        //System.out.println("Width: " + width + " Height: " + height + " Ratio: " + dimensionRatio);
        return dimensionRatio;
    }

    private double fovDistance(double sideLength, double viewingAngle) {
        return (sideLength / 2) / Math.tan(Math.toRadians(viewingAngle) / 2);
    }

    private double calculateCosineTheta(double lengthA, double lengthB, double lengthC) {
        // cos C = (a^2 + b^2 - c^2) / (2ab)
        return Math.toDegrees(Math.acos((Math.pow(lengthA, 2) + Math.pow(lengthB, 2) - Math.pow(lengthC, 2)) / (2 * lengthA * lengthB)));
    }

    private float pixelToRealWorld(int coord, int resolution) {
        float center = resolution / 2;
        return (coord - center) / center;
    }

    private void assignPoints() {
        points = new WPIPoint[4];
        for (WPIPoint point : polygon.getPoints()) {
            points[getQuadrantIndex(point)] = point;
        }
    }

    private int getQuadrantIndex(WPIPoint point) {
        if (point.getX() < cX && point.getY() < cY) {
            return 0; // Top Left
        } else if (point.getX() >= cX && point.getY() < cY) {
            return 1; // Top Right
        } else if (point.getX() >= cX && point.getY() >= cY) {
            return 2; // Bottom Right
        } else {
            return 3; // Bottom Left
        }
    }
}
