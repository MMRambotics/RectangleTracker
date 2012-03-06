package mmrambotics.smartdashboard.extension.camera;

import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.smartdashboard.properties.NumberProperty;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;
import java.awt.Color;
import java.awt.Graphics;
import java.text.DecimalFormat;
import java.util.ArrayList;
import mmrambotics.smartdashboard.extesion.camera.Match;

public class AxisRectangleTracker extends WPICameraExtension {
    public static final String NAME = "Axis Rectangle Tracker";
    public final NumberProperty  threshold          = new NumberProperty(this, "Threshold", 250);
    public final NumberProperty  minArea            = new NumberProperty(this, "Minimum Area", 10000);
    public final NumberProperty  dilations          = new NumberProperty(this, "Dilations", 1);
    public final NumberProperty  erosions           = new NumberProperty(this, "Erosions", 0);
    public final NumberProperty  polygonAccuracy    = new NumberProperty(this, "Polygon Accuracy", 10);
    public final BooleanProperty displayVertices    = new BooleanProperty(this, "Display Vertices", true);
    public final BooleanProperty displayMatchCenter = new BooleanProperty(this, "Display Match Center", true);
    public final BooleanProperty displayImageCenter = new BooleanProperty(this, "Display Image Center", true);
    public final BooleanProperty displayMatchCoords = new BooleanProperty(this, "Display Match Coordinates", true);
    public final BooleanProperty displayContours    = new BooleanProperty(this, "Display Contours", false);
    public final NumberProperty  viewingAngle       = new NumberProperty(this, "Viewing Angle", 43.5);
    public final NumberProperty  heightOfTarget     = new NumberProperty(this, "Height of Target", 89);
    public final NumberProperty  heightOfCamera     = new NumberProperty(this, "Height of Camera", 36);
    public final NumberProperty  targetSpeed        = new NumberProperty(this, "Target Speed Factor", 0.5);
    public final BooleanProperty sendDataToRobot    = new BooleanProperty(this, "Send Data to Robot", false);

    WPIColor contourColour   = new WPIColor(51, 153, 255);
    WPIColor centerColour    = new WPIColor(0, 0, 255);
    WPIColor verticeColour   = new WPIColor(255, 0, 0);
    ArrayList<Match> matches = new ArrayList<Match>();

    long startTime = 0;
    long numFrames = 0;

    @Override
    public WPIImage processImage(WPIColorImage rawImage) {
        if (startTime == 0) startTime = System.currentTimeMillis();

        int t = threshold.getValue().intValue();
        WPIBinaryImage r = rawImage.getRedChannel().getThreshold(t);
        WPIBinaryImage g = rawImage.getGreenChannel().getThreshold(t);
        WPIBinaryImage b = rawImage.getBlueChannel().getThreshold(t);
        WPIBinaryImage binImage = r.getAnd(g).getAnd(b);
        r.dispose();
        g.dispose();
        b.dispose();

        binImage.erode(erosions.getValue().intValue());
        binImage.dilate(dilations.getValue().intValue());

        WPIContour[] contours = binImage.findContours();

        if (displayContours.getValue().booleanValue())
            rawImage.drawContours(contours, verticeColour, 2);

        matches.clear();
        for (WPIContour contour : contours) {
            if (contour.getHeight() * contour.getWidth() < minArea.getValue().intValue())
                continue;

            WPIPolygon temp = contour.approxPolygon(polygonAccuracy.getValue().intValue());
            if (temp.isConvex() && temp.getNumVertices() == 4) {
                Match tempMatch = new Match(temp, rawImage.getWidth(), rawImage.getHeight());
                if (tempMatch.points[0] != null && tempMatch.points[1] != null && tempMatch.points[2] != null && tempMatch.points[3] != null) {
                    matches.add(tempMatch);
                }
            }
        }

        for (int i = 0; i < matches.size(); i++) {
            for (int j = 0; j < matches.size(); j++) {
                if (matches.get(i).isSubMatchOf(matches.get(j))) {
                    matches.remove(i);
                    i--;
                    break;
                }
            }
        }

        for (Match match : matches) {
            rawImage.drawPolygon(match.polygon, contourColour, 1);

            if (displayMatchCenter.getValue().booleanValue())
                rawImage.drawPoint(new WPIPoint(match.cX, match.cY), centerColour, 1);

             if (displayVertices.getValue().booleanValue())
                for (WPIPoint vertice : match.points)
                    rawImage.drawPoint(vertice, verticeColour, 2);
        }

        if (displayImageCenter.getValue().booleanValue()) {
            int cX = rawImage.getWidth() / 2;
            int cY = rawImage.getHeight() / 2;
            rawImage.drawLine(new WPIPoint(cX, cY - 3), new WPIPoint(cX, cY + 3), centerColour, 1);
            rawImage.drawLine(new WPIPoint(cX - 3, cY), new WPIPoint(cX + 3, cY), centerColour, 1);
        }

        if (sendDataToRobot.getValue().booleanValue()) {
            Robot.getTable().beginTransaction();
            Robot.getTable().putBoolean("targetsFound", matches.size() > 0);
            
            if (matches.size() > 0) {
                Robot.getTable().putDouble("targetX", matches.get(0).dX);
                double distance = matches.get(0).distanceInInches(viewingAngle.getValue().doubleValue(), heightOfTarget.getValue().doubleValue(), heightOfCamera.getValue().doubleValue());
                Robot.getTable().putDouble("targetDistance", distance);
            } else {
                Robot.getTable().putDouble("targetX", 0);
                Robot.getTable().putDouble("targetDistance", 0);
            }

            Robot.getTable().putDouble("targetSpeed", targetSpeed.getValue().doubleValue());
            Robot.getTable().endTransaction();
        }
        
        numFrames++;
        binImage.dispose();
        return rawImage;
    }

    
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        String affix = (matches.size() == 1) ? "" : "es";
        String temp  = matches.size() + " match" + affix + " found.";
        g.drawString(temp, 5, 40);

        long timeDifference = System.currentTimeMillis() - startTime;
        float secondsElapsed = timeDifference / 1000;
        float fps = Math.round(numFrames / secondsElapsed);
        g.drawString(Float.toString(fps) + " FPS", 5, 20);

        DecimalFormat f = new DecimalFormat("#.##");
        g.setColor(Color.BLUE);

        for (Match match : matches) {
            String values = f.format(match.dX) + ", " + f.format(match.dY);
            g.drawString(values, match.cX + 4, match.cY + 12);

            String distance = f.format(match.distanceInInches(viewingAngle.getValue().doubleValue(), heightOfTarget.getValue().doubleValue(), heightOfCamera.getValue().doubleValue())) + "\"";
            g.drawString(distance, match.cX + 4, match.cY + 32);

            String angle = f.format(match.angleFromTarget(viewingAngle.getValue().doubleValue()));
            g.drawString(angle, match.cX + 4, match.cY + 52);

            if (!displayMatchCoords.getValue().booleanValue()) continue;
            int quadrant = 1;
            for (WPIPoint point : match.points) {
                String coordinates = Integer.toString(point.getX()) + ", " + Integer.toString(point.getY());
                int dX = (quadrant == 2 || quadrant == 3) ? 5 : -60;
                int dY = (quadrant == 1 || quadrant == 2) ? -5 : 5;
                g.drawString(coordinates, point.getX() + dX, point.getY() + dY);
                quadrant++;
            }
        }
    }
}
