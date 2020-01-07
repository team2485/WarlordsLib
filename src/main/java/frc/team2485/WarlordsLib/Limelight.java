package frc.team2485.WarlordsLib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Limelight {

    /**
     * Whether Limelight has any valid targets
     * @return limelight has valid target
     */
    public boolean hasValidTarget() {
        return getProperty("tv") == 1.0;
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     * @return degrees offset
     */
    public double getHorizontalOffset() {
        return getProperty("tx");
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     * @return degrees offset
     */
    public double getVerticalOffset() {
        return getProperty("ty");
    }

    /**
     * Target Area (0% of image to 100% of image)
     * @return percent of image
     */
    public double getTargetArea() {
        return getProperty("ta");
    }

    /**
     * Skew or rotation (-90 degrees to 0 degrees)
     * @return degrees
     */
    public double getSkew() {
        return getProperty("ts");
    }

    /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     * @return milliseconds
     */
    public double getLatencyContribution() {
        return getProperty("tl");
    }

    /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     * @return pixels
     */
    public double getBoundingBoxShortLength() {
        return getProperty("tshort");
    }

    /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     * @return pixels
     */
    public double getBoundingBoxLongLength() {
        return getProperty("tlong");
    }

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * @return 0-320 pixels
     */
    public double getBoundingBoxHorizontalLength() {
        return getProperty("thor");
    }

    /**
     * 	Vertical sidelength of the rough bounding box (0 - 320 pixels)
     * @return 0-320 pixels
     */
    public double getBoundingBoxVerticalLength() {
        return getProperty("tvert");
    }

    /**
     * True active pipeline index of the camera (0 .. 9)
     * @return index 0 to 9
     */
    public double getActivePipelineIndex() {
        return getProperty("getpipe");
    }

    /**
     * Results of a 3D position solution, 6 numbers: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * @return position array
     */
    public double[] getCameraTranslation() {
        return getTable().getEntry("camtran").getDoubleArray(new double[6]);
    }

    private enum LedMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private int id;

        LedMode(int id) {
            this.id = id;
        }
    }

    /**
     * Sets limelight’s LED state
     * @param mode Default, Off, Blink or On
     */
    public void setLedMode(LedMode mode) {
        setProperty("ledMode", mode.id);
    }

    /**
     * Sets limelight’s operation mode
     * @param enable If true use LL as a vision processor; if false use LL as a driver camera (Increases exposure, disables vision processing)
     */
    public void enableVisionProcessing(boolean enable) {
        setProperty("camMode", enable ? 0 : 1);
    }

    /**
     * Sets limelight’s current pipeline
     * @param pipeline value between 0 and 9
     */
    public void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            setProperty("pipeline", pipeline);
        }
    }

    private enum StreamingMode{
        STANDARD(0), PIP_MAIN(1), PIP_SECONDARY(2);

        private int id;

        StreamingMode(int id) {
            this.id = id;
        }
    }

    /**
     * Sets limelight’s streaming mode
     *
     * Standard - Side-by-side streams if a webcam is attached to Limelight
     * PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     * PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     * @param mode streaming mode
     */
    public void setStreamingMode(StreamingMode mode) {
        setProperty("stream", mode.id);
    }

    /**
     * Allows users to take snapshots during a match. If enabled takes two snapshots per second. Default false.
     * @param enable enable snapshots
     */
    public void enableSnapshots(boolean enable) {
        setProperty("snapshot", enable ? 1 : 0);
    }

    /**
     * Enable “send contours” in the “Output” tab to stream corner coordinates:
     * @return number array
     */
    public Number[] getCornerYCoordinates() {
        return getTable().getEntry("tcorny").getNumberArray(new Number[0]);
    }

    /**
     * Enable “send contours” in the “Output” tab to stream corner coordinates:
     * @return number array
     */
    public Number[] getCornerXCoordinates() {
        return getTable().getEntry("tcornx").getNumberArray(new Number[0]);
    }


    /**
     * Get the limelight's network table.
     * @return NetworkTable of limelight;
     */
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getProperty(String key) {
        return getTable().getEntry(key).getDouble(0);
    }

    public void setProperty(String key, int n) {
        getTable().getEntry(key).setNumber(n);
    }
}
