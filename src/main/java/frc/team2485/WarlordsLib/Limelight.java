package frc.team2485.WarlordsLib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class Limelight {

    private NetworkTableInstance _networkTableInstance;
    private NetworkTable _limelightTable;

    private enum LimelightVersion {
        ONE(54, 41), TWO(59.6, 49.7);

        private double fovx;
        private double fovy;

        public double getFovx() {
            return fovx;
        }

        public double getFovy() {
            return fovy;
        }

        private LimelightVersion(double fovx, double fovy) {
            this.fovx = fovx;
            this.fovy = fovy;
        }
    }

    private LimelightVersion _limelightVersion;

    /**
     * Wrapper for Limelight.
     * @param limelightVersion version of this limelight.
     */
    public Limelight(LimelightVersion limelightVersion) {
        _limelightVersion = limelightVersion;
        _networkTableInstance = NetworkTableInstance.getDefault();
        _limelightTable = _networkTableInstance.getTable("limelight");
    }

    /**
     * Wrapper for Limelight.
     */
    public Limelight() {
        this(LimelightVersion.TWO);
    }

    public LimelightVersion getLimelightVersion() {
        return this._limelightVersion;
    }
    /**
     * Whether Limelight has any valid targets
     * @return limelight has valid target
     */
    public boolean hasValidTarget() {
        return getDoubleProperty("tv", 0.0) == 1.0;
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     * @return degrees offset
     */
    public double getTargetHorizontalOffset(double defaultValue) {
        return getDoubleProperty("tx", defaultValue);
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     * @return degrees offset
     */
    public double getTargetVerticalOffset(double defaultValue) {
        return getDoubleProperty("ty", defaultValue);
    }

    /**
     * Target Area (0% of image to 100% of image)
     * @return percent of image
     */
    public double getTargetArea(double defaultValue) {
        return getDoubleProperty("ta", defaultValue);
    }

    /**
     * Skew or rotation (-90 degrees to 0 degrees)
     * @return degrees
     */
    public double getTargetSkew(double defaultValue) {
        return getDoubleProperty("ts", defaultValue);
    }

    /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     * @return milliseconds
     */
    public double getLatencyContribution(double defaultValue) {
        return getDoubleProperty("tl", defaultValue);
    }

    /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     * @return pixels
     */
    public double getBoundingBoxShortLength(double defaultValue) {
        return getDoubleProperty("tshort", defaultValue);
    }

    /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     * @return pixels
     */
    public double getBoundingBoxLongLength(double defaultValue) {
        return getDoubleProperty("tlong", defaultValue);
    }

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * @return 0-320 pixels
     */
    public double getBoundingBoxHorizontalLength(double defaultValue) {
        return getDoubleProperty("thor", defaultValue);
    }

    /**
     * 	Vertical sidelength of the rough bounding box (0 - 320 pixels)
     * @return 0-320 pixels
     */
    public double getBoundingBoxVerticalLength(double defaultValue) {
        return getDoubleProperty("tvert", defaultValue);
    }

    /**
     * True active pipeline index of the camera (0 .. 9)
     * @return index 0 to 9
     */
    public double getActivePipelineIndex(double defaultValue) {
        return getDoubleProperty("getpipe", defaultValue);
    }

    /**
     * Results of a 3D position solution, 6 numbers: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * @return position array
     */
    public double[] getCameraTranslation() {
        return _limelightTable.getEntry("camtran").getDoubleArray(new double[6]);
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
        setNumberProperty("ledMode", mode.id);
    }

    /**
     * Sets limelight’s operation mode
     * @param enable If true use LL as a vision processor; if false use LL as a driver camera (Increases exposure, disables vision processing)
     */
    public void enableVisionProcessing(boolean enable) {
        setNumberProperty("camMode", enable ? 0 : 1);
    }

    /**
     * Sets limelight’s current pipeline
     * @param pipeline value between 0 and 9
     */
    public void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            setNumberProperty("pipeline", pipeline);
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
        setNumberProperty("stream", mode.id);
    }

    /**
     * Allows users to take snapshots during a match. If enabled takes two snapshots per second. Default false.
     * @param enable enable snapshots
     */
    public void enableSnapshots(boolean enable) {
        setNumberProperty("snapshot", enable ? 1 : 0);
    }

    /**
     * Enable "send contours" in the "Output" tab to stream corner coordinates:
     * @return number array
     */
    public Number[] getCornerYCoordinates() {
        return _limelightTable.getEntry("tcorny").getNumberArray(new Number[0]);
    }

    /**
     * Enable "send contours" in the "Output" tab to stream corner coordinates:
     * @return number array
     */
    public Number[] getCornerXCoordinates() {
        return _limelightTable.getEntry("tcornx").getNumberArray(new Number[0]);
    }

    /**
     * Raw screenspace X of ungrouped target
     * @param target contour 0 to 2
     * @return -1 to 1 normalized screenspace
     */
    public double getUngroupedTargetScreenspaceX(int target, double defaultValue) {
        return getDoubleProperty("tx" + target, defaultValue);
    }

    /**
     * Raw screenspace Y of ungrouped target
     * @param target contour 0 to 2
     * @return -1 to 1 normalized screenspace
     */
    public double getUngroupedTargetScreenspaceY(int target, double defaultValue) {
        return getDoubleProperty("ty" + target, defaultValue);
    }


    /**
     * Target area of ungrouped target
     * @param target contour 0 to 2
     * @return percent area of image
     */
    public double getUngroupedTargetArea(int target, double defaultValue) {
        return getDoubleProperty("ta" + target, defaultValue);
    }

    /**
     * Get crosshair x
     * @param crosshair crosshair 0 or 1
     * @return normalized screenspace
     */
    public double getCrosshairX(int crosshair, double defaultValue) {
        return _limelightTable.getEntry("cx" + crosshair).getDouble(defaultValue);
    }

    /**
     * Get crosshair y
     * @param crosshair crosshair 0 or 1
     * @return normalized screenspace
     */
    public double getCrosshairY(int crosshair, double defaultValue) {
        return _limelightTable.getEntry("cy" + crosshair).getDouble(defaultValue);
    }

    /**
     * Skew or rotation of ungrouped target
     * @param target contour 0 to 2
     * @return -90 to 90 degrees
     */
    public double getUngroupedTargetrSkew(int target, double defaultValue) {
        return getDoubleProperty("tx" + target, defaultValue);
    }

    public double getDoubleProperty(String key, double defaultValue) {
        return _limelightTable.getEntry(key).getDouble(defaultValue);
    }

    public void setNumberProperty(String key, Number n) {
        _limelightTable.getEntry(key).setNumber(n);
    }
}