package frc.team2485.WarlordsLib;

public class Point implements Comparable<Point> {
  public final double X;
  public final double Y;

  public Point(double x, double y) {
    X = x;
    Y = y;
  }

  public int compareTo(Point p) {
    return Double.compare(this.X, p.X);
  }
}
