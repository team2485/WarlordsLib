package frc.team2485.WarlordsLib;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

/**
 * Linear interpolator
 */
public class Interpolator {
    
    private ArrayList<Point> points;

    public Interpolator(Point... points) {
        this.points = new ArrayList<Point>(Arrays.asList(points));
        Collections.sort(this.points);
    }

    public double getOutput(double input) {
        int bottomI = -1;
        //find bottom point to interpolate from 
        if(input < points.get(0).X) { //here, we extrapolate from the lowest two points
            bottomI = 0;
        } else if (input >= points.get(points.size() - 1).X) { //here, we extrapolate from the top two points
            bottomI = points.size() - 2;
        } else {
            for(int i=0; i<points.size() - 1; i++){ //here, we interpolate from the points bordering the given input
                if(input >= points.get(i).X && input < points.get(i+1).X) {
                    bottomI = i;
                }
            }
        }
        
        return points.get(bottomI).Y + (input - points.get(bottomI).X)*(points.get(bottomI+1).Y - points.get(bottomI).Y)/(points.get(bottomI+1).X - points.get(bottomI).X);
        
    }
}
