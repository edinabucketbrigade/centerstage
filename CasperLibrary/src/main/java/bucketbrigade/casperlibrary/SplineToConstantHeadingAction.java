package bucketbrigade.casperlibrary;

public class SplineToConstantHeadingAction extends Action {
    public double x;
    public double y;
    public double heading;

    public SplineToConstantHeadingAction(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void mirror() {
        this.y = mirrorY(y);
        this.heading = mirrorAngle(heading);
    }
}