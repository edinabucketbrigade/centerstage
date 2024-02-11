package bucketbrigade.casperlibrary;

public class SplineToLinearHeadingAction extends Action {
    public double x;
    public double y;
    public double heading;
    public double tangent;

    public SplineToLinearHeadingAction(double x, double y, double heading, double tangent) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tangent = tangent;
    }

    public void mirror() {
        this.y = mirrorY(y);
        this.heading = mirrorAngle(heading);
        this.tangent = mirrorAngle(tangent);
    }
}