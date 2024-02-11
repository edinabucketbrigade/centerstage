package bucketbrigade.casperlibrary;

public class LineToLinearHeadingAction extends Action {

    public double x;
    public double y;
    public double heading;

    public LineToLinearHeadingAction(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void mirror() {
        y = mirrorY(y);
        heading = mirrorAngle(heading);
    }

}