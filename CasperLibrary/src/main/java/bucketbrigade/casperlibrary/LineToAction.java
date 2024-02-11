package bucketbrigade.casperlibrary;

public class LineToAction extends Action {
    public double x;
    public double y;

    public LineToAction(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void mirror() {
        y = mirrorY(y);
    }
}