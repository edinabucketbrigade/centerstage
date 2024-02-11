package bucketbrigade.casperlibrary;

public class TurnAction extends Action {
    public double angle;

    public TurnAction(double angle) {
        this.angle = angle;
    }

    public void mirror() {
        this.angle = -angle;
    }
}