package bucketbrigade.casperlibrary;

public class SetTangentAction extends Action {
    public double tangent;

    public SetTangentAction(double tangent) {
        this.tangent = tangent;
    }

    public void mirror() {
        this.tangent = mirrorAngle(tangent);
    }
}
