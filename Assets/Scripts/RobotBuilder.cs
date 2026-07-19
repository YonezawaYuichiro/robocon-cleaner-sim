using UnityEngine;

public class RobotBuilder : MonoBehaviour
{
    [Header("寸法 (m)")]
    public float bodyDiameter = 0.30f;     // 掃除機本体の直径
    public float bodyHeight = 0.12f;       // 本体の厚み
    public float cameraTopHeight = 1.00f;  // カメラ頭頂部の高さ
    public float wheelTrack = 0.30f;       // 左右車輪の間隔

    [Header("質量・物理")]
    public float totalMass = 10f;          // 全質量(kg)
    public float centerOfMassHeight = 0.50f; // 重心高さ(m)

    [Header("スタート位置 (mm, コース座標)")]
    public float startXmm = 2100f;         // 右下のマス中心
    public float startZmm = 300f;

    [Header("見た目のみ（走行には無影響）")]
    public float modelYawOffsetDeg = 90f;  // 見た目モデルのヨー回転。左=+90/右=-90

    [ContextMenu("機体を組み立てる")]
    public void BuildRobot()
    {
        Transform old = transform.Find("RobotBody");
        if (old != null) DestroyImmediate(old.gameObject);
        Transform oldCam = transform.Find("CameraTop");
        if (oldCam != null) DestroyImmediate(oldCam.gameObject);

        // ルートをスタート地点へ
        transform.position = new Vector3(startXmm / 1000f, 0.05f, startZmm / 1000f);
        transform.rotation = Quaternion.Euler(0f, 270f, 0f); // -X(左)向きでスタート

        Transform root = new GameObject("RobotBody").transform;
        root.SetParent(transform);
        root.localPosition = Vector3.zero;
        root.localRotation = Quaternion.Euler(0f, modelYawOffsetDeg, 0f); // 見た目だけ回す（物理は親のRobotが担当）

        // 本体（円盤）＝見た目はCylinder、当たり判定は安定したCapsuleを別付け
        GameObject body = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        body.name = "Body";
        body.transform.SetParent(root);
        body.transform.localScale = new Vector3(bodyDiameter, bodyHeight / 2f, bodyDiameter);
        body.transform.localPosition = new Vector3(0, bodyHeight / 2f, 0);
        DestroyImmediate(body.GetComponent<Collider>()); // 見た目用なのでコライダーは消す
        Colorize(body, new Color(0.12f, 0.12f, 0.12f));

        // 当たり判定は親(Robot)側に CapsuleCollider を付ける（すり抜け防止で安定）
        CapsuleCollider cap = GetComponent<CapsuleCollider>();
        if (cap == null) cap = gameObject.AddComponent<CapsuleCollider>();
        cap.direction = 1; // Y軸方向
        cap.radius = bodyDiameter / 2f;
        cap.height = bodyHeight;
        cap.center = new Vector3(0, bodyHeight / 2f, 0);

        // ポール（視覚のみ）
        GameObject pole = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pole.name = "Pole";
        pole.transform.SetParent(root);
        float poleBottom = bodyHeight;
        float poleTop = cameraTopHeight - 0.05f;
        float poleLen = poleTop - poleBottom;
        pole.transform.localScale = new Vector3(0.03f, poleLen / 2f, 0.03f);
        pole.transform.localPosition = new Vector3(0, poleBottom + poleLen / 2f, 0);
        DestroyImmediate(pole.GetComponent<Collider>());
        Colorize(pole, new Color(0.35f, 0.35f, 0.35f));

        // カメラ頭部（視覚のみ）
        GameObject head = GameObject.CreatePrimitive(PrimitiveType.Cube);
        head.name = "CameraHead";
        head.transform.SetParent(root);
        head.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
        head.transform.localPosition = new Vector3(0, cameraTopHeight - 0.05f, 0);
        DestroyImmediate(head.GetComponent<Collider>());
        Colorize(head, Color.white);

        // 車輪マーカー（左右・視覚のみ）
        MakeWheel(root, "Wheel_L", -wheelTrack / 2f);
        MakeWheel(root, "Wheel_R", wheelTrack / 2f);

        // カメラ頭頂部の計測点（回転軸の真上）
        Transform camTop = new GameObject("CameraTop").transform;
        camTop.SetParent(transform);
        camTop.localPosition = new Vector3(0, cameraTopHeight, 0);

        // Rigidbody設定
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        rb.mass = totalMass;
        rb.useGravity = true;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.centerOfMass = new Vector3(0, centerOfMassHeight, 0);

        Debug.Log("機体を組み立てました（質量" + totalMass + "kg / 重心" + centerOfMassHeight + "m / ヨーのみ回転可）");
    }

    void MakeWheel(Transform parent, string name, float x)
    {
        GameObject w = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        w.name = name;
        w.transform.SetParent(parent);
        w.transform.localScale = new Vector3(0.08f, 0.02f, 0.08f);
        w.transform.localRotation = Quaternion.Euler(0, 0, 90);
        w.transform.localPosition = new Vector3(x, 0.05f, 0);
        DestroyImmediate(w.GetComponent<Collider>());
        Colorize(w, new Color(0.05f, 0.05f, 0.05f));
    }

    void Colorize(GameObject go, Color c)
    {
        go.GetComponent<Renderer>().material.color = c;
    }
}