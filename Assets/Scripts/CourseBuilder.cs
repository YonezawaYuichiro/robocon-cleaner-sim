using UnityEngine;

public class CourseBuilder : MonoBehaviour
{
    [Header("コース寸法 (mm)")]
    public float overallWidthMM = 2400f;   // X方向 全体幅
    public float overallDepthMM = 3000f;   // Z方向 全体高さ
    public float corridorWidthMM = 600f;   // 通路幅

    [Header("壁の設定")]
    public float wallHeight = 0.5f;        // 壁の高さ(m)
    public float wallThickness = 0.05f;    // 外周壁の厚み(m)

    private Transform courseRoot;
    private float W, D, C; // メートル換算

    [ContextMenu("コースを生成する")]
    public void BuildCourse()
    {
        Transform old = transform.Find("CourseGeometry");
        if (old != null) DestroyImmediate(old.gameObject);
        courseRoot = new GameObject("CourseGeometry").transform;
        courseRoot.SetParent(transform);

        W = overallWidthMM / 1000f;
        D = overallDepthMM / 1000f;
        C = corridorWidthMM / 1000f;

        BuildFloor();
        BuildOuterWalls();
        BuildInternalWalls();
        BuildLines();

        Debug.Log("コース生成完了");
    }

    void BuildFloor()
    {
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = "Floor";
        floor.transform.SetParent(courseRoot);
        floor.transform.position = new Vector3(W / 2f, -0.05f, D / 2f);
        floor.transform.localScale = new Vector3(W, 0.1f, D);
    }

    void BuildOuterWalls()
    {
        float t = wallThickness;
        CreateBox("Wall_Bottom", new Vector3(W / 2f, wallHeight / 2f, -t / 2f),    new Vector3(W + 2 * t, wallHeight, t));
        CreateBox("Wall_Top",    new Vector3(W / 2f, wallHeight / 2f, D + t / 2f), new Vector3(W + 2 * t, wallHeight, t));
        CreateBox("Wall_Left",   new Vector3(-t / 2f, wallHeight / 2f, D / 2f),    new Vector3(t, wallHeight, D));
        CreateBox("Wall_Right",  new Vector3(W + t / 2f, wallHeight / 2f, D / 2f), new Vector3(t, wallHeight, D));
    }

    void BuildInternalWalls()
    {
        // 下段 z∈[0,C], 中段 z∈[2C,3C], 上段 z∈[4C,5C]  (D=5C=3.0)
        BuildGapWall("Lower", C, 2 * C, false);   // 下段↔中段の通路は左(x:0〜600)
        BuildGapWall("Upper", 3 * C, 4 * C, true); // 中段↔上段の通路は右(x:1800〜2400)
    }

    // z0..z1 の帯を壁で埋め、connectorOnRight 側に幅Cの通路穴を残す
    void BuildGapWall(string name, float z0, float z1, bool connectorOnRight)
    {
        float zc = (z0 + z1) / 2f;
        float thick = z1 - z0;
        float wallW = W - C;

        float centerX = connectorOnRight ? (wallW / 2f)      // 右に穴 → 壁は左寄り
                                         : (C + wallW / 2f); // 左に穴 → 壁は右寄り
        CreateBox($"Wall_{name}", new Vector3(centerX, wallHeight / 2f, zc), new Vector3(wallW, wallHeight, thick));
    }

    void BuildLines()
    {
        float halfC = C / 2f;
        // スタート(赤)：下段、左下マスの入口 x=600（右下(0,0)基準で1800）
        CreateLine("StartLine", new Vector3(C, 0.01f, halfC), Color.red);
        // ゴール(緑)：上段、左上の一つ右マスの入口 x=1800
        CreateLine("GoalLine", new Vector3(3f * C, 0.01f, D - halfC), Color.green);
    }

    void CreateLine(string name, Vector3 center, Color color)
    {
        GameObject line = GameObject.CreatePrimitive(PrimitiveType.Cube);
        line.name = name;
        line.transform.SetParent(courseRoot);
        line.transform.position = center;
        line.transform.localScale = new Vector3(0.02f, 0.02f, C); // 通路を横切る向き
        line.GetComponent<BoxCollider>().isTrigger = true;
        line.GetComponent<Renderer>().material.color = color;
    }

    void CreateBox(string name, Vector3 center, Vector3 size)
    {
        GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = name;
        box.transform.SetParent(courseRoot);
        box.transform.position = center;
        box.transform.localScale = size;
    }
}