using UnityEngine;                 // Unity本体のクラス群（Rigidbody, Vector3, Mathfなど）
using System.Collections.Generic;  // List<T> を使うため
using System.IO;                   // CSVファイル書き出し（Path, File）に使う

// ======================================================================================
//  RobotController ── 初心者向け全体マップ（まず地図、それから細部）
// ======================================================================================
//
// ■ このプログラムは何？
//   「ロボット掃除機」を、あらかじめ決めた順路（前進・旋回のリスト）に沿って
//   S字クランクのコースを自動で走らせる制御プログラム。走りながら、カメラ頂部の
//   加速度などを計測し、最後にCSVとサマリを出力する。
//
// ■ 3つの大事な考え方（これさえ掴めば全部読める）
//   1) 命令リスト方式：やることを Command のリスト（前進1.8m→左旋回90°→…）にして、
//      currentCommandIndex で「今どれを実行中か」を1つずつ進める（＝状態機械）。
//   2) 力で動かす：位置を直接書き換えず、毎フレーム Rigidbody に「力」と「トルク」を
//      加えて物理的に動かす（課題の必須条件）。前進力とヨートルクを、左右2輪それぞれの
//      駆動力に分解して加える＝2輪差動駆動のモデル化。
//   3) 加速度の上限：カメラ頂部（床から1.0m）の水平加速度を常に1.0m/s²以下に保つ。
//      そのため「加速のきつさ」を制限し、旋回は“その場回転”にして加速度を使わない。
//
// ■ 部品どうしのつながり（1周の流れ）
//
//     [命令リスト] --currentCommandIndex--> FixedUpdate()（毎物理ステップ）
//          │                                   │
//          │  Forwardなら DriveStraight()、Spinなら Spin() を呼ぶ
//          │                                   ▼
//          │           「前進力 forwardForce」「ヨートルク yawTorque」を決める
//          │                                   ▼
//          │              ApplyWheelForces() が左右2輪の力に分解 → Rigidbodyへ
//          ▼                                   ▼
//     区間到達で NextCommand() ← 物理エンジンがロボットを動かす（次フレームに反映）
//
//   ・スタート/ゴールの「線」は別オブジェクト（当たり判定＝トリガー）。ロボットが
//     その線を通ると OnTriggerEnter/Exit が呼ばれ、走破時間の計測が始まる/止まる。
//     ＝“線の名前(StartLine/GoalLine)”がこのスクリプトと線を結ぶ合言葉。
//   ・壁に触れると OnCollisionEnter が呼ばれ「無効」と警告する。
//
// ■ 一連の具体例（最初の1区間で何が起きるか）
//   1) Startで命令リストを作り、最初は Forward 1.8m。ロボットは -X（左）向きで待機。
//   2) FixedUpdateが DriveStraight を呼び、残り距離から目標速度を計算。
//   3) SpeedToForce が目標速度に近づく“前進力”を、HoldTorque が“まっすぐ保つトルク”を返す。
//   4) ApplyWheelForces が左右輪に力を配分して Rigidbody に加える → ロボットが進む。
//   5) 途中で赤い StartLine を通過 → OnTriggerEnter で計測開始（elapsedTimeが増え出す）。
//   6) 1.8m進むと DriveStraight が NextCommand を呼び、次の命令（左旋回90°）へ。
//
// ■ ミニ用語集（この分野の言葉）
//   ・Rigidbody      : Unityの物理エンジンが動かす“剛体”。質量・速度・力を持つ。
//   ・FixedUpdate    : 物理計算のタイミングで一定間隔で呼ばれる関数（毎フレームとは別）。
//   ・力(Force)/トルク: 力＝押す/引く、トルク＝回す力。速度や角速度を変える原因。
//   ・ヨー(yaw)       : 上下軸(Y)まわりの回転（＝左右の向き変え）。
//   ・超信地旋回      : 左右輪を逆回しして“その場で”回る旋回（車体中心が動かない）。
//   ・合成加速度      : 前後＋左右の加速度をベクトル合成した大きさ（水平方向のみ見る）。
//   ・ジャーク        : 加速度の変化率（加速度が急に変わるほど大きい＝ガクガク度）。
//   ・PD制御         : 誤差(P)とその変化速度(D)からトルクを決める、揺れにくい制御法。
//   ・トリガー        : すり抜ける当たり判定。重なると OnTriggerEnter/Exit が呼ばれる。
//
// ■ 呼び出しマップ（誰が誰を呼ぶか）
//   Unityが自動で呼ぶ入口（ライフサイクル/イベント）:
//     Start()               … 起動時に1回。→ BuildCommands(), CamTopVelocity()
//     FixedUpdate()         … 物理ステップ毎。→ DriveStraight()/Spin()/SpeedToForce()/
//                                HoldTorque()/ApplyWheelForces()/ApplyLateralFriction()/
//                                Measure()/WriteSummary()
//     OnTriggerEnter/Exit() … スタート/ゴール線を通過した時（計測の開始/終了）
//     OnCollisionEnter()    … 壁に接触した時（無効判定の警告）
//   区間の動きを作る中身:
//     DriveStraight() … 直進。→ SpeedToForce(), HoldTorque(), CurrentForwardSpeed(), NextCommand()
//     Spin()          … その場旋回。→ SpeedToForce(), HoldTorque(), CurrentYawRateDeg(), NextCommand()
//   共通の道具（小さな部品）:
//     SpeedToForce()      … 目標速度→前進力（加速度とジャークを制限）
//     HoldTorque()        … 目標方位へ向けるPDトルク
//     ApplyWheelForces()  … 前進力＋トルクを左右2輪の力に分解してRigidbodyへ
//     ApplyLateralFriction() … 横滑りを摩擦力で消す
//     NextCommand()       … 次の命令へ進める
//     Current*/CamTop*    … 現在の前進速度・角速度・カメラ頂部の位置/速度を計算
//     Measure()/WriteSummary() … 計測とCSV/サマリ出力
//   読み方のコツ：Unityが呼ぶ「入口」は Start/FixedUpdate/On～ の4種だけ。
//                残りは全部、その入口が使う“道具”。
// ======================================================================================

[RequireComponent(typeof(Rigidbody))]   // このスクリプトは必ずRigidbodyと一緒に付ける（無いと自動追加）
public class RobotController : MonoBehaviour
{
    // 命令の種類。Forward=前進, Backward=後退, SpinLeft/Right=その場で左/右旋回
    public enum CommandType { Forward, Backward, SpinLeft, SpinRight }

    // 1つの命令（種類＋数値）。Inspectorに出せるよう Serializable にしている
    [System.Serializable]
    public struct Command
    {
        public CommandType type;
        public float value; // Forward/Backward=距離(m), Spin=角度(度)
        public Command(CommandType t, float v) { type = t; value = v; }
    }

    // ── 以下 public 変数は、Unityの Inspector で見た目・調整できる“つまみ” ──

    [Header("車体諸元（課題の必須条件に合わせる）")]
    public float wheelHalfTrack = 0.15f;   // 車体中心から各駆動輪までの距離(m)
                                           //   【調整】大→左右輪の間隔が広く、同じ力差で回りやすい（旋回トルク大）。
                                           //          小→回りにくい。実機モデルの車輪位置に合わせる値。
    public float camHeight = 1.0f;         // カメラ頂部の高さ(m)。加速度を測る点の高さ（必須条件=1.0）
    public float comHeight = 0.5f;         // 重心高さ(m)（必須条件=0.5）。カメラ頂部の計算にも使う
    public bool setCenterOfMass = true;    // 重心高さをRigidbodyに設定するか
                                           //   【調整】false→Unityが自動計算した重心を使う（transform原点が床でない時など）

    [Header("加速度制約（必須条件）")]
    public float maxCamAccel = 1.0f;       // カメラ頂部の水平合成加速度 上限(m/s²)
                                           //   【調整】これは“ルールの上限”。基本1.0のまま。超えると無効。
    public float accelSafety = 0.9f;       // 実際に使う割合（上限に対する安全マージン）
                                           //   【調整】大(→1.0)→速いが上限ギリギリで危険。小→余裕大だが遅い。
    public float maxJerk = 3.0f;           // 加速度指令の変化率上限(m/s³)（ジャーク抑制）
                                           //   【調整】大→加速をパッと切替（機敏だがジャーク大＝ガクガク・採点減）。
                                           //          小→なめらかだが反応が鈍く、止まり際に行き過ぎやすい。

    [Header("直進制御")]
    public float speedKp = 4.0f;             // 目標速度への追従の強さ（P制御ゲイン）
                                             //   【調整】大→キビキビ追従、大きすぎると振動。小→もたつく。
    public float positionTolerance = 0.005f; // 到達判定(m)。残り距離がこれ以下で「着いた」とみなす
    public float stopSpeed = 0.02f;          // 停止とみなす速度(m/s)。これ未満で次の命令へ進む
    public float brakeSafety = 0.35f;        // 減速プロファイル安全率（小=早めに減速＝突っ込み防止）
                                             //   【調整】小→かなり手前から減速（安全・遅い）。大→ギリギリ減速
                                             //          （速いが、止まり際に壁へ突っ込むリスク）。現状0.35で完走。

    [Header("旋回制御（超信地旋回：中心まわり）")]
    public float spinAccelMax = 200f;      // 旋回の角加速度上限(deg/s²)。旋回トルクの上限にも効く
                                           //   【調整】大→速く回るがジャーク増。小→ゆっくり滑らか。
    public float maxYawRate = 150f;        // 角速度上限(deg/s)  ※現在は未使用（PD制御に変更したため）
    public float spinOmegaKp = 10f;        // 旋回の角速度追従ゲイン ※現在は未使用（PD制御に変更したため）
    public float headingKp = 20f;          // 方位保持P：目標角との差に比例して回す強さ
                                           //   【調整】大→目標角へ強く引く。headingKdと組で臨界減衰(≈ζ1)に調整済み。
    public float headingKd = 9f;           // 方位保持D：角速度にブレーキ（行き過ぎ＝オーバーシュート抑制）
                                           //   【調整】小→揺れて何回転もする恐れ。大→戻りが鈍い。
    public float maxSpinTorque = 5f;       // 旋回トルクの安全上限(N·m)。暴走防止のクランプ
    public float angleTolerance = 1.0f;    // 角度到達判定(度)。目標角との差がこれ以下で旋回完了

    [Header("横滑り抑制（非ホロノミック拘束）")]
    public float lateralGrip = 20f;        // 横速度を消す強さ（タイヤの横グリップの代わり）
                                           //   【調整】大→横滑りを素早く止め通路中心を保つ。小→ズルズル滑る。

    [Header("状態表示（読み取り専用）")]
    public int currentCommandIndex = 0;    // 今実行中の命令番号（Inspectorで進行が見える）
    public string currentPhase = "待機";   // 今の状態の文字（前進0/旋回1/完了 など）
    public bool timing = false;            // 計測中か（スタート通過〜ゴール通過）
    public float elapsedTime = 0f;         // 走破時間（スタートライン〜ゴールライン）

    [Header("計測（レポート用・読み取り専用）")]
    public float fixedTimestep = 0f;       // Fixed Timestep(s)。物理1ステップの時間（提出項目）
    public float maxCompositeAccel = 0f;   // カメラ頂部 水平合成加速度の最大(m/s²)。1.0以下の証拠
    public float avgCompositeAccel = 0f;   // 走破中の平均(m/s²)
    public float maxSpeed = 0f;            // 最大速度(m/s)
    public float maxAngularVel = 0f;       // 最大角速度(deg/s)
    public float maxJerkMeasured = 0f;     // 最大ジャーク(m/s³)

    [Header("デバッグ")]
    public bool verboseLog = false;        // ON→0.5秒ごとに状態をConsoleへ出力
    public float csvInterval = 0.02f;      // CSV記録間隔(s)。大きいほど行数が減る
                                           //   【調整】0.02→約50Hz。大きく→行数減（Excelで軽い）が粗くなる。

    // ── 以下 private 変数は、内部だけで使う作業用の値（Inspectorには出ない） ──
    private Rigidbody rb;                                          // 動かす対象の剛体
    private readonly List<Command> commands = new List<Command>(); // 命令リスト（順路）
    private Vector3 segmentStartPos;                              // 今の区間を始めた位置（進んだ距離の起点）
    private float segmentStartAngle;                             // 今の区間を始めた向き（角度の起点）
    private bool finished = false;                                // 全命令を終えたか

    // 制御・計測の内部状態
    private float accelCmdPrev = 0f;       // 前フレームの加速度指令（ジャーク制限に使う）
    private float maxLinAccel;             // = maxCamAccel * accelSafety（実際に使う直線加速度の上限）
    private float yawInertia = 0.1f;       // ヨー慣性(kg·m²)（Startで実測）。トルク＝慣性×角加速度に使う
    private Vector3 camVelPrev;            // 前フレームのカメラ頂部速度（加速度を差分で出すため）
    private Vector3 camAccelPrev;          // 前フレームのカメラ頂部加速度（ジャークを差分で出すため）
    private int measureWarmup = 3;         // 計測開始直後の数フレームは無視（差分の立ち上がりノイズ除け）
    private float logTimer = 0f;           // verboseLogの間隔管理
    private float csvTimer = 0f;           // CSV間引きの間隔管理
    private float simTime = 0f;            // Play開始からの経過(s)
    private double accelSum = 0.0;         // 平均計算用（走破中の加速度の合計）
    private long accelCount = 0;           // 平均計算用（サンプル数）
    private bool summaryWritten = false;   // サマリ/CSVを書いたか（1回だけにするため）
    private readonly List<string> csvRows = new List<string>();   // CSVの各行をためる箱

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : Unity（Play開始時に自動で1回）
    // │ 呼ぶ物 : BuildCommands(), CamTopVelocity()
    // │ 目的   : 部品の準備（剛体取得・重心/慣性の設定・命令作成・計測の初期化）
    // └─────────────────────────────────────────────
    void Start()
    {
        rb = GetComponent<Rigidbody>();   // 同じオブジェクトのRigidbodyを取得（これに力を加える）

        // kinematicだとAddForceが効かない＝物理で動かせないので警告
        if (rb.isKinematic)
            Debug.LogError("Rigidbody が kinematic です。物理駆動できません。Is Kinematic を外してください。", this);

        if (setCenterOfMass)
            rb.centerOfMass = new Vector3(0f, comHeight, 0f); // ※transform原点が床レベル前提。重心を0.5mに

        maxLinAccel = maxCamAccel * accelSafety;  // 実際に使う直線加速度の上限（例:1.0×0.9=0.9）
        yawInertia = rb.inertiaTensor.y;          // ヨー方向の慣性を実測（トルク計算に使う）
        if (yawInertia <= 1e-4f || float.IsNaN(yawInertia)) yawInertia = 0.1f; // 変な値なら安全な既定値

        BuildCommands();                          // 順路（命令リスト）を作る
        segmentStartPos = rb.worldCenterOfMass;   // 最初の区間の起点位置
        segmentStartAngle = transform.eulerAngles.y; // 最初の区間の起点角度
        camVelPrev = CamTopVelocity();            // 加速度を差分で出すための初期値

        fixedTimestep = Time.fixedDeltaTime;      // 物理ステップ時間を記録（提出項目）
        // CSVの見出し行を先頭に入れる
        csvRows.Add("simTime_s,runTime_s,timing,phase,aCam_mps2,speed_mps,yawRate_dps,jerk_mps3");

        // 初期状態をConsoleに表示（質量・重心・回転拘束・加速度上限・物理ステップ）
        Debug.Log($"[初期化] mass={rb.mass} CoM(local)={rb.centerOfMass} " +
                  $"constraints={rb.constraints} maxLinAccel={maxLinAccel:F2} " +
                  $"FixedTimestep={fixedTimestep:F4}s", this);
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : Start()
    // │ 呼ぶ物 : なし（リストに詰めるだけ）
    // │ 目的   : S字クランクの順路を、前進/旋回の命令列として定義する
    // └─────────────────────────────────────────────
    // コースを命令列に変換（S字クランク：PDFマップに一致）
    // マップ: 原点左下, X=幅0〜2.4(4列), Z=奥0〜3.0(5行), 1マス0.6
    // スタート=右下(2.1,0.3)で -X 向き。下段↔中段の通路は左(x0〜0.6)、中段↔上段は右(x1.8〜2.4)。
    // 距離: 水平=1.8m, 縦=1.2m
    void BuildCommands()
    {
        commands.Clear();
        commands.Add(new Command(CommandType.Forward, 1.8f));   // 下段を左へ (2.1→0.3)
        commands.Add(new Command(CommandType.SpinLeft, 90f));   // -X→+Z（上を向く）
        commands.Add(new Command(CommandType.Forward, 1.2f));   // 左の縦通路で中段へ (0.3→1.5)
        commands.Add(new Command(CommandType.SpinLeft, 90f));   // +Z→+X（右を向く）
        commands.Add(new Command(CommandType.Forward, 1.8f));   // 中段を右へ (0.3→2.1)
        commands.Add(new Command(CommandType.SpinRight, 90f));  // +X→+Z（上を向く）
        commands.Add(new Command(CommandType.Forward, 1.2f));   // 右の縦通路で上段へ (1.5→2.7)
        commands.Add(new Command(CommandType.SpinRight, 90f));  // +Z→-X（左を向く）
        commands.Add(new Command(CommandType.Forward, 1.8f));   // 上段を左へ、ゴール通過→左上 (2.1→0.3)
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : Unity（物理ステップ毎に自動）＝このプログラムの“心臓”
    // │ 呼ぶ物 : DriveStraight()/Spin()/SpeedToForce()/HoldTorque()/
    // │          ApplyWheelForces()/ApplyLateralFriction()/Measure()/WriteSummary()
    // │ 目的   : 今の命令に応じて力を決め、Rigidbodyに加え、計測する（毎ステップ）
    // └─────────────────────────────────────────────
    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;   // この1ステップの時間(s)

        float forwardForce = 0f;   // 前進方向の合力(N)   … このステップで加える前進力
        float yawTorque = 0f;      // ヨー方向の合トルク(N·m, +yまわり) … このステップで加える回す力

        if (finished || currentCommandIndex >= commands.Count)
        {
            // すべての命令を終えた後：完了状態にして、その場に止め続ける
            if (!finished) { finished = true; currentPhase = "完了"; }
            forwardForce = SpeedToForce(0f);            // 目標速度0＝止める
            yawTorque = HoldTorque(segmentStartAngle);  // 今の向きを保持
        }
        else
        {
            // 今の命令を取り出し、種類に応じて動きを計算（forwardForce/yawTorqueを埋める）
            Command cmd = commands[currentCommandIndex];
            switch (cmd.type)
            {
                case CommandType.Forward:
                    DriveStraight(cmd.value, +1f, out forwardForce, out yawTorque);  // 前進(dir=+1)
                    break;
                case CommandType.Backward:
                    DriveStraight(cmd.value, -1f, out forwardForce, out yawTorque);  // 後退(dir=-1)
                    break;
                case CommandType.SpinLeft:
                    Spin(+1f, cmd.value, out forwardForce, out yawTorque);           // 左旋回(sign=+1)
                    break;
                case CommandType.SpinRight:
                    Spin(-1f, cmd.value, out forwardForce, out yawTorque);           // 右旋回(sign=-1)
                    break;
            }
        }

        ApplyWheelForces(forwardForce, yawTorque);  // 決めた力/トルクを左右輪に分けてRigidbodyへ
        ApplyLateralFriction();                     // 横滑りを消す（毎ステップ）

        simTime += dt;                              // シミュ全体の経過時間
        if (timing) elapsedTime += dt;              // 計測中だけ走破時間を進める

        Measure(dt);                                // 加速度など計測＆CSV記録

        if (finished && !summaryWritten) WriteSummary();  // 完了した瞬間に一度だけ結果出力
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : Unity（ロボットがトリガー(線)に入った時）
    // │ 呼ぶ物 : なし
    // │ 目的   : スタートライン通過で計測開始（“StartLine”という名前が合言葉）
    // └─────────────────────────────────────────────
    // スタート/ゴールラインのトリガーで計測を開始・終了
    void OnTriggerEnter(Collider other)
    {
        if (!timing && !summaryWritten && other.name == "StartLine")  // まだ計測前で、相手がStartLineなら
        {
            timing = true;                              // 計測開始（以後 elapsedTime が増える）
            Debug.Log($"▶ 計測開始（スタートライン通過）", this);
        }
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : Unity（ロボットがトリガー(線)から出た時）
    // │ 呼ぶ物 : なし
    // │ 目的   : ゴールラインを“完全に通過”した瞬間に計測終了
    // └─────────────────────────────────────────────
    void OnTriggerExit(Collider other)
    {
        if (timing && other.name == "GoalLine")   // 計測中で、相手がGoalLineなら
        {
            timing = false;                        // 計測終了（走破時間が確定）
            Debug.Log($"■ 計測終了（ゴールライン通過） 走破時間={elapsedTime:F3}s", this);
        }
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : Unity（ロボットが何かにぶつかった時）
    // │ 呼ぶ物 : なし
    // │ 目的   : 壁接触を検知して「無効」と警告（規則：壁接触＝無効）
    // └─────────────────────────────────────────────
    // 壁接触の検知（規則：壁接触＝無効）
    void OnCollisionEnter(Collision c)
    {
        if (c.gameObject.name.StartsWith("Wall"))   // ぶつかった相手の名前が"Wall"で始まれば壁
            Debug.LogWarning($"⚠ 壁に接触（{c.gameObject.name}）→ このランは無効です", this);
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : FixedUpdate()（命令がForward/Backwardのとき）
    // │ 呼ぶ物 : SpeedToForce(), HoldTorque(), CurrentForwardSpeed(), NextCommand()
    // │ 目的   : 目標距離まで、加速度制限を守りつつ最短で直進し、着いたら次へ
    // └─────────────────────────────────────────────
    // --- 直進（減速距離から目標速度を決める時間最適プロファイル）---
    void DriveStraight(float distance, float dir, out float forwardForce, out float yawTorque)
    {
        currentPhase = (dir > 0 ? "前進 " : "後退 ") + currentCommandIndex;  // 表示用の状態文字

        Vector3 fwd = transform.forward;   // ロボットの前方向（ワールド座標）
        // 起点からの移動量を前方向に射影＝この区間で進んだ距離（dirで前進/後退の符号を合わせる）
        float traveled = Vector3.Dot(rb.worldCenterOfMass - segmentStartPos, fwd) * dir;
        float remaining = distance - traveled;   // あと何m残っているか

        if (remaining <= positionTolerance)  // ほぼ着いた
        {
            forwardForce = SpeedToForce(0f);           // 止めにかかる
            yawTorque = HoldTorque(segmentStartAngle); // 向きは保持
            if (Mathf.Abs(CurrentForwardSpeed()) < stopSpeed)  // 十分止まったら
                NextCommand();                                  // 次の命令へ
            return;
        }

        // 減速に必要な距離 = v²/(2a) より、残り距離から到達可能な目標速度を決める
        // 安全率で減速計画を弱め（早め減速）、実際の減速力(maxLinAccel)に余裕を残す＝突っ込み防止
        float brakeAccel = maxLinAccel * brakeSafety;              // 計画上の減速（実際の力より弱めに）
        float vBrake = Mathf.Sqrt(2f * brakeAccel * remaining);    // 残り距離で止まれる速度
        float targetSpeed = vBrake * dir;                         // 目標速度（前進/後退の向き付き）

        forwardForce = SpeedToForce(targetSpeed);  // 目標速度に近づける前進力
        yawTorque = HoldTorque(segmentStartAngle); // まっすぐ保つ
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : FixedUpdate()（命令がSpinLeft/SpinRightのとき）
    // │ 呼ぶ物 : SpeedToForce(), HoldTorque(), CurrentYawRateDeg(), NextCommand()
    // │ 目的   : その場で目標角まで回り（前進は止める）、着いたら次へ
    // └─────────────────────────────────────────────
    // --- 超信地旋回（中心まわり回転：カメラ頂部の水平加速度は原理的にほぼ0）---
    // 目標方位へのPD制御。DeltaAngleは常に近道(±180)を返すので、通り越しても戻り、暴走しない。
    void Spin(float sign, float angleDeg, out float forwardForce, out float yawTorque)
    {
        currentPhase = "旋回 " + currentCommandIndex;

        forwardForce = SpeedToForce(0f); // 前進成分は消して純粋なその場旋回に

        float targetAngle = segmentStartAngle + sign * angleDeg;              // 目標の絶対角（起点±90°）
        float err = Mathf.DeltaAngle(transform.eulerAngles.y, targetAngle);   // 目標までの角度差（近道の符号付き）

        yawTorque = HoldTorque(targetAngle);   // 目標角へ向けて回すPDトルク

        // 目標角に十分近く、かつほぼ止まっていれば旋回完了→次へ
        if (Mathf.Abs(err) <= angleTolerance && Mathf.Abs(CurrentYawRateDeg()) < 5f)
            NextCommand();
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : DriveStraight(), Spin(), FixedUpdate()(完了時)
    // │ 呼ぶ物 : CurrentForwardSpeed()
    // │ 目的   : 「目標速度」を実現する前進力を、加速度上限とジャーク上限を守って返す
    // └─────────────────────────────────────────────
    // 目標前進速度 → 必要な前進力（加速度上限・ジャーク上限を尊重）
    float SpeedToForce(float targetSpeed)
    {
        float speedErr = targetSpeed - CurrentForwardSpeed();                     // 目標と現在の速度差
        float accelTarget = Mathf.Clamp(speedKp * speedErr, -maxLinAccel, maxLinAccel); // 望む加速度(上限でクランプ)
        // ジャーク制限：加速度指令の変化率を制限（前回値から maxJerk×dt しか変えない）
        float accelCmd = Mathf.MoveTowards(accelCmdPrev, accelTarget, maxJerk * Time.fixedDeltaTime);
        accelCmdPrev = accelCmd;              // 次フレームのために記録
        return rb.mass * accelCmd;            // 力 = 質量 × 加速度（F=ma）
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : DriveStraight()(直進の蛇行防止), Spin()(旋回), FixedUpdate()(完了時)
    // │ 呼ぶ物 : CurrentYawRateDeg()
    // │ 目的   : 目標の向きへ回すPDトルクを、慣性を使って物理トルクに換算して返す
    // └─────────────────────────────────────────────
    // 目標方位へ向けPD制御（直進中の蛇行防止・旋回後の保持）。慣性ベースでトルク化。
    float HoldTorque(float targetAngleDeg)
    {
        float err = Mathf.DeltaAngle(transform.eulerAngles.y, targetAngleDeg); // deg  目標までの角度差
        // 角加速度指令 = P(誤差) - D(角速度)。上限でクランプ（暴れ防止）
        float angAccel = Mathf.Clamp(headingKp * err - headingKd * CurrentYawRateDeg(),
                                     -spinAccelMax, spinAccelMax);              // deg/s²
        // トルク = 慣性 × 角加速度（rad換算）。上限クランプで安全に
        return Mathf.Clamp(yawInertia * angAccel * Mathf.Deg2Rad, -maxSpinTorque, maxSpinTorque);
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : FixedUpdate()
    // │ 呼ぶ物 : Rigidbody.AddForceAtPosition（物理エンジンへ出力）
    // │ 目的   : 前進力とヨートルクを「左右2輪の駆動力」に分解して物理的に加える
    // └─────────────────────────────────────────────
    // 前進力＋ヨートルクを「左右2輪の駆動力」に分解して物理適用
    void ApplyWheelForces(float forwardForce, float yawTorque)
    {
        // 連立を解いて左右輪の力を求める：
        //   制約:  fL + fR = forwardForce                     （合計＝前進力）
        //          正味ヨートルク(+y) = halfTrack·(fL - fR) = yawTorque （差＝回す力）
        float fL = forwardForce * 0.5f + yawTorque / (2f * wheelHalfTrack);  // 左輪の前進力
        float fR = forwardForce * 0.5f - yawTorque / (2f * wheelHalfTrack);  // 右輪の前進力

        Vector3 fwd = transform.forward;   // 力を加える向き（前方）
        // 左右輪の“接地点”のワールド位置（ローカルの±halfTrackを世界座標へ）
        Vector3 leftWheel = transform.TransformPoint(new Vector3(-wheelHalfTrack, 0f, 0f));
        Vector3 rightWheel = transform.TransformPoint(new Vector3(+wheelHalfTrack, 0f, 0f));

        // 各輪の位置に前方向の力を加える＝合計で前進、差で回転が生まれる（差動駆動）
        rb.AddForceAtPosition(fwd * fL, leftWheel, ForceMode.Force);
        rb.AddForceAtPosition(fwd * fR, rightWheel, ForceMode.Force);
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : FixedUpdate()
    // │ 呼ぶ物 : Rigidbody.AddForce
    // │ 目的   : 横（車軸方向）の滑りを摩擦力で打ち消す＝タイヤの横グリップの代役
    // └─────────────────────────────────────────────
    // 横滑り抑制：横速度を消す向きの摩擦力（加速度予算を超えないようクランプ）
    void ApplyLateralFriction()
    {
        Vector3 right = transform.right;                             // ロボットの真横方向
        float lateralSpeed = Vector3.Dot(rb.linearVelocity, right);  // 横方向の速度成分
        // 横速度を打ち消す加速度（上限クランプで加速度予算を守る）
        float lateralAccel = Mathf.Clamp(-lateralSpeed * lateralGrip, -maxLinAccel, maxLinAccel);
        rb.AddForce(right * rb.mass * lateralAccel, ForceMode.Force); // 横向きの摩擦力を加える
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : DriveStraight(), Spin()（区間が終わった時）
    // │ 呼ぶ物 : なし
    // │ 目的   : 次の命令へ進め、新しい区間の起点（位置・角度）を記録し直す
    // └─────────────────────────────────────────────
    void NextCommand()
    {
        currentCommandIndex++;                       // 命令番号を1つ進める
        segmentStartPos = rb.worldCenterOfMass;      // 新区間の距離の起点
        segmentStartAngle = transform.eulerAngles.y; // 新区間の角度の起点
        accelCmdPrev = 0f;                           // 加速度指令の履歴をリセット
    }

    // 現在の“前進方向”の速度(m/s)＝速度ベクトルを前方向へ射影
    float CurrentForwardSpeed() => Vector3.Dot(rb.linearVelocity, transform.forward);
    // 現在のヨー角速度(deg/s)＝角速度のY成分をラジアン→度に変換
    float CurrentYawRateDeg() => rb.angularVelocity.y * Mathf.Rad2Deg;

    // カメラ頂部のワールド位置＝重心の真上（頂部高さ−重心高さ）だけ上
    Vector3 CamTopWorldPos() => rb.worldCenterOfMass + Vector3.up * (camHeight - comHeight);
    // カメラ頂部の速度＝剛体上のその点の速度（回転の効果も含む）
    Vector3 CamTopVelocity() => rb.GetPointVelocity(CamTopWorldPos());

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : FixedUpdate()（毎ステップ）
    // │ 呼ぶ物 : CamTopVelocity(), CurrentYawRateDeg()
    // │ 目的   : カメラ頂部の加速度/ジャーク等を差分で測り、最大・平均・CSVを更新
    // └─────────────────────────────────────────────
    // レポート用メトリクスの実測（カメラ頂部の水平合成加速度・速度・角速度・ジャーク）
    void Measure(float dt)
    {
        Vector3 camVel = CamTopVelocity();               // 今のカメラ頂部速度
        Vector3 camAccel = (camVel - camVelPrev) / dt;   // 加速度＝速度の変化 ÷ 時間
        camAccel.y = 0f; // 水平成分のみ（重力は含めない）… 上下成分を捨てる
        float aMag = camAccel.magnitude;                 // 水平合成加速度の大きさ
        float jerk = ((camAccel - camAccelPrev) / dt).magnitude; // ジャーク＝加速度の変化 ÷ 時間

        if (measureWarmup > 0)
        {
            measureWarmup--; // 初期の数値ノイズ（差分の立ち上がり）は除外
        }
        else
        {
            // 最大値は全区間で監視（1.0制約の証明）
            maxCompositeAccel = Mathf.Max(maxCompositeAccel, aMag);
            maxSpeed = Mathf.Max(maxSpeed, rb.linearVelocity.magnitude);
            maxAngularVel = Mathf.Max(maxAngularVel, Mathf.Abs(CurrentYawRateDeg()));
            maxJerkMeasured = Mathf.Max(maxJerkMeasured, jerk);

            // 上限を超えたら警告（本来は起きない想定＝制約遵守の自己チェック）
            if (aMag > maxCamAccel + 1e-3f)
                Debug.LogWarning($"⚠ カメラ頂部の水平加速度が上限超過: {aMag:F3} m/s²（上限{maxCamAccel}）", this);

            // 平均は走破中（計測中）のみ
            if (timing)
            {
                accelSum += aMag;                          // 合計に足し
                accelCount++;                              // 個数を数え
                avgCompositeAccel = (float)(accelSum / accelCount); // 平均を更新
            }

            // 時系列CSV（csvInterval秒ごとに間引いて記録。timing列で走破区間を判別可能）
            csvTimer += dt;
            if (!summaryWritten && csvTimer >= csvInterval)
            {
                csvTimer -= csvInterval;                   // 次の記録まで待つ
                // 1行＝時刻・走破時間・計測中フラグ・状態・加速度・速度・角速度・ジャーク
                csvRows.Add($"{simTime:F3},{elapsedTime:F3},{(timing ? 1 : 0)},{currentPhase}," +
                            $"{aMag:F4},{rb.linearVelocity.magnitude:F4},{CurrentYawRateDeg():F3},{jerk:F3}");
            }
        }

        camVelPrev = camVel;      // 次フレームの差分計算のため保存
        camAccelPrev = camAccel;

        if (verboseLog)  // ONなら0.5秒ごとに状態を出力
        {
            logTimer += dt;
            if (logTimer >= 0.5f)
            {
                logTimer = 0f;
                Debug.Log($"[{currentPhase}] t={elapsedTime:F2} v={rb.linearVelocity.magnitude:F3} " +
                          $"ω={CurrentYawRateDeg():F1}deg/s aCam={aMag:F3} " +
                          $"(max: a={maxCompositeAccel:F3} v={maxSpeed:F3} jerk={maxJerkMeasured:F2})", this);
            }
        }
    }

    // ┌─ relationship ──────────────────────────────
    // │ 呼ぶ人 : FixedUpdate()（全命令を終えた瞬間に1回）
    // │ 呼ぶ物 : File.WriteAllLines（CSVをディスクへ）
    // │ 目的   : 結果サマリをConsoleへ、時系列データをCSVファイルへ出力
    // └─────────────────────────────────────────────
    // 走行終了時：サマリ表示＋時系列CSV書き出し（1回だけ）
    void WriteSummary()
    {
        summaryWritten = true;   // 二度書きしないようフラグを立てる

        // 主要な計測結果をまとめてConsoleに表示（レポートに転記できる）
        Debug.Log("===== 走行結果サマリ =====\n" +
                  $"走破時間            : {elapsedTime:F3} s\n" +
                  $"最大合成加速度(頂部): {maxCompositeAccel:F3} m/s²（上限{maxCamAccel}）\n" +
                  $"平均合成加速度(頂部): {avgCompositeAccel:F3} m/s²\n" +
                  $"最大速度            : {maxSpeed:F3} m/s\n" +
                  $"最大角速度          : {maxAngularVel:F1} deg/s\n" +
                  $"最大ジャーク        : {maxJerkMeasured:F2} m/s³\n" +
                  $"Fixed Timestep      : {fixedTimestep:F4} s", this);

        try
        {
            string path = Path.Combine(Application.dataPath, "..", "RunData.csv"); // プロジェクト直下へ
            // Excelで日本語が化けないようBOM付きUTF-8で保存
            File.WriteAllLines(path, csvRows.ToArray(), new System.Text.UTF8Encoding(true));
            Debug.Log($"時系列CSVを書き出しました: {Path.GetFullPath(path)}", this);
        }
        catch (System.Exception e)   // 書き込み失敗（権限など）はエラー表示だけして続行
        {
            Debug.LogError($"CSV書き出し失敗: {e.Message}", this);
        }
    }
}
