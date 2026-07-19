using UnityEngine;
using System.Collections.Generic;
using System.IO;

/// <summary>
/// ロボット掃除機（2輪差動駆動）の制御。
/// ・駆動は左右2輪の位置への前後力で表現（AddForceAtPosition）。
/// ・前進/後退＝左右同力、超信地旋回＝左右逆力（中心まわり）。
/// ・カメラ頂部（床1.0m）の水平合成加速度が常に1.0m/s²以下になるよう制限・実測。
/// ・回転X/Zは固定（転倒防止＆カメラを鉛直軸上に保つ）想定。
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RobotController : MonoBehaviour
{
    public enum CommandType { Forward, Backward, SpinLeft, SpinRight }

    [System.Serializable]
    public struct Command
    {
        public CommandType type;
        public float value; // Forward/Backward=距離(m), Spin=角度(度)
        public Command(CommandType t, float v) { type = t; value = v; }
    }

    [Header("車体諸元（課題の必須条件に合わせる）")]
    public float wheelHalfTrack = 0.15f;   // 車体中心から各駆動輪までの距離(m)
    public float camHeight = 1.0f;         // カメラ頂部の高さ(m)
    public float comHeight = 0.5f;         // 重心高さ(m)
    public bool setCenterOfMass = true;    // 重心高さをRigidbodyに設定するか

    [Header("加速度制約（必須条件）")]
    public float maxCamAccel = 1.0f;       // カメラ頂部の水平合成加速度 上限(m/s²)
    public float accelSafety = 0.9f;       // 安全マージン（上限に対する使用割合）
    public float maxJerk = 3.0f;           // 加速度指令の変化率上限(m/s³)（ジャーク抑制）

    [Header("直進制御")]
    public float speedKp = 4.0f;
    public float positionTolerance = 0.005f; // 到達判定(m)
    public float stopSpeed = 0.02f;          // 停止とみなす速度(m/s)
    public float brakeSafety = 0.35f;        // 減速プロファイル安全率（小=早めに減速＝突っ込み防止）

    [Header("旋回制御（超信地旋回：中心まわり）")]
    public float spinAccelMax = 200f;      // 角加速度上限(deg/s²)
    public float maxYawRate = 150f;        // 角速度上限(deg/s)
    public float spinOmegaKp = 10f;        // 旋回：角速度追従ゲイン
    public float headingKp = 20f;          // 方位保持P
    public float headingKd = 9f;           // 方位保持D
    public float maxSpinTorque = 5f;       // 旋回トルクの安全上限(N·m)
    public float angleTolerance = 1.0f;    // 角度到達判定(度)

    [Header("横滑り抑制（非ホロノミック拘束）")]
    public float lateralGrip = 20f;        // 横速度を消す強さ

    [Header("状態表示（読み取り専用）")]
    public int currentCommandIndex = 0;
    public string currentPhase = "待機";
    public bool timing = false;            // 計測中（スタート通過〜ゴール通過）
    public float elapsedTime = 0f;         // 走破時間（スタートライン〜ゴールライン）

    [Header("計測（レポート用・読み取り専用）")]
    public float fixedTimestep = 0f;       // Fixed Timestep(s)
    public float maxCompositeAccel = 0f;   // カメラ頂部 水平合成加速度の最大(m/s²)
    public float avgCompositeAccel = 0f;   // 走破中の平均(m/s²)
    public float maxSpeed = 0f;            // 最大速度(m/s)
    public float maxAngularVel = 0f;       // 最大角速度(deg/s)
    public float maxJerkMeasured = 0f;     // 最大ジャーク(m/s³)

    [Header("デバッグ")]
    public bool verboseLog = false;
    public float csvInterval = 0.02f;      // CSV記録間隔(s)。大きいほど行数が減る

    private Rigidbody rb;
    private readonly List<Command> commands = new List<Command>();
    private Vector3 segmentStartPos;
    private float segmentStartAngle;
    private bool finished = false;

    // 制御・計測の内部状態
    private float accelCmdPrev = 0f;
    private float maxLinAccel;             // = maxCamAccel * accelSafety
    private float yawInertia = 0.1f;       // ヨー慣性(kg·m²)（Startで実測）
    private Vector3 camVelPrev;
    private Vector3 camAccelPrev;
    private int measureWarmup = 3;
    private float logTimer = 0f;
    private float csvTimer = 0f;
    private float simTime = 0f;            // Play開始からの経過(s)
    private double accelSum = 0.0;         // 平均計算用（走破中）
    private long accelCount = 0;
    private bool summaryWritten = false;
    private readonly List<string> csvRows = new List<string>();

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        if (rb.isKinematic)
            Debug.LogError("Rigidbody が kinematic です。物理駆動できません。Is Kinematic を外してください。", this);

        if (setCenterOfMass)
            rb.centerOfMass = new Vector3(0f, comHeight, 0f); // ※transform原点が床レベル前提

        maxLinAccel = maxCamAccel * accelSafety;
        yawInertia = rb.inertiaTensor.y;
        if (yawInertia <= 1e-4f || float.IsNaN(yawInertia)) yawInertia = 0.1f;

        BuildCommands();
        segmentStartPos = rb.worldCenterOfMass;
        segmentStartAngle = transform.eulerAngles.y;
        camVelPrev = CamTopVelocity();

        fixedTimestep = Time.fixedDeltaTime;
        csvRows.Add("simTime_s,runTime_s,timing,phase,aCam_mps2,speed_mps,yawRate_dps,jerk_mps3");

        Debug.Log($"[初期化] mass={rb.mass} CoM(local)={rb.centerOfMass} " +
                  $"constraints={rb.constraints} maxLinAccel={maxLinAccel:F2} " +
                  $"FixedTimestep={fixedTimestep:F4}s", this);
    }

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

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        float forwardForce = 0f;   // 前進方向の合力(N)
        float yawTorque = 0f;      // ヨー方向の合トルク(N·m, +yまわり)

        if (finished || currentCommandIndex >= commands.Count)
        {
            if (!finished) { finished = true; currentPhase = "完了"; }
            forwardForce = SpeedToForce(0f);
            yawTorque = HoldTorque(segmentStartAngle);
        }
        else
        {
            Command cmd = commands[currentCommandIndex];
            switch (cmd.type)
            {
                case CommandType.Forward:
                    DriveStraight(cmd.value, +1f, out forwardForce, out yawTorque);
                    break;
                case CommandType.Backward:
                    DriveStraight(cmd.value, -1f, out forwardForce, out yawTorque);
                    break;
                case CommandType.SpinLeft:
                    Spin(+1f, cmd.value, out forwardForce, out yawTorque);
                    break;
                case CommandType.SpinRight:
                    Spin(-1f, cmd.value, out forwardForce, out yawTorque);
                    break;
            }
        }

        ApplyWheelForces(forwardForce, yawTorque);
        ApplyLateralFriction();

        simTime += dt;
        if (timing) elapsedTime += dt;

        Measure(dt);

        if (finished && !summaryWritten) WriteSummary();
    }

    // スタート/ゴールラインのトリガーで計測を開始・終了
    void OnTriggerEnter(Collider other)
    {
        if (!timing && !summaryWritten && other.name == "StartLine")
        {
            timing = true;
            Debug.Log($"▶ 計測開始（スタートライン通過）", this);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (timing && other.name == "GoalLine")
        {
            timing = false;
            Debug.Log($"■ 計測終了（ゴールライン通過） 走破時間={elapsedTime:F3}s", this);
        }
    }

    // 壁接触の検知（規則：壁接触＝無効）
    void OnCollisionEnter(Collision c)
    {
        if (c.gameObject.name.StartsWith("Wall"))
            Debug.LogWarning($"⚠ 壁に接触（{c.gameObject.name}）→ このランは無効です", this);
    }

    // --- 直進（減速距離から目標速度を決める時間最適プロファイル）---
    void DriveStraight(float distance, float dir, out float forwardForce, out float yawTorque)
    {
        currentPhase = (dir > 0 ? "前進 " : "後退 ") + currentCommandIndex;

        Vector3 fwd = transform.forward;
        float traveled = Vector3.Dot(rb.worldCenterOfMass - segmentStartPos, fwd) * dir;
        float remaining = distance - traveled;

        if (remaining <= positionTolerance)
        {
            forwardForce = SpeedToForce(0f);
            yawTorque = HoldTorque(segmentStartAngle);
            if (Mathf.Abs(CurrentForwardSpeed()) < stopSpeed)
                NextCommand();
            return;
        }

        // 減速に必要な距離 = v²/(2a) より、残り距離から到達可能な目標速度を決める
        // 安全率で減速計画を弱め（早め減速）、実際の減速力(maxLinAccel)に余裕を残す＝突っ込み防止
        float brakeAccel = maxLinAccel * brakeSafety;
        float vBrake = Mathf.Sqrt(2f * brakeAccel * remaining);
        float targetSpeed = vBrake * dir;

        forwardForce = SpeedToForce(targetSpeed);
        yawTorque = HoldTorque(segmentStartAngle); // まっすぐ保つ
    }

    // --- 超信地旋回（中心まわり回転：カメラ頂部の水平加速度は原理的にほぼ0）---
    // 目標方位へのPD制御。DeltaAngleは常に近道(±180)を返すので、通り越しても戻り、暴走しない。
    void Spin(float sign, float angleDeg, out float forwardForce, out float yawTorque)
    {
        currentPhase = "旋回 " + currentCommandIndex;

        forwardForce = SpeedToForce(0f); // 前進成分は消して純粋なその場旋回に

        float targetAngle = segmentStartAngle + sign * angleDeg;
        float err = Mathf.DeltaAngle(transform.eulerAngles.y, targetAngle);

        yawTorque = HoldTorque(targetAngle);

        if (Mathf.Abs(err) <= angleTolerance && Mathf.Abs(CurrentYawRateDeg()) < 5f)
            NextCommand();
    }

    // 目標前進速度 → 必要な前進力（加速度上限・ジャーク上限を尊重）
    float SpeedToForce(float targetSpeed)
    {
        float speedErr = targetSpeed - CurrentForwardSpeed();
        float accelTarget = Mathf.Clamp(speedKp * speedErr, -maxLinAccel, maxLinAccel);
        // ジャーク制限：加速度指令の変化率を制限
        float accelCmd = Mathf.MoveTowards(accelCmdPrev, accelTarget, maxJerk * Time.fixedDeltaTime);
        accelCmdPrev = accelCmd;
        return rb.mass * accelCmd;
    }

    // 目標方位へ向けPD制御（直進中の蛇行防止・旋回後の保持）。慣性ベースでトルク化。
    float HoldTorque(float targetAngleDeg)
    {
        float err = Mathf.DeltaAngle(transform.eulerAngles.y, targetAngleDeg); // deg
        float angAccel = Mathf.Clamp(headingKp * err - headingKd * CurrentYawRateDeg(),
                                     -spinAccelMax, spinAccelMax);              // deg/s²
        return Mathf.Clamp(yawInertia * angAccel * Mathf.Deg2Rad, -maxSpinTorque, maxSpinTorque);
    }

    // 前進力＋ヨートルクを「左右2輪の駆動力」に分解して物理適用
    void ApplyWheelForces(float forwardForce, float yawTorque)
    {
        // 制約:  fL + fR = forwardForce
        //        正味ヨートルク(+y) = halfTrack·(fL - fR) = yawTorque
        float fL = forwardForce * 0.5f + yawTorque / (2f * wheelHalfTrack);
        float fR = forwardForce * 0.5f - yawTorque / (2f * wheelHalfTrack);

        Vector3 fwd = transform.forward;
        Vector3 leftWheel = transform.TransformPoint(new Vector3(-wheelHalfTrack, 0f, 0f));
        Vector3 rightWheel = transform.TransformPoint(new Vector3(+wheelHalfTrack, 0f, 0f));

        rb.AddForceAtPosition(fwd * fL, leftWheel, ForceMode.Force);
        rb.AddForceAtPosition(fwd * fR, rightWheel, ForceMode.Force);
    }

    // 横滑り抑制：横速度を消す向きの摩擦力（加速度予算を超えないようクランプ）
    void ApplyLateralFriction()
    {
        Vector3 right = transform.right;
        float lateralSpeed = Vector3.Dot(rb.linearVelocity, right);
        float lateralAccel = Mathf.Clamp(-lateralSpeed * lateralGrip, -maxLinAccel, maxLinAccel);
        rb.AddForce(right * rb.mass * lateralAccel, ForceMode.Force);
    }

    void NextCommand()
    {
        currentCommandIndex++;
        segmentStartPos = rb.worldCenterOfMass;
        segmentStartAngle = transform.eulerAngles.y;
        accelCmdPrev = 0f;
    }

    float CurrentForwardSpeed() => Vector3.Dot(rb.linearVelocity, transform.forward);
    float CurrentYawRateDeg() => rb.angularVelocity.y * Mathf.Rad2Deg;

    Vector3 CamTopWorldPos() => rb.worldCenterOfMass + Vector3.up * (camHeight - comHeight);
    Vector3 CamTopVelocity() => rb.GetPointVelocity(CamTopWorldPos());

    // レポート用メトリクスの実測（カメラ頂部の水平合成加速度・速度・角速度・ジャーク）
    void Measure(float dt)
    {
        Vector3 camVel = CamTopVelocity();
        Vector3 camAccel = (camVel - camVelPrev) / dt;
        camAccel.y = 0f; // 水平成分のみ（重力は含めない）
        float aMag = camAccel.magnitude;
        float jerk = ((camAccel - camAccelPrev) / dt).magnitude;

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

            if (aMag > maxCamAccel + 1e-3f)
                Debug.LogWarning($"⚠ カメラ頂部の水平加速度が上限超過: {aMag:F3} m/s²（上限{maxCamAccel}）", this);

            // 平均は走破中（計測中）のみ
            if (timing)
            {
                accelSum += aMag;
                accelCount++;
                avgCompositeAccel = (float)(accelSum / accelCount);
            }

            // 時系列CSV（csvInterval秒ごとに間引いて記録。timing列で走破区間を判別可能）
            csvTimer += dt;
            if (!summaryWritten && csvTimer >= csvInterval)
            {
                csvTimer -= csvInterval;
                csvRows.Add($"{simTime:F3},{elapsedTime:F3},{(timing ? 1 : 0)},{currentPhase}," +
                            $"{aMag:F4},{rb.linearVelocity.magnitude:F4},{CurrentYawRateDeg():F3},{jerk:F3}");
            }
        }

        camVelPrev = camVel;
        camAccelPrev = camAccel;

        if (verboseLog)
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

    // 走行終了時：サマリ表示＋時系列CSV書き出し（1回だけ）
    void WriteSummary()
    {
        summaryWritten = true;

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
            string path = Path.Combine(Application.dataPath, "..", "RunData.csv");
            // Excelで日本語が化けないようBOM付きUTF-8で保存
            File.WriteAllLines(path, csvRows.ToArray(), new System.Text.UTF8Encoding(true));
            Debug.Log($"時系列CSVを書き出しました: {Path.GetFullPath(path)}", this);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"CSV書き出し失敗: {e.Message}", this);
        }
    }
}
