# 性能計測

```bash
cargo bench --locked --bench cloth_scaling -- --output target/benchmarks/run-01
cargo bench --locked --bench cloth_scaling --no-default-features --features f64 -- --output target/benchmarks/run-01
```

32×32、64×64、128×128頂点、1m角、面密度0.2、半径0.005m、h=1/240秒、8反復。固定半空間上に水平に置いた同じ初期条件を精度・サイズごとに再生成する。材料は `ClothMaterial::default()`、重力は(0,-9.81,0)、pinなし。100substepのwarmup後に1,000substepを測定する。

JSONはCPU、OS、Rust、commit、dirty状態、精度、条件と測定結果を保持し、CSVは比較しやすい行形式を保存する。出力先を変えて過去の実測を保持する。

| 計測項目 | 範囲 |
|---|---|
| core | solver呼出時間から接触callback時間を差し引いた時間 |
| query | 候補AABB作成・列挙、Parry contact/sweep、接触データ変換 |
| total | cloth worldの入口処理、kinematic確認、全clothのstaging、coreとquery |
| scratch_array_bytes | coreの保持配列の容量。現行版はcontact state配列を含む。bridgeのstaging、Rapierのメモリは含まない |
| 誤差 | 計測区間内の最大侵入、最大伸び、各stepのp95伸びの最大値 |

Rapierの剛体step、記録JSON生成、描画はtotalに含まない。p50/p95は1,000個のsubstep時間の分位点であり、異なる条件の数字を足して得た値ではない。直列CPU版の測定で、128×128がリアルタイム要件を満たすという宣言ではない。絶対時間のCI合否判定は設けない。

実測値と保存先は[検証記録](progress.md)へ追記する。coreとqueryのどちらが時間を占めるかを確認し、必要なアプリの予算を決めてからpatch単位の候補生成・拘束並列化を検討する。

## 初回の実測（最適化前）

測定日: 2026-09-13。Intel Core i9-13900H、Linux 6.8.0-139-generic、Rust 1.93.0。commit `53afdb2bf5f5ad6220c80bff79a1c135112dc312`、両精度ともcleanなcheckoutで開始し、逐次実行した。通常のデスクトップ環境での計測であり、専用の隔離されたベンチマークマシンではない。

| 精度 | grid | core p50 | query p50 | total p50 | total p95 |
|---|---|---:|---:|---:|---:|
| f32 | 32×32 | 3.65ms | 1.48ms | 5.14ms | 5.37ms |
| f32 | 64×64 | 15.45ms | 6.44ms | 21.92ms | 25.08ms |
| f32 | 128×128 | 64.44ms | 27.49ms | 92.03ms | 99.43ms |
| f64 | 32×32 | 4.04ms | 1.70ms | 5.74ms | 6.69ms |
| f64 | 64×64 | 16.63ms | 7.32ms | 24.01ms | 25.54ms |
| f64 | 128×128 | 69.31ms | 30.83ms | 100.37ms | 117.03ms |

平面上の静止fixtureでは計測区間の最大伸び・侵入が0、接触数は頂点数と一致した。変形中の誤差は別の垂れ下がり・pick-and-place試験で確認している。

1/240秒の予算は1substepあたり約4.17msであり、このCPUの初版逐次版は32×32でも超えた。初版の標準条件でリアルタイム運用するには、要求解像度・時間刻みの設計と性能改善が必要。0.1候補は数値参照と操作機能の基盤として評価する。

生データ: [f32 JSON](evidence/cloth-scaling-f32.json) / [f64 JSON](evidence/cloth-scaling-f64.json) / [f32 CSV](evidence/cloth-scaling-f32.csv) / [f64 CSV](evidence/cloth-scaling-f64.csv)。各ファイルに誤差、接触数、配列容量と生成条件を含める。

## 32×32・1枚・CPU・60fpsへの改善

目標は1frameにつきh=1/240秒を4回、8反復、標準f32。曲げ拘束の解析勾配を簡潔な式にし、通常の角度差から不要な三角関数を除いた。接触状態とstatic sweepの保持は、木構造での検索・挿入から整列済み配列の線形マージへ変更。最大伸びとp95伸びは全ソートを使わず同じ定義で計算し、最終曲げ診断では勾配を生成しない。

拘束の順序、反復数、接触の毎反復更新、static sweep、全clothの失敗時atomicity、kinematic移動量制限は維持している。並列化、SIMD専用型、fast-math、LTO設定の変更は行っていない。

```bash
cargo bench --locked --bench realtime -- --output target/realtime/run-01
cargo bench --locked --bench realtime --no-default-features --features f64 -- --output target/realtime/run-01
```

`benches/realtime.rs` は32×32頂点、1m角、標準材料、100substepのwarmup後に1,000substepを実行する。4substepを実際に続けて実行した250frameの時間を取り、Rapier step、snapshot、cloth worldのstaging、接触、拘束、診断を含める。描画、GPU転送、法線更新、JSON出力、sleepは含めない。substepのp95を4倍した値ではない。

| fixture | 条件 |
|---|---|
| flat-halfspace | y=0.005mに水平な布、固定半空間、全頂点が接触 |
| hanging | y=1mで最初の32頂点をpin、外部colliderなし、重力で変形 |
| moving-sphere | y=0.56mから固定床と半径0.3mのkinematic球へdrape。球中心は(0.5+0.08sin(1.5t), 0.25, 0.5)、最大並進速度0.12m/s |

変更前はbenchmarkだけを追加した `a603f02`、変更後は `5d9ac1e`。同じIntel Core i9-13900H、Linux 6.8.0-139-generic、Rust 1.93.0の通常release設定で比較する。両方のソースはclean。変更前・変更後を精度ごとにそれぞれ3回実行し、実行中に別のbuild/test/benchmarkは並走させていない。通常のデスクトップ環境で、CPU固定・専用マシンによる測定ではない。生データの各runにcommitと条件を記録する。

| 精度 | 場面 | 改善前 p95中央値 | 改善後 p95中央値 | 改善後 p95の範囲 | 改善後16.67ms超過 |
|---|---|---:|---:|---:|---:|
| f32 | 静止接触 | 22.91ms | 11.98ms | 11.74–15.49ms | 2/750frame |
| f32 | 垂れ下がり | 16.69ms | 11.46ms | 9.38–12.10ms | 1/750frame |
| f32 | 動く球 | 24.29ms | 13.13ms | 12.77–14.64ms | 0/750frame |
| f64 | 静止接触 | 24.87ms | 13.98ms | 13.30–14.50ms | 2/750frame |
| f64 | 垂れ下がり | 20.33ms | 10.71ms | 10.02–12.69ms | 1/750frame |
| f64 | 動く球 | 25.46ms | 17.57ms | 15.70–19.47ms | 142/750frame |

p95中央値は各runの250frameから求めたp95を3個並べた中央値。範囲はその3個の最小〜最大であり、全frameを混ぜたp95ではない。優先対象のf32は全場面・全runでp95が16.67ms以内。f64の動く球は3回中2回でp95が予算を超えており、この条件での60fps達成とは扱わない。f32でも単発frameの最大は17.16ms未満で、常に期限を満たすhard realtimeの保証ではない。

誤差は変更前後で近い値を保った。f32の計測区間全体の最大p95伸びは、hangingが5.7686%→5.7681%、moving-sphereが3.8606%→3.8605%。hangingの過渡状態を含む最大の単一edge伸びは約23.48%で、最終1,000stepだけを評価する長時間試験とは別指標。moving-sphereの最大侵入は約1.6e-7m以下。f64でも同様の伸びを記録し、最大侵入は約2.5e-16m以下だった。

生データ: [変更前 f32 run 1](evidence/realtime/before/run-1/realtime-f32.json) / [変更後 f32 run 1](evidence/realtime/after/run-1/realtime-f32.json) / [変更前 f64 run 1](evidence/realtime/before/run-1/realtime-f64.json) / [変更後 f64 run 1](evidence/realtime/after/run-1/realtime-f64.json)。同じ配置の `run-2`、`run-3` に全測定を保存。[benchmark実行ファイルのSHA256](evidence/realtime/binaries.json)も保存している。

この結果は3つの32×32 fixtureのCPU物理処理を対象とし、描画込みのアプリの60fpsを保証するものではない。OSのスケジューリング等で予算を超えたframeはJSONの `frames_over_budget` と最大時間に残す。64×64以上、複数cloth、大量のcollider、ブラウザ、自己衝突を含む性能は今回の達成範囲に含めない。実アプリではreleaseビルド、固定hと4substep、描画時間の計測を組み合わせ、遅延が累積しないようにframe全体の予算を確認する。

数値検証では両精度で、旧曲げ勾配との2,000ケース比較、有限差分、角度のbranch cut、旧接触map実装とのlambda/位置の一致、sweep平面の保持・優先・予算制限を確認。10,000substepの垂れ下がりは既存の最終1,000step p95伸び5%以下を満たした。並べ替えによる接触処理順は維持するが、曲げ式の浮動小数点丸めが変わるため旧版とのbit単位の軌跡一致は約束しない。同一build内のcheckpoint replayと記録のroundtrip試験は通過している。

`scratch_array_bytes` は今回から2本のcontact state配列も数える。以前のBTreeMapノードは旧指標に含まれていなかったため、旧値との増減をそのまま総メモリの増減として比較できない。bridgeのstagingとsweep作業配列、Rapier側のメモリは引き続き含めない。
