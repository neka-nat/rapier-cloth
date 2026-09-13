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
| scratch_array_bytes | coreの保持配列の容量。BTreeMapノード、bridgeのstaging、Rapierのメモリは含まない |
| 誤差 | 計測区間内の最大侵入、最大伸び、各stepのp95伸びの最大値 |

Rapierの剛体step、記録JSON生成、描画はtotalに含まない。p50/p95は1,000個のsubstep時間の分位点であり、異なる条件の数字を足して得た値ではない。直列CPU版の測定で、128×128がリアルタイム要件を満たすという宣言ではない。絶対時間のCI合否判定は設けない。

実測値と保存先は[検証記録](progress.md)へ追記する。coreとqueryのどちらが時間を占めるかを確認し、必要なアプリの予算を決めてからpatch単位の候補生成・拘束並列化を検討する。

## 初回の実測

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

1/240秒の予算は1substepあたり約4.17msであり、このCPUの現行逐次版は32×32でも超える。現状の標準条件でリアルタイム運用するには、要求解像度・時間刻みの設計と性能改善が必要。0.1候補は数値参照と操作機能の基盤として評価する。

生データ: [f32 JSON](evidence/cloth-scaling-f32.json) / [f64 JSON](evidence/cloth-scaling-f64.json) / [f32 CSV](evidence/cloth-scaling-f32.csv) / [f64 CSV](evidence/cloth-scaling-f64.csv)。各ファイルに誤差、接触数、配列容量と生成条件を含める。
