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
