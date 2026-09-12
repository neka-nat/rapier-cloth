# 記録形式 v1

`examples/support/recording.rs` の `Recording` と `Summary` がschemaの型定義。serdeはexamples/tests/benchのdev依存で、ライブラリの通常依存に含まれない。記録は閲覧・解析用で、solver再開用のcheckpointではない。

単位はm、s、kg。座標系はRapier worldと同じで、実行例ではY上向き。回転はquaternionの `[x,y,z,w]`。f64計算はJSONまでf64のまま保存し、serde_jsonの `float_roundtrip` を有効化して再読込の丸めを確認する。viewerは元の数値を保持し、GPUに渡す頂点だけFloat32Arrayへ変換する。

| フィールド | 内容 |
|---|---|
| schema_version | 現在は1。未知のversionはviewerで拒否 |
| precision | f32 / f64 |
| config | grid、h、反復数、材料、操作のstep番号、記録間隔 |
| triangles | 全frame共通の0始まり頂点添字3個の列 |
| shapes | body idごとのbox半寸法、body局所位置、表示色 |
| frames[].step / time | 外部substep番号 / 経過秒 |
| frames[].positions | 物理頂点順のworld座標 |
| frames[].bodies | 安定した記録内id、world位置、回転 |
| frames[].attached_particles / anchors | 把持頂点と対応するworld anchor |
| frames[].pinned_particles | 固定頂点 |
| frames[].diagnostics | 最大/p95伸び、最大侵入、target誤差、接触数 |

`phase` は `settle`、`lift`、`transport`、`release`、`drop`。把持はattach_stepの終了状態で次のsubstepを開始するときに作る。解放はrelease_stepの終了状態で実行し、そのframeではattachmentは空になる。解放直前の粒子速度はsummaryに保存する。

summaryには解放時刻・速度、持ち上げた高さ、運搬終点、最終重心・最高点、全期間の最大誤差を保存する。運搬距離は初期化中の微小な変形を含む把持時点の実位置を基準に評価する。`finite: true` は全substepが有限値検査を通って完了した場合にのみ出力される。失敗した実行例は非0終了し、成功summaryを生成しない。

現行viewerのshape schemaはboxのみ。これは記録実行例の表示形状の範囲で、ライブラリの衝突形状の範囲とは異なる。viewerは添字、有限値、時刻順序、body対応を検証してから表示を置き換える。
