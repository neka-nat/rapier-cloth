# 実行例とviewer

物理計算と描画を同時に動かす場合は[ライブデモ](live-demo.ja.md)を使う。`npm --prefix demos/viewer run live` でローカルCPUサーバーと操作画面を起動できる。以下は従来のheadless実行例と記録再生の手順。

```bash
cargo run --release --example hanging_cloth
cargo run --release --example drape_static
cargo run --release --example moving_anchor
cargo run --release --example pick_and_place -- --record target/run-01/f32.json --summary target/run-01/f32-summary.json
cargo run --release --no-default-features --features f64 --example pick_and_place -- --record target/run-01/f64.json --summary target/run-01/f64-summary.json
```

出力先は実行ごとに新しいディレクトリを使う。既存ファイルへの書込みは拒否する。エラー時には両worldの時刻がずれるため、実行例は停止する。[復元の契約](integration.ja.md)を参照。

`pick_and_place` の条件は `examples/support/pick_fixture.json`。16×16頂点・30cm角、h=1/240秒・8反復。0.5秒で把持、2秒までに30cm持ち上げ、4秒までに40cm運び、0.2m/sで動いている状態から解放する。6.5秒まで落下を計算する。4substepごとに記録し、先頭を含め391frameを保存する。

単一グリッパーに一辺の16頂点を明示的にattachmentする例で、指先の静止摩擦による把持ではない。自己衝突がないため、自由部分が自分自身と交差しない保証はない。検証は把持位置、移動距離、解除速度、最終高さ、侵入量、有限値とJSON再読込を対象にする。変形誤差はsummaryへ別途記録する。

```bash
npm --prefix demos/viewer ci
npm --prefix demos/viewer run dev
```

表示されたlocalhost URLを開く。同梱のf64記録が表示される。「記録を開く」で自分のJSONを読み込む。再生・停止、時刻slider、マウスによるカメラ操作、wireframe、固定点/把持点表示に対応する。ライブシミュレーションではない。

```bash
npm --prefix demos/viewer run build
npm --prefix demos/viewer exec -- playwright install chromium
npm --prefix demos/viewer run test
```

既存Chromeを使う場合は `CHROME_PATH=/usr/bin/google-chrome npm --prefix demos/viewer run test`。テストは実際の記録とGPU用頂点・法線・bounds・body姿勢・anchorを照合し、再生・seek・カメラ・file inputを操作する。スクリーンショットは `demos/viewer/test-results/` に保存する。

位置更新時の法線とboundsは [Three.js BufferGeometry](https://threejs.org/docs/pages/BufferGeometry.html) のAPIで再計算する。ブラウザ試験用サーバーは [Playwright webServer](https://playwright.dev/docs/test-webserver) で起動する。

[JSON schema](recording-format.ja.md) / [検証とサンプルの生成元](progress.md)
