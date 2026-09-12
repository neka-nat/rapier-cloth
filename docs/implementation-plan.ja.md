# rapier-cloth 実装プラン

作成日: 2026-09-13。計画作成時は未実装。[進捗と実測](progress.md)を別途記録する。根拠となる構成は[設計書](package-design.ja.md)を参照する。本書のPR名は予定する変更単位であり、GitHub上に作成済みのPRではない。

**初版の完成条件**

`rapier-cloth-core` と `rapier-cloth` を実装し、既存のRapier worldへ布を追加して、固定環境上の布をkinematicなグリッパーで掴み、持ち上げ、運び、離すRust実行例を完成させる。f32/f64の数値試験、記録データを確認するviewer、独立した利用側からのパッケージ検証までを0.1候補の作業に含める。

| 項目 | 初版で実装する範囲 |
|---|---|
| 計算 | 3D、CPU逐次実行、XPBD、固定トポロジー |
| 精度 | f32を既定、f64を排他的featureで選択 |
| Rapier | 0.34系列。アプリが剛体worldと時間進行を所有 |
| 布 | 三角形meshと矩形grid、面密度、辺長拘束、二面角の曲げ、固定点 |
| 外部衝突 | 固定の球・箱・カプセル・半空間。球・箱・カプセルのkinematic移動 |
| 把持 | 頂点または複数頂点を剛体の局所anchorへ拘束し、途中で解除 |
| 摩擦 | 接触点の相対速度に対する動摩擦、反発係数0 |
| 連成 | 一方向。布が剛体の動きへ追従する |
| 出力 | 頂点・三角形・法線、step診断、headless実行記録、再生viewer |
| 配布準備 | 2クレートのパッケージ検証、利用例、対応表、変更履歴 |

自己衝突、折り畳み保証、任意TriMesh/compound、動的剛体への反作用、GPU、Python/npm bindingsは後続のPR4以降で扱う。把持は明示的なattachmentで実現する。指の接触圧力・静止摩擦だけで保持する物理モデルは初版の受入範囲に含めない。

計画作成時のcheckoutにはREADMEと設計書のみがあり、Rust本体は未作成だった。Rust/Cargoはローカルで1.93.0、botrailのlockfileは `rapier3d-f64 0.34.0` を使用していた。これらは今回読み取ったローカル状態であり、本計画によるbuild成功を示すものではない。

**実装順序**

| 順序 | 変更単位 | 依存 | レビュー可能な成果物 |
|---|---|---|---|
| 1 | PR0: workspaceと接続実験 | なし | 精度を選択してRapierの型を利用する最小ライブラリ・利用側試験 |
| 2 | PR1a: メッシュと状態 | PR0 | 入力検証、面積に基づく質量、安定したhandle |
| 3 | PR1b: XPBDと布単体 | PR1a | 垂れ下がる布、拘束の数値試験、診断出力 |
| 4 | PR2a: 固定形状との接触 | PR1b | 複数Colliderとの片側接触・侵入復旧 |
| 5 | PR2b: sweep・filter・速度補正 | PR2a | 静的環境の高速移動試験、衝突filter、失敗時の契約 |
| 6 | PR3a: kinematicと把持 | PR2b | 移動する局所anchorへの追従と解除 |
| 7 | PR3b: 摩擦と布操作例 | PR3a | `pick_and_place` の自動受入試験・記録 |
| 8 | PR3c: viewerと性能計測 | PR3b | 記録の再生・数値表示、誤差を併記したbenchmark |
| 9 | R0: 初版の配布確認 | PR0〜PR3c | 展開したcrateを独立consumerから利用できる0.1候補 |
| 後続 | PR4: 自己衝突 | R0 | 頂点–面・辺–辺、重なり・折り畳みの受入試験 |
| 後続 | PR5: 双方向連成 | PR4を基本とする | 自由剛体への反作用と連成誤差の検証 |
| 後続 | PR6: 性能・利用先拡張 | 必要な物理機能の完成 | 並列化、実アプリ統合、必要なbindings |

PR0〜PR3cは、先行する完了条件を満たして順に進める。複雑な段階は表のa/b/cの単位でレビューできるようにする。R0のローカル配布確認と、実際のregistry公開は別の作業として記録する。

**PR0 — workspaceとRapier接続実験**

変更先: `Cargo.toml`、`Cargo.lock`、`rust-toolchain.toml`、`.gitignore`、`src/lib.rs`、`crates/rapier-cloth-core/{Cargo.toml,src/lib.rs,src/math.rs}`、`tests/rapier_compat.rs`、`tests/consumers/{f32,f64}/`、`scripts/check-features.sh`、`.github/workflows/ci.yml`。

1. rootを `rapier-cloth` package兼workspaceにし、coreをmemberにする。別consumerはworkspaceから除外する。
2. rootはoptionalな依存として `rapier-f32`（package `rapier3d`）と `rapier-f64`（package `rapier3d-f64`）を持つ。両方ともversion requirementは `0.34`、実際に検証する解決結果はlockfileに残す。
3. root/coreとも `default = ["f32"]` とし、rootからcoreへ選択精度を転送する。rootのcore依存は `path` と同一リリースの `version` を併記し、`default-features = false` にする。両精度有効・両精度無効をコンパイルエラーにする。
4. coreの `Real` と `Vec3` をf32/f64、glamの対応型で定義する。rootは選択されたRapierを `rapier_cloth::rapier` として再公開する。coreはRapier/Parryへ依存しない。
5. Rapier step後のquery、AABB候補検索、粒子球とプリミティブのcontact/sweep、局所anchorのworld変換を実験する。反作用APIは自由剛体の単独試験で型と回転方向を確認するだけに留める。
6. f32/f64の独立consumerが直接依存するRapierのbody/shapeを入口の型と交換できることを確認する。botrail自体への変更はここでは不要。

`rust-toolchain.toml` はまず手元にある1.93.0に固定する。最小対応RustはRapierの宣言に合わせ1.86を候補とし、依存全体と実装がそのtoolchainで通ることを別jobで確認してから宣言する。デフォルトのRapier並列featureは追加しない。

完了条件: f32/f64の別buildとconsumer試験が通る。core単体の依存にRapier/Parry/描画が入らない。無効なfeature組合せは想定した説明を伴って失敗する。0.34で必要なAPIを使用できない場合は、この段階で原因と対応versionを設計へ反映してからPR1aへ進む。

**PR1a — メッシュ、質量、状態管理**

変更先: coreの `mesh.rs`、`material.rs`、`cloth.rs`、`surface.rs`、`error.rs`、`tests/mesh_contract.rs`。

1. rest頂点と三角形を受け取る `ClothMesh`、`GridBuilder`、材料値を検証する `ClothMaterial` を実装する。
2. 有限値・添字・ゼロ面積・重複面・非多様体辺・不整合な面の向き・孤立頂点を拒否する。境界辺は許可する。weldや面の反転を黙って実施しない。
3. 三角形面積×面密度を各頂点に配分し、質量と拘束時の有効逆質量を分ける。
4. 位置、前位置、速度、逆質量を配列で保持する。共有辺、曲げ用の隣接三角形、rest長・rest角を事前計算する。
5. 世代付き `ClothHandle` と、布内の安定した頂点IDを実装する。削除時には拘束・attachment側が失効を検知できるようにする。
6. `SurfaceView` は読み取りsliceとして公開する。描画頂点と物理頂点の対応を明示し、0.1では原則1対1とする。

完了条件: 1m角・面密度0.2kg/m²の総質量が0.2kgになる。gridの頂点・面・辺数が計算上の個数と一致する。不正入力を拒否し、handle再利用で別の布へ誤接続しない。mesh生成順序と拘束列挙順序が同一build内で再現可能である。

**PR1b — XPBDと布単体の数値検証**

変更先: coreの `constraints/{distance,bend,target}.rs`、`solver.rs`、`diagnostics.rs`、`tests/solver_reference.rs`、`tests/hanging_cloth.rs`、`examples/hanging_cloth.rs`。

1. 外力による予測、XPBD投影、速度再構成を1substepの処理として実装する。coreにもRapierにも隠れた外部時間刻みを作らない。
2. `alpha_tilde = compliance / h²` とし、lambdaはsubstep開始時に0、反復中は累積とする。固定点は分母が0になる拘束を安全に処理する。
3. 辺長拘束、二面角の曲げ、world固定点・移動targetを実装する。gridの対角辺を含めるが、独立した材料せん断モデルとは扱わない。
4. 曲げの勾配を有限差分と比較する。非常に短い辺・退化する面・角度の符号切替を専用fixtureで確認する。
5. 速度減衰は時間に基づく係数とし、例えば `exp(-damping * h)` を使う。大きなhや非有限値は入力で拒否する。
6. step用作業配列を再利用し、正常に完了した状態だけをcommitする。内部計算の非有限値を検知した場合は直前のcloth状態を維持する。
7. 最大/95 percentileの伸び、拘束誤差、退化件数、有限値チェック結果を `StepReport` に出す。

完了条件: 自由落下が採用した離散積分の解析式と一致し、hを小さくした比較で連続解へ収束する。単一距離拘束の解とhard pinの誤差を確認する。1m角の垂れ下がりfixtureが10,000 substepで有限値を維持し、最後の1,000 substepで辺伸びの95 percentileが5%以下となる。

この段階で静止形状の見た目だけを合格条件にしない。時間刻み・反復数・解像度を変えた誤差を測り、材料complianceの値と試験設定を記録する。

**PR2a — 固定Rapier形状との接触**

変更先: `src/{world,scene,collision,diagnostics}.rs`、coreの `contact.rs` とsolverの外部拘束境界、`tests/static_contacts.rs`、`examples/drape_static.rs`。

1. `RapierClothWorld` を利用者向け入口として実装する。内部にcloth状態、solver作業領域、外部接触の対応表を持ち、Rapier worldは所有しない。
2. 更新済み `QueryPipeline` を借りる `RapierScene` を作る。重力、world識別、substep番号、hを明示する。
3. 予測したcloth位置を包むAABBから候補を取得する。初めは布単位で候補を作り、性能計測後にパッチ単位へ分割できるようにする。
4. 各候補のParry形状へ粒子球のcontact queryを行い、world法線・接触点・分離距離をcoreの接触形式へ変換する。全sceneの最初の1接触だけで処理を打ち切らない。
5. 法線拘束は片側拘束としてlambdaを非負に制限する。内部拘束の投影後も接触を再評価し、接触キーとlambdaの対応を反復内で維持する。
6. 接触補正によって布が既存候補AABBを出た場合は候補を取り直す。接触予算超過は診断付きの失敗とし、候補を黙って間引かない。
7. 静的接触の速度を再構成し、初期侵入の復旧が不自然な跳ね返りを作らないよう安定化補正を分離する。

完了条件: 球・箱・カプセル・半空間への接触と、床/壁の角の同時接触が通る。接触していない粒子に引力が生じない。球中心など法線が不定になり得る初期配置を処理し、失敗する場合は明確な診断を返す。対応fixtureの収束後の最大侵入量が接触半径の20%以下となる。

**PR2b — 静的sweep、filter、step失敗時の契約**

変更先: `src/collision.rs`、`src/scene.rs`、`src/world.rs`、`tests/{static_sweep,query_lifecycle,step_contract}.rs`、`docs/integration.ja.md`。

1. 旧位置と予測位置を包むswept AABBを使い、静的形状に対する粒子球のsweepを追加する。TOIをhのどの単位で扱うかを統一し、開始時の侵入を通常のsweepと分ける。
2. 複数形状に当たる場合は候補ごとに処理し、必要な追加接触を解く。TOI後の残り移動の扱いと反発係数0を固定する。
3. sensorと無効Colliderを除外し、groups・個別除外・利用者predicateを評価する。cloth固有のfilterとして文書化する。
4. 初版で未対応のdynamic body・形状が接触候補になったら `UnsupportedCollision` を返す。利用者が明示的に除外したものは報告付きで無視できる。sceneに存在するだけの無関係な形状はstep失敗の理由にしない。
5. Collider追加・削除・直接移動の後にRapierをstepしてからqueryを作る経路を試験する。queryの再生成だけでBVH同期が保証されるとは扱わない。
6. 同じsubstepの二重実行、hの不一致、異なるworldキー、古いcloth handleを拒否する。Rapierの生の参照だけからBVHの鮮度や実際のworld同一性を完全検証するAPIはないため、利用側の同期契約と、入口が確認できるtokenの検証範囲を区別する。

完了条件: 薄い固定箱を高速粒子が横切るfixtureでsweepの効果を検証する。ignore/filterの各組合せを試験する。失敗時にcloth状態が部分更新されず、失敗理由とsubstep番号が残る。

Rapier先行step後のcloth失敗では、Rapierは既に `t+h`、clothは `t` のままとなる。アプリは両者の不一致を検出して進行を止める。hを小さくして再試行するなら、利用側が両worldを同じ時刻へ復元する必要がある。clothだけを進め直す自動retryや、成功した同じsubstepの再実行は提供しない。この制約を実行例と統合ガイドに記載する。

**PR3a — kinematic同期とattachment**

変更先: `src/{scene,attachment,world}.rs`、coreの `constraints/target.rs`、`tests/{kinematic_motion,attachments}.rs`、`examples/moving_anchor.rs`。

1. Rapier step前の姿勢とstep後の姿勢をsceneに渡す。標準順序を「kinematic目標設定 → Rapier step(h) → cloth step(h)」に固定する。
2. 固定形状のうち有限サイズの球・箱・カプセルへkinematic対応を追加する。並進量と回転による表面移動の保守的な上限を計算し、外部substepあたりの移動予算を監視する。
3. 初期設定は表面移動上限を対象の最小接触半径の0.5倍以下とする。これは品質管理用の制限でありCCD保証ではない。予算超過は必要な分割数の目安を含むエラーにする。予算判定の対象は選択されたkinematic形状全体とし、終端AABBから外れた高速横断を見落とさない。
4. `AttachmentHandle`、単一頂点・複数頂点のattachment、局所anchor、complianceを実装する。目標位置はbodyの終端姿勢から計算する。
5. hard attachmentの速度は離散的なanchor移動と整合させ、soft attachmentは投影後位置から速度を再構成する。body表面の摩擦速度は接触点での並進・角速度から求める。
6. attachment解除時は直前に得た粒子速度を維持する。剛体または布の削除ではattachmentを失効させ、解除/失効のイベントを出す。
7. 把持粒子と把持Colliderの接触除外はその組合せだけに適用する。他の粒子や環境との衝突を無効にしない。

完了条件: 並進・回転する剛体の局所anchorに追従し、解除後の速度が予想値と一致する。異なるbodyへの再attachment、body削除とhandle再利用、移動量予算超過を試験する。大きな回転や高速横断を完全な相対CCDで処理できるとは宣言しない。

**PR3b — 動摩擦とpick-and-place**

変更先: coreの `contact.rs`、`src/collision.rs`、`tests/{friction,pick_and_place}.rs`、`tests/fixtures/`、`examples/{pick_and_place,support/recording}.rs`、`docs/recording-format.ja.md`。

1. 法線impulseと整合するCoulomb上限内で、相対接線速度を減らす動摩擦を実装する。布とColliderの係数合成は初版では算術平均とし、明示的なoverrideを許す。
2. 残りの接線速度を超えて逆方向へ加速しないよう補正量を制限する。数値的な侵入復旧の補正を摩擦の根拠となる物理impulseへ混ぜない。
3. 摩擦0の滑り、水平面の減速、移動/回転する接触面に対する速度を試験する。静止摩擦の接触履歴はここでは追加しない。
4. 初期化 → 把持 → 持ち上げ → 水平移動 → 解放 → 落下/静止の決められた軌道を実装する。まず単一グリッパーの複数頂点attachmentを使う。
5. `--record` と `--summary` を実行例へ用意し、シミュレーション設定、トポロジー、各時刻の布位置・剛体姿勢、attachment状態、診断を保存する。出力は実行ごとのディレクトリへ書く。
6. 記録のschema versionと実行設定を保存する。serde関連依存は当面examples/testsのdev依存に限定し、core全状態の公開シリアライズAPIは作らない。

完了条件: headlessな `pick_and_place` をf32/f64で実行し、把持対象の高さ・移動先・解除時刻・解除速度と全期間の有限値を自動検証する。摩擦の減速は既知の単純系と比較する。記録を再読込して頂点数・面数・時刻・最終位置が元の実行と一致する。

**PR3c — viewerと性能計測**

変更先: `demos/viewer/{package.json,package-lock.json,index.html,src/}`、`benches/cloth_scaling.rs`、`docs/{examples,benchmarks}.ja.md`、viewer用CI job。

初版のviewerは、Rustのheadless実行が保存した記録をThree.jsで再生する小さなWebアプリとする。Rustライブラリへの描画依存を増やさず、数値結果と見た目を同じ記録で照合できる。ブラウザ内のcloth計算やライブ把持操作は後続のbindings設計に分ける。

1. 再生/停止、時刻移動、カメラ操作、wireframe、固定点/attachment表示、伸び/侵入量の表示を実装する。
2. 頂点更新には `BufferGeometry` と `BufferAttribute` を使う。頂点更新に応じて法線・boundsを更新し、時刻を前後に移動しても表示が一致するようにする。[Three.js BufferGeometry](https://threejs.org/docs/pages/BufferGeometry.html)、[BufferAttribute](https://threejs.org/docs/pages/BufferAttribute.html)
3. f64の記録は計算時の精度を保持して保存し、GPU表示時だけf32へ変換する。数値の合否判定には表示用頂点を使わない。UIには記録再生であることを示す。
4. Node依存はこのdemo内に閉じ、実装時に選定した解決結果をlockfileへ固定する。デモのWeb公開やnpm公開は不要。
5. `cloth_scaling` benchmarkは32×32、64×64、128×128頂点の固定fixtureを使用する。core時間、形状query時間、全cloth時間を分け、CSV/JSONで出力する。
6. 100 substepのwarmup後に1,000 substepを計測し、p50/p95、最大侵入、伸び、接触数、作業領域容量を保存する。実行中の状態が変わるため、各サイズ・精度で同じ初期条件から開始する。

完了条件: 実際のRust記録を読み込み、先頭・中間・末尾frameの頂点と剛体姿勢を描画データと照合する。再生と時刻移動をブラウザで操作し、console errorがない。benchmarkにCPU、OS、Rust、commit、精度、h、反復数が含まれる。性能目標のms値は初回計測と利用アプリの時間予算から定める。

**R0 — 初版の配布確認**

変更先: root/coreのpackage metadata、README、`CHANGELOG.md`、`docs/integration.ja.md`、`docs/compatibility.md`、`scripts/check-packages.sh`、独立consumer。

1. APIの利用例を実際のpublic entrypointでコンパイルする。rootからの再公開、同精度のRapier handle/shapeの受渡し、描画用sliceの借用期間を確認する。
2. f32/f64、MSRV、対象OSのCIを完了させ、実際の完了状態と失敗ログを確認する。ローカルLinuxの成功をWindows/macOSの成功と扱わない。
3. 相互依存するcoreと入口を `cargo package --workspace` でまとめて梱包・検証し、各 `.crate` の内容を確認する。f64は `--no-default-features --features rapier-cloth/f64,rapier-cloth-core/f64` を付けて別途検証する。さらに両crateを一時ディレクトリへ展開し、consumer側の一時的なpatch設定で接続してf32/f64のbuild・実行を検証する。path依存にはversionが必要で、梱包後はpathが取り除かれる。[Cargoのpackage仕様](https://doc.rust-lang.org/cargo/commands/cargo-package.html)
4. 一時consumerではworkspaceのソースpathへ依存させない。公開後のregistryからの取得試験は、実際に公開する段階の別チェックとする。
5. READMEにインストール方法、最小例、Rapier/精度の対応表、対応形状、一方向連成、自己衝突の未対応を明記する。運用詳細はdocsへリンクする。

完了条件: PR0〜PR3cの数値・操作試験が合格し、梱包したソースから利用側がbuild・実行できる。デモ画像/記録とその生成条件が揃う。未完了のプラットフォームや公開作業は結果に明示する。

**テスト設定と基準**

標準の布fixtureは、1m角、32×32頂点、面密度0.2kg/m²、重力加速度9.81m/s²、接触半径0.005m、外部tick 1/60秒、4substep、各8反復とする。試験ごとのcompliance・減衰・摩擦・固定点・姿勢・warmup期間をfixtureへ保存する。誤差目標は未実装時点の受入条件であり、達成済みの値ではない。

| 対象 | 独立した検証方法・合格目標 |
|---|---|
| 質量 | 面積×面密度。grid細分化で総質量が変わらない |
| 積分 | 自由落下の離散式と比較し、hを半分にして連続解への収束を確認 |
| 拘束 | 1拘束の直接計算、曲げ勾配の有限差分、平行移動/回転した入力の比較 |
| hard pin | 1mスケールでf32は1e-5m以下、f64は1e-9m以下の誤差 |
| 垂れ下がり | 10,000 substepで有限値。終盤の辺伸びp95が5%以下 |
| 静的接触 | 矛盾しないfixtureで、収束後の最大侵入が接触半径の20%以下 |
| sweep | 薄い固定箱を横切る粒子を捕捉。sweepを切った比較fixtureとの差も確認 |
| kinematic | 移動/回転anchorの終点、解除速度、移動予算超過を確認 |
| 摩擦 | 相対接線速度を逆転させない。摩擦0と既知の水平面減速を比較 |
| 再現性 | 同じbuild・CPU・設定で、拘束順序と実行結果が再現する |
| メッシュ依存 | 16×16、32×32、64×64で同じ密度と設定を比較し、誤差を記録 |
| 配布 | 同じworkspaceではなく、展開済みcrateを使う独立consumerで実行 |

接触不可能な配置（例えばhard pinを床内部へ置く）は、精度試験とは別に矛盾する入力として試験する。合格目標を満たさない場合は最初に拘束/衝突/時間同期の原因を調べ、設定を変えるときはfixtureと記録に理由を残す。

**確認コマンドとCI**

以下は実装後に実行する予定のコマンドで、今回実行したものではない。各PRでは該当するtargetの試験を先に実行し、その後に共通確認を実施する。

```bash
cargo fmt --all -- --check
cargo test --locked -p rapier-cloth-core
cargo test --locked -p rapier-cloth
cargo test --locked -p rapier-cloth-core --no-default-features --features f64
cargo test --locked -p rapier-cloth --no-default-features --features f64
cargo clippy --locked -p rapier-cloth-core -p rapier-cloth --all-targets -- -D warnings
cargo clippy --locked -p rapier-cloth-core --no-default-features --features f64 --all-targets -- -D warnings
cargo clippy --locked -p rapier-cloth --no-default-features --features f64 --all-targets -- -D warnings
bash scripts/check-features.sh
cargo test --locked --manifest-path tests/consumers/f32/Cargo.toml
cargo test --locked --manifest-path tests/consumers/f64/Cargo.toml
```

consumerのlockfileもPR0で生成・保存する。`check-features.sh` は無効featureの失敗理由を検証し、別原因のbuild失敗を成功として扱わない。`--all-features` は排他的精度のため使用しない。

```bash
cargo run --locked -p rapier-cloth --release --example hanging_cloth
cargo run --locked -p rapier-cloth --release --example drape_static
cargo run --locked -p rapier-cloth --release --no-default-features --features f64 --example pick_and_place -- --record target/recordings/pick-f64.json --summary target/recordings/pick-f64-summary.json
cargo bench --locked -p rapier-cloth --bench cloth_scaling
cargo bench --locked -p rapier-cloth --no-default-features --features f64 --bench cloth_scaling
npm --prefix demos/viewer ci
npm --prefix demos/viewer run build
npm --prefix demos/viewer run test
bash scripts/check-packages.sh
```

初期CIはLinuxでfmt、両精度のtest/clippy、feature拒否、consumerを実行する。Windows/macOSとMSRVのjobをPR0から設定し、0.1候補で全ての対応宣言に必要なjobを完了させる。viewerはPR3cから別jobにする。性能の絶対値は共有CI runnerで合否判定せず、固定された計測環境の結果を残す。

**後続の実装範囲**

| 変更単位 | 変更先 | 追加する内容 | 完了条件 |
|---|---|---|---|
| PR4a | coreの `self_collision/{broad_phase,narrow_phase}.rs` | 変形meshの候補検索、頂点–面と辺–辺、隣接除外、2枚の布 | 小さな総当たり判定との候補/接触比較、両面・非隣接交差の試験 |
| PR4b | coreの自己接触拘束、CCD、折り畳み例 | 高速交差、初期交差の扱い、接触予算 | 折り畳みfixtureで侵入・交差・誤差を測る。初期交差を処理できない場合は診断して拒否 |
| PR5a | `src/coupling.rs`、剛体proxy、反作用buffer | 有効質量・逆慣性・拘束Jacobianに基づく反作用、反作用の一度だけの適用 | 自由箱の並進/回転、二重適用拒否、孤立系の運動量誤差 |
| PR5b | 接地/関節/attachmentを含む連成tests | 質量比、h依存、Rapier先行stepによる反作用の遅れ | 接地・関節を含む試験結果を別々に記録。必要なら連成driverを再設計 |
| PR6a | coreの拘束並列化、benches | グラフ彩色などによる並列計算 | 逐次基準との誤差・性能比較、競合の検証 |
| PR6b | botrail adapterまたは別統合crate | `BodyId` 対応、外部substepの挿入、実利用scene | botrailの実entrypointで既存剛体/関節動作とcloth操作が共存 |
| PR6c | 必要に応じたbindings | WASM/JS、Python、GPU | 対象環境でworld連携とartifactを検証してから対応を宣言 |

後続の双方向連成では、内部の伸び・曲げ補正を反作用へ混ぜない。接触点の回転寄与、固定自由度、摩擦、attachmentを含め、剛体proxyの反復中更新とRapierへの適用時刻を検証する。孤立系の総運動量の正規化誤差1%以下を初期目標とし、静止して総運動量が0に近い場合にも意味のある分母をfixtureで定義する。

**着手時に行うこと**

最初の実装単位はPR0とする。既存のREADME・設計書・本プランを保持し、2クレートとprecision CIを作成して、Rapier 0.34の実際のquery/contact型を通す。PR0の互換性実験が完了したら、その結果に合わせて依存とAPI境界を確定し、PR1aのメッシュ実装へ進む。
