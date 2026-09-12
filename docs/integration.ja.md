# Rapierとの統合

`rapier_cloth::rapier` は選択した精度のRapierを再公開する。利用側が同じ0.34系列・精度を直接依存している場合、RigidBodyHandle、ColliderHandle、SharedShape、Poseをそのまま交換できる。`Real` は双方のpreludeに含まれるため、両方をglob importする場合は `use rapier_cloth::Real;` を明示する。

## 時間の所有権

アプリがRapier worldと外部substepを所有する。例えば外部tick 1/60秒に対して、Rapierとclothをそれぞれ1/240秒で4回進める。cloth側の隠れた時間分割はない。

1. 同じRapier worldに対して一つの `WorldId` を保持する。
2. `SceneSnapshot::capture(id, step, &bodies, &colliders)` で開始時の姿勢を保存する。stepは0からの連番。
3. kinematic bodyへ次の姿勢を設定する。
4. `integration_parameters.dt = h` でRapierを一度stepする。
5. 更新済みbroad phaseからqueryを作り、`RapierScene::new(query, &before, h, gravity)` を借りる。
6. `RapierClothWorld::step_substep(h, &scene)` を一度だけ呼ぶ。

入口はWorldId、step番号、hの一致を確認する。WorldIdは利用側の宣言であり、別のRapier worldの参照へ同じtokenを付けた誤用や、実際にRapierを何回stepしたかを検出するものではない。Colliderの追加・削除・直接移動後はRapierをstepしてBVHを更新する。queryを作り直すだけでは同期されない。

`SurfaceView` の位置と三角形はclothに借用される。描画へのコピーが終わったら借用を終え、次のstepを呼ぶ。物理頂点と表示頂点は1対1。法線は `surface.write_normals(&mut normals)` で取得できる。

## 接触とfilter

静的な球・箱・カプセル・半空間、およびkinematicな球・箱・カプセルに対応する。各粒子を `contact_radius` の球として扱う。全頂点のAABBで候補を集め、拘束投影後も候補と接触を再評価する。候補となった未対応形状やdynamic bodyは `UnsupportedCollision` になる。遠くに存在するだけの未対応形状は通常の接触候補にはならない。

`QueryFilter` のgroups・predicateと `CollisionSettings::excluded_colliders` を利用する。sensor・無効Colliderも除外する。これはcloth queryのfilterであり、Rapierの剛体solver用contact hooksやsolver groupsを代理実行しない。`ignored_colliders` は列挙された候補のうちadapterで除外したもの。QueryFilterで既に除外されたものやBVHから除かれた無効Colliderは列挙できない。

kinematicの移動予算は、選択されたkinematic形状全体について `Colliderの並進距離 + (形状の外接半径 + body原点からの距離) × 回転角` を監視する。回転角には終端姿勢差に加えてRapierの角速度×hを考慮し、1回転による姿勢の一致を見落とさない。既定では最小粒子半径の0.5倍以下。高速横断を終端AABBだけで見落とさないため、布から離れたkinematic形状にも適用する。関係しないものは明示的にfilterする。超過時は `MotionBudget` の `required_substeps` を参照して外部時間分割を設計する。これは相対CCDの保証ではない。

静的sweepは開始位置→予測位置の変位を速度、TOIの上限を1としてParryへ渡す。ヒットした接平面をそのsubstep中の拘束として保持し、残りの移動は接線方向へ解く。反発係数は0。移動面には終端の離散接触を使う。開始時に存在した侵入の復旧は速度・摩擦から分離し、kinematic移動によって新たに生じた侵入は物理的な接触補正として扱う。

摩擦は接触点の相対速度へ作用する動摩擦。係数は布とColliderの算術平均で、Rapierの係数合成ルール設定は使わない。`friction_override` で全体を上書きできる。法線補正から得られたimpulseにCoulomb上限をかけ、残る滑りを超えて逆転させない。静止摩擦の履歴はない。

## 把持

`AttachmentDesc` にcloth handle、body handle、`AttachmentPoint { particle, local_anchor }` の列、complianceを指定し、`world.attach(desc, &bodies, &colliders)` を呼ぶ。anchorはColliderではなくbodyの局所座標。現在位置を掴む場合は `body.position().inverse_transform_point(particle_position)` で作る。

- compliance=0はhard target。速度は前substepの粒子位置から終端anchor位置への変位/h。
- compliance>0はXPBDの柔らかいtarget。位置補正と整合した速度を計算する。
- 一つの頂点への複数attachmentやpinとの競合は拒否する。
- `excluded_colliders` は指定bodyに属するColliderに限定され、除外するのはそのattachmentの頂点との組合せのみ。
- `release(handle)` は直前の粒子速度を保存する。解除・布削除・body削除・body無効化は `drain_attachment_events()` で取得する。body削除/無効化の通知は次の成功stepで確定する。
- handleはarena識別と世代を含む。削除されたbodyやattachmentのindexが再利用されても誤接続しない。

`examples/moving_anchor.rs` と `examples/support/recording.rs` が実際の利用例。

## 失敗と復元

coreは成功時だけ状態をcommitし、Rapier連携も複数の布をまとめてcommitする。接触数の予算はqueryごとの数とsubstep内で保持する接触キー数を制限し、超過時に黙って間引かない。退化拘束、非有限値、矛盾するhard targetもエラーになる。`next_step_index()` が失敗したsubstep番号を保持する。

数値処理中の失敗ではclothは時刻t、Rapierは既にt+hであり、`is_desynchronized()` がtrueになる。進行を止める。hを変更して再試行する場合は以下を行う。

1. step前に `cloth.checkpoint()` とアプリ側のRapier全状態を保存する。
2. エラー後、Rapierを保存した時刻へ復元する。body位置だけでなくCollider、接触、島、関節、BVHなども対象にする。
3. `cloth.restore(&checkpoint)` を呼び、アプリの時刻・step番号・操作入力も復元する。
4. 新しいhで両者を改めてstepする。復元後に使う外部handleはcheckpoint時点のものへ戻す。

checkpointはインメモリの同一cloth world用で、公開シリアライズ形式ではない。checkpoint後に生成したhandleやイベントは破棄する。復元の実例は `tests/checkpoint.rs` の固定環境fixtureを参照。一般のアプリでは入力制御器等の状態も保存する必要がある。

入口でのtoken/h不一致は数値計算を開始せずに拒否する。修正した入口情報で再度呼べるが、実際のRapier時間が正しいかはアプリが確認する。

## 数値モデルの限界

辺長と二面角のXPBD拘束を使う。complianceは解像度に依存しない連続体材料同定を保証しない。解像度、h、反復数と誤差を併記して評価する。辺の対角拘束は独立したせん断材料モデルではない。面の向きが不整合、非多様体辺、孤立頂点、ゼロ面積などの入力を黙って修復しない。

自己衝突、布同士の衝突、粒子間の辺・面と障害物の衝突、完全な相対CCD、任意mesh/compoundとの衝突、双方向連成は未実装。粗いmeshの隙間を細い障害物が通ることや、布が自分自身を横切ることはこの範囲では検知しない。
