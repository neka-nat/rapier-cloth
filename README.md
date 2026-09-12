# rapier-cloth

既存のRapier worldへ追加できる、Rust製のCPU XPBD clothシミュレータです。
`rapier-cloth-core` が布の計算、`rapier-cloth` がRapierとの接触・把持・時間同期を担当します。

0.1.0候補。crates.ioへの公開はまだ行っていません。checkoutをpath依存として利用できます。

```toml
[dependencies]
rapier-cloth = { path = "../rapier-cloth" }
```

f64の場合は `default-features = false, features = ["f64"]` を指定します。

| 精度 | 同じworldで利用するRapier | Rust |
|---|---|---|
| f32（既定） | `rapier3d 0.34` | 1.90以上 |
| f64 | `rapier3d-f64 0.34` | 1.90以上 |

```rust
use rapier_cloth::{rapier::prelude::*, prelude::*};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = WorldId::new();
    let mut rigid = PhysicsWorld::new();
    let h = 1.0 / 240.0;
    rigid.integration_parameters.dt = h;
    let mut cloths = RapierClothWorld::new(id);
    let cloth = Cloth::new(
        GridBuilder::new(32, 32).origin(Vec3::Y).build()?,
        ClothMaterial::default(),
    )?;
    let handle = cloths.add_cloth(cloth);
    
    // 外部substepごとに、Rapier → clothの順で同じhを一度だけ進める。
    let before = SceneSnapshot::capture(id, 0, &rigid.bodies, &rigid.colliders);
    // kinematic目標を設定する場合はここ。
    rigid.step();
    let query = rigid.broad_phase.as_query_pipeline(
        rigid.narrow_phase.query_dispatcher(),
        &rigid.bodies, &rigid.colliders, QueryFilter::default(),
    );
    cloths.step_substep(h, &RapierScene::new(query, &before, h, rigid.gravity))?;
    let surface = cloths.cloth(handle)?.surface();
    assert_eq!(surface.positions.len(), 1024);
    Ok(())
}
```

固定トポロジーの三角形mesh、面密度、伸び・二面角の曲げ、固定点、局所anchorによる複数頂点の把持を扱います。対応する外部形状は球・箱・カプセル・固定の半空間です。有限形状のkinematic移動にはsubstepごとの移動量制限があります。

連成は一方向です。自己衝突、布の辺・面のCCD、動的剛体への反作用、任意TriMesh、静止摩擦だけによる把持は未対応です。初版の把持は明示的なattachmentを使います。

```bash
cargo run --release --example pick_and_place -- --record target/run-01/cloth.json --summary target/run-01/summary.json
npm --prefix demos/viewer ci
npm --prefix demos/viewer run dev
```

viewerはRustの記録を再生します。「記録を開く」で生成したJSONを選べます。

- [時間同期・接触・把持・失敗からの復元](docs/integration.ja.md)
- [実行例と記録形式](docs/examples.ja.md) / [性能計測](docs/benchmarks.ja.md)
- [対応表](docs/compatibility.md) / [検証記録](docs/progress.md) / [変更履歴](CHANGELOG.md)
- [設計検討](docs/package-design.ja.md) / [実装プラン](docs/implementation-plan.ja.md)

MIT OR Apache-2.0。
