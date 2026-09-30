//! Parametric garments built as flat-folded cloth meshes.
//!
//! A garment is one [`ClothMesh`]. Its panels lie coincident in the pattern
//! plane, which is their rest geometry: rest lengths and triangle shapes come
//! from the flat pattern of each panel. The vertices along a seam are shared by
//! the two panels the seam joins, and the back panel's triangles are wound the
//! other way, so the sewn surface is consistently oriented and every seam edge
//! has exactly two faces. Panel interiors therefore rest flat while seam hinges
//! rest fully folded, which is the shape of a garment lying on a table. Rest
//! geometry never places the layers apart; [`Garment::placed_positions`] does
//! that for the first step.
//!
//! Pattern frame: `x` runs across the garment (left is `-x`, the wearer's
//! right), `z` runs from the hem (`-z`) to the shoulder line (`+z`), and the
//! layers are separated along `y`, the front panel on the `+y` side. The
//! outline is aligned to a square grid, so every triangle is a right isosceles
//! triangle with legs of one cell; dimensions are rounded to whole cells.

use crate::{ClothError, ClothMesh, Real, Vec3};
use std::collections::BTreeMap;

/// Shape of the neck opening.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum NeckShape {
    /// A grid-aligned rectangular notch.
    #[default]
    Notch,
    /// A half ellipse as wide as the notch and as deep as each panel's neck
    /// depth: cells whose centre falls inside are removed and the vertices
    /// left inside the curve are moved out onto it, unless that would make a
    /// triangle thinner than three tenths of a cell.
    Round,
}

/// How the panels of a sewn garment are joined.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SeamJoin {
    /// Seam vertices are shared by both panels: one closed surface, seam
    /// hinges rest folded and can be stiffened (`seam_stiffness`).
    #[default]
    Shared,
    /// Each panel keeps its own seam vertices and stitches
    /// (`ClothMesh::stitches`) of length `stitch_length` join them: two open
    /// sheets threaded together, free to hinge at the seam, and a model for
    /// panels meshed separately.
    Stitched,
}

/// Which panels a pattern is cut into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GarmentLayers {
    /// One open panel: the outline as a single sheet, with the front neckline.
    Single,
    /// A front and a back panel sewn along the outline except the openings
    /// (neck, hem and cuffs).
    Sewn,
}

/// The panel a vertex belongs to. Seam vertices belong to both panels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layer {
    Front,
    Back,
    Seam,
}

/// Pattern regions of a T-shirt. The body includes its side seams.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Region {
    Body,
    LeftSleeve,
    RightSleeve,
}

/// A side of the pattern: left is `-x`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// Named pattern points. They lie on the front panel (or on a seam) except
/// [`Landmark::NeckBack`], which lies on the back panel of a sewn garment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Landmark {
    /// Where a side seam meets the hem.
    HemCorner(Side),
    HemCenter,
    /// Where a sleeve's top seam meets the cuff.
    CuffTop(Side),
    /// Where a sleeve's bottom seam meets the cuff.
    CuffBottom(Side),
    /// The middle of the cuff edge.
    CuffCenter(Side),
    /// Where a shoulder seam meets the neck opening.
    Shoulder(Side),
    /// Where a sleeve's bottom seam meets the body's side seam.
    Underarm(Side),
    /// The lowest point of the front neckline.
    NeckFront,
    /// The lowest point of the back neckline (sewn garments only).
    NeckBack,
    /// The centre of the front body panel.
    Chest,
}

/// Where a vertex sits in the pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VertexInfo {
    pub layer: Layer,
    pub region: Region,
    /// Grid cell corner `[column, row]`; column 0 is the body's left side and
    /// row 0 the hem.
    pub cell: [i32; 2],
}

/// Dimensions of a T-shirt pattern in metres.
///
/// The body is `body_width` wide and `body_length` from hem to shoulder line.
/// Each sleeve extends `sleeve_length` beyond the body side and is
/// `sleeve_width` deep below the shoulder line. The neck opening is a notch
/// `neck_width` wide, centred, cut `neck_depth_front` into the front panel and
/// `neck_depth_back` into the back panel. All lengths are rounded to whole
/// cells of `spacing`; every notch is at least one cell deep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TShirtPattern {
    pub body_width: Real,
    pub body_length: Real,
    pub sleeve_length: Real,
    pub sleeve_width: Real,
    pub neck_width: Real,
    pub neck_depth_front: Real,
    pub neck_depth_back: Real,
    pub spacing: Real,
    pub layers: GarmentLayers,
    pub seams: SeamJoin,
    /// Rest length of the stitches of a `SeamJoin::Stitched` garment: place
    /// the layers this far apart (at least the contact thickness).
    pub stitch_length: Real,
    pub neck: NeckShape,
    /// Membrane stiffness of the sleeves relative to the body (1 keeps one
    /// material); a per-region material for the implicit solver.
    pub sleeve_stiffness: Real,
    /// Bending stiffness of the seams relative to the panels: a stitched seam
    /// with its allowance is stiffer than the fabric (a few times), 1 keeps the
    /// seams as soft as the panels. Applied to the hinges across seam edges of
    /// a sewn garment.
    pub seam_stiffness: Real,
}

impl Default for TShirtPattern {
    /// A medium adult T-shirt on 2 cm cells, sewn.
    fn default() -> Self {
        Self {
            body_width: 0.50,
            body_length: 0.70,
            sleeve_length: 0.22,
            sleeve_width: 0.20,
            neck_width: 0.18,
            neck_depth_front: 0.08,
            neck_depth_back: 0.02,
            spacing: 0.02,
            layers: GarmentLayers::Sewn,
            seams: SeamJoin::Shared,
            stitch_length: 0.0015,
            neck: NeckShape::Notch,
            sleeve_stiffness: 1.0,
            seam_stiffness: 1.0,
        }
    }
}

/// A pattern resolved to grid cells.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TShirtCells {
    /// Body columns and rows.
    pub body: [usize; 2],
    /// Columns of one sleeve beyond the body, and sleeve rows.
    pub sleeve: [usize; 2],
    /// Neck columns, front notch rows, back notch rows.
    pub neck: [usize; 3],
    /// The first neck column.
    pub neck_start: usize,
    /// Whether the neck is a half ellipse rather than a notch.
    pub round_neck: bool,
}

impl TShirtCells {
    fn body_columns(&self) -> i32 {
        self.body[0] as i32
    }
    fn body_rows(&self) -> i32 {
        self.body[1] as i32
    }
    fn sleeve_columns(&self) -> i32 {
        self.sleeve[0] as i32
    }
    fn sleeve_rows(&self) -> i32 {
        self.sleeve[1] as i32
    }
    fn neck_columns(&self) -> i32 {
        self.neck[0] as i32
    }
    fn neck_start(&self) -> i32 {
        self.neck_start as i32
    }
    fn neck_rows(&self, layer: Layer) -> i32 {
        match layer {
            Layer::Back => self.neck[2] as i32,
            _ => self.neck[1] as i32,
        }
    }
    /// Whether the cell with lower-left corner `(i, j)` belongs to a panel.
    fn cell(&self, layer: Layer, i: i32, j: i32) -> bool {
        let (bx, by) = (self.body_columns(), self.body_rows());
        let (sx, sy) = (self.sleeve_columns(), self.sleeve_rows());
        if j < 0 || j >= by {
            return false;
        }
        let in_body = (0..bx).contains(&i);
        let in_sleeves = j >= by - sy && (-sx..bx + sx).contains(&i);
        if !(in_body || in_sleeves) {
            return false;
        }
        let n0 = self.neck_start();
        let in_neck = if self.round_neck {
            let (xi, eta) = self.neck_coordinates(layer, i as Real + 0.5, j as Real + 0.5);
            xi * xi + eta * eta < 1.0
        } else {
            (n0..n0 + self.neck_columns()).contains(&i) && j >= by - self.neck_rows(layer)
        };
        !in_neck
    }
    /// Grid coordinates relative to the neck's half ellipse: `xi` across it
    /// (±1 at the shoulder corners), `eta` down from the shoulder line (1 at
    /// the panel's neck depth).
    fn neck_coordinates(&self, layer: Layer, column: Real, row: Real) -> (Real, Real) {
        let half_width = self.neck_columns() as Real * 0.5;
        let centre = self.neck_start() as Real + half_width;
        let depth = self.neck_rows(layer) as Real;
        (
            (column - centre) / half_width,
            (self.body_rows() as Real - row) / depth,
        )
    }
    /// Whether the corner `(i, j)` belongs to a panel: a corner of some cell.
    fn corner(&self, layer: Layer, i: i32, j: i32) -> bool {
        self.cell(layer, i - 1, j - 1)
            || self.cell(layer, i, j - 1)
            || self.cell(layer, i - 1, j)
            || self.cell(layer, i, j)
    }
    /// Whether the corner lies on a panel's outline.
    fn boundary(&self, layer: Layer, i: i32, j: i32) -> bool {
        self.corner(layer, i, j)
            && !(self.cell(layer, i - 1, j - 1)
                && self.cell(layer, i, j - 1)
                && self.cell(layer, i - 1, j)
                && self.cell(layer, i, j))
    }
    /// Whether an outline corner lies on an opening rather than a seam. The
    /// corners where a seam ends belong to the seam.
    fn open(&self, layer: Layer, i: i32, j: i32) -> bool {
        let (bx, by) = (self.body_columns(), self.body_rows());
        let (sx, sy) = (self.sleeve_columns(), self.sleeve_rows());
        let hem = j == 0 && i > 0 && i < bx;
        let cuff = (i == -sx || i == bx + sx) && j > by - sy && j < by;
        let n0 = self.neck_start();
        let n1 = n0 + self.neck_columns();
        let neck = (n0..=n1).contains(&i)
            && j >= by - self.neck_rows(layer)
            && !(j == by && (i == n0 || i == n1));
        hem || cuff || neck
    }
    fn seam(&self, layer: Layer, i: i32, j: i32) -> bool {
        self.boundary(layer, i, j) && !self.open(layer, i, j)
    }
    fn region(&self, i: i32) -> Region {
        if i < 0 {
            Region::LeftSleeve
        } else if i > self.body_columns() {
            Region::RightSleeve
        } else {
            Region::Body
        }
    }
}

impl TShirtPattern {
    /// Resolves the dimensions to whole cells.
    pub fn cells(&self) -> Result<TShirtCells, ClothError> {
        let values = [
            self.body_width,
            self.body_length,
            self.sleeve_length,
            self.sleeve_width,
            self.neck_width,
            self.neck_depth_front,
            self.neck_depth_back,
            self.spacing,
        ];
        if values.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err(ClothError::InvalidParameter(
                "garment dimensions must be positive and finite",
            ));
        }
        if [
            self.seam_stiffness,
            self.sleeve_stiffness,
            self.stitch_length,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(ClothError::InvalidParameter(
                "seam and sleeve stiffness and stitch length must be finite and positive",
            ));
        }
        let count = |length: Real, minimum: usize| -> Result<usize, ClothError> {
            let cells = (length / self.spacing).round();
            if !cells.is_finite() || cells > 4096.0 {
                return Err(ClothError::InvalidParameter(
                    "garment spacing is too fine for its dimensions",
                ));
            }
            Ok((cells as usize).max(minimum))
        };
        let body = [count(self.body_width, 2)?, count(self.body_length, 2)?];
        let sleeve = [count(self.sleeve_length, 1)?, count(self.sleeve_width, 1)?];
        if sleeve[1] >= body[1] {
            return Err(ClothError::InvalidParameter(
                "sleeve width must be less than the body length",
            ));
        }
        let mut neck_columns = count(self.neck_width, 1)?;
        // Centre the notch on whole cells.
        if (body[0] - neck_columns.min(body[0])) % 2 == 1 {
            neck_columns += 1;
        }
        if neck_columns + 2 > body[0] {
            return Err(ClothError::InvalidParameter(
                "neck width must leave a shoulder seam of at least one cell on each side",
            ));
        }
        let neck_front = count(self.neck_depth_front, 1)?;
        let neck_back = count(self.neck_depth_back, 1)?;
        if neck_front.max(neck_back) >= body[1] {
            return Err(ClothError::InvalidParameter(
                "neck depth must be less than the body length",
            ));
        }
        Ok(TShirtCells {
            body,
            sleeve,
            neck: [neck_columns, neck_front, neck_back],
            neck_start: (body[0] - neck_columns) / 2,
            round_neck: self.neck == NeckShape::Round,
        })
    }

    /// Builds the garment mesh.
    pub fn build(&self) -> Result<Garment, ClothError> {
        let cells = self.cells()?;
        let s = self.spacing;
        let (bx, by) = (cells.body_columns(), cells.body_rows());
        let sx = cells.sleeve_columns();
        let half = Vec3::new(bx as Real * s * 0.5, 0.0, by as Real * s * 0.5);
        let rest = |i: i32, j: i32| Vec3::new(i as Real * s, 0.0, j as Real * s) - half;

        let mut positions = Vec::new();
        let mut triangles = Vec::new();
        let mut vertices = Vec::new();
        let mut front = BTreeMap::new();
        let mut back = BTreeMap::new();
        let sewn = self.layers == GarmentLayers::Sewn;
        let shared = sewn && self.seams == SeamJoin::Shared;
        let mut stitches = Vec::new();

        // Front panel corners in row-major order, then its triangles.
        for j in 0..=by {
            for i in -sx..=bx + sx {
                if !cells.corner(Layer::Front, i, j) {
                    continue;
                }
                let layer = if shared && cells.seam(Layer::Front, i, j) {
                    Layer::Seam
                } else {
                    Layer::Front
                };
                front.insert([i, j], positions.len() as u32);
                positions.push(rest(i, j));
                vertices.push(VertexInfo {
                    layer,
                    region: cells.region(i),
                    cell: [i, j],
                });
            }
        }
        for j in 0..by {
            for i in -sx..bx + sx {
                if !cells.cell(Layer::Front, i, j) {
                    continue;
                }
                let a = front[&[i, j]];
                let b = front[&[i + 1, j]];
                let c = front[&[i, j + 1]];
                let d = front[&[i + 1, j + 1]];
                triangles.push([a, c, b]);
                triangles.push([b, c, d]);
            }
        }
        if sewn {
            // The back panel shares the seam vertices and is wound the other
            // way. Its neckline may be shallower, so it may have corners the
            // front lacks.
            for j in 0..=by {
                for i in -sx..=bx + sx {
                    if !cells.corner(Layer::Back, i, j) {
                        continue;
                    }
                    if cells.seam(Layer::Back, i, j) {
                        let counterpart =
                            front.get(&[i, j]).copied().ok_or(ClothError::InvalidMesh(
                                "garment seam corner missing on the front panel".to_owned(),
                            ))?;
                        if shared {
                            if vertices[counterpart as usize].layer != Layer::Seam {
                                return Err(ClothError::InvalidMesh(
                                    "garment seams differ between the panels".to_owned(),
                                ));
                            }
                            back.insert([i, j], counterpart);
                            continue;
                        }
                        stitches.push(crate::mesh::Stitch {
                            vertices: [counterpart, positions.len() as u32],
                            rest_length: self.stitch_length,
                        });
                    }
                    back.insert([i, j], positions.len() as u32);
                    positions.push(rest(i, j));
                    vertices.push(VertexInfo {
                        layer: Layer::Back,
                        region: cells.region(i),
                        cell: [i, j],
                    });
                }
            }
            for j in 0..by {
                for i in -sx..bx + sx {
                    if !cells.cell(Layer::Back, i, j) {
                        continue;
                    }
                    let a = back[&[i, j]];
                    let b = back[&[i + 1, j]];
                    let c = back[&[i, j + 1]];
                    let d = back[&[i + 1, j + 1]];
                    triangles.push([a, b, c]);
                    triangles.push([b, d, c]);
                }
            }
        }
        if cells.round_neck {
            round_neckline(&cells, s, half, &mut positions, &triangles, &vertices);
        }
        let mut mesh = ClothMesh::new(positions, triangles)?;
        mesh.add_stitches(stitches)?;
        // Woven along the length: the warp runs from the hem to the shoulders
        // on both panels, so directional stiffness applies consistently.
        mesh.set_material_axes(Some(vec![Vec3::Z; mesh.triangles().len()]))?;
        if self.sleeve_stiffness != 1.0 {
            let regions = triangle_regions(mesh.triangles(), &vertices);
            let scale = self.sleeve_stiffness;
            mesh.scale_triangle_stiffness(|k, _| {
                if regions[k] == Region::Body {
                    1.0
                } else {
                    scale
                }
            })?;
        }
        if shared && self.seam_stiffness != 1.0 {
            let seam = |v: u32| vertices[v as usize].layer == Layer::Seam;
            let scale = self.seam_stiffness;
            mesh.scale_hinge_stiffness(|hinge| {
                if seam(hinge.vertices[0]) && seam(hinge.vertices[1]) {
                    scale
                } else {
                    1.0
                }
            })?;
        }
        Ok(Garment {
            mesh,
            pattern: *self,
            cells,
            vertices,
            front,
            back,
        })
    }
}

/// The region of each triangle: a sleeve when any corner lies beyond the
/// body's side columns, the body otherwise.
fn triangle_regions(triangles: &[[u32; 3]], vertices: &[VertexInfo]) -> Vec<Region> {
    triangles
        .iter()
        .map(|t| {
            t.iter()
                .map(|&v| vertices[v as usize].region)
                .find(|r| *r != Region::Body)
                .unwrap_or(Region::Body)
        })
        .collect()
}

/// Moves the vertices left inside a round neck's ellipse out onto the curve,
/// radially from the ellipse centre, then puts back any move that would leave
/// a triangle thinner than three tenths of a cell.
fn round_neckline(
    cells: &TShirtCells,
    spacing: Real,
    half: Vec3,
    positions: &mut [Vec3],
    triangles: &[[u32; 3]],
    vertices: &[VertexInfo],
) {
    let rest = |column: Real, row: Real| Vec3::new(column * spacing, 0.0, row * spacing) - half;
    let mut moved = std::collections::BTreeSet::new();
    for (v, info) in vertices.iter().enumerate() {
        let layer = match info.layer {
            Layer::Back => Layer::Back,
            _ => Layer::Front,
        };
        let [i, j] = info.cell;
        if info.layer == Layer::Seam || j >= cells.body_rows() {
            continue;
        }
        let (xi, eta) = cells.neck_coordinates(layer, i as Real, j as Real);
        let r2 = xi * xi + eta * eta;
        if r2 >= 1.0 - 1e-9 || r2 <= 0.0 {
            continue;
        }
        let scale = 1.0 / r2.sqrt();
        let half_width = cells.neck_columns() as Real * 0.5;
        let column = cells.neck_start() as Real + half_width + xi * scale * half_width;
        let row = cells.body_rows() as Real - eta * scale * cells.neck_rows(layer) as Real;
        positions[v] = rest(column, row);
        moved.insert(v);
    }
    // Quality guard, repeated until no triangle is thin.
    let minimum = 0.3 * spacing;
    for _ in 0..4 {
        let mut reverted = false;
        for t in triangles {
            let p = t.map(|i| positions[i as usize]);
            let area2 = (p[1] - p[0]).cross(p[2] - p[0]).length();
            let longest = [
                p[0].distance(p[1]),
                p[1].distance(p[2]),
                p[2].distance(p[0]),
            ]
            .into_iter()
            .fold(0.0, Real::max);
            if area2 / longest < minimum {
                for &v in t {
                    if moved.remove(&(v as usize)) {
                        let [i, j] = vertices[v as usize].cell;
                        positions[v as usize] = rest(i as Real, j as Real);
                        reverted = true;
                    }
                }
            }
        }
        if !reverted {
            break;
        }
    }
}

/// A built garment: the mesh plus the pattern metadata that locates panels,
/// seams and landmarks in it.
#[derive(Debug, Clone)]
pub struct Garment {
    mesh: ClothMesh,
    pattern: TShirtPattern,
    cells: TShirtCells,
    vertices: Vec<VertexInfo>,
    front: BTreeMap<[i32; 2], u32>,
    back: BTreeMap<[i32; 2], u32>,
}

impl Garment {
    pub fn mesh(&self) -> &ClothMesh {
        &self.mesh
    }
    pub fn into_mesh(self) -> ClothMesh {
        self.mesh
    }
    pub fn pattern(&self) -> &TShirtPattern {
        &self.pattern
    }
    pub fn cells(&self) -> TShirtCells {
        self.cells
    }
    /// The region of each triangle, indexed like the mesh triangles.
    pub fn triangle_regions(&self) -> Vec<Region> {
        triangle_regions(self.mesh.triangles(), &self.vertices)
    }
    /// Per-vertex pattern placement, indexed like the mesh vertices.
    pub fn vertices(&self) -> &[VertexInfo] {
        &self.vertices
    }
    /// The vertex at a grid corner of a panel, if the panel has that corner.
    /// Seam corners resolve from either panel; `Layer::Seam` accepts only
    /// corners that are seams.
    pub fn vertex(&self, layer: Layer, column: i32, row: i32) -> Option<u32> {
        let key = [column, row];
        match layer {
            Layer::Front => self.front.get(&key).copied(),
            Layer::Back => match self.pattern.layers {
                GarmentLayers::Sewn => self.back.get(&key).copied(),
                GarmentLayers::Single => None,
            },
            Layer::Seam => self
                .front
                .get(&key)
                .copied()
                .filter(|&v| self.vertices[v as usize].layer == Layer::Seam),
        }
    }
    /// The vertex of a landmark, if the pattern has it.
    pub fn landmark(&self, landmark: Landmark) -> Option<u32> {
        let c = &self.cells;
        let (bx, by) = (c.body_columns(), c.body_rows());
        let (sx, sy) = (c.sleeve_columns(), c.sleeve_rows());
        let cuff = |side: Side| match side {
            Side::Left => -sx,
            Side::Right => bx + sx,
        };
        let neck_centre = c.neck_start() + c.neck_columns() / 2;
        let (layer, i, j) = match landmark {
            Landmark::HemCorner(Side::Left) => (Layer::Front, 0, 0),
            Landmark::HemCorner(Side::Right) => (Layer::Front, bx, 0),
            Landmark::HemCenter => (Layer::Front, bx / 2, 0),
            Landmark::CuffTop(side) => (Layer::Front, cuff(side), by),
            Landmark::CuffBottom(side) => (Layer::Front, cuff(side), by - sy),
            Landmark::CuffCenter(side) => (Layer::Front, cuff(side), by - sy / 2),
            Landmark::Shoulder(Side::Left) => (Layer::Front, c.neck_start(), by),
            Landmark::Shoulder(Side::Right) => {
                (Layer::Front, c.neck_start() + c.neck_columns(), by)
            }
            Landmark::Underarm(Side::Left) => (Layer::Front, 0, by - sy),
            Landmark::Underarm(Side::Right) => (Layer::Front, bx, by - sy),
            Landmark::NeckFront => (Layer::Front, neck_centre, by - c.neck_rows(Layer::Front)),
            Landmark::NeckBack => (Layer::Back, neck_centre, by - c.neck_rows(Layer::Back)),
            Landmark::Chest => (Layer::Front, bx / 2, by / 2),
        };
        self.vertex(layer, i, j)
    }
    /// The vertices of every layer whose rest position lies within `radius`
    /// of a landmark: a pinch through all layers around that point, ordered by
    /// vertex index. Empty when the pattern lacks the landmark.
    pub fn patch(&self, landmark: Landmark, radius: Real) -> Vec<u32> {
        let Some(centre) = self.landmark(landmark) else {
            return Vec::new();
        };
        let rest = self.mesh.rest_positions();
        let centre = rest[centre as usize];
        (0..rest.len() as u32)
            .filter(|&v| rest[v as usize].distance(centre) <= radius)
            .collect()
    }
    /// The vertices of the front panel and the seams within `radius` of a
    /// landmark: what a gripper closing on the top layer of a garment lying
    /// front up would hold. Empty when the pattern lacks the landmark.
    pub fn top_patch(&self, landmark: Landmark, radius: Real) -> Vec<u32> {
        self.patch(landmark, radius)
            .into_iter()
            .filter(|&v| self.vertices[v as usize].layer != Layer::Back)
            .collect()
    }
    /// The stitches of a `SeamJoin::Stitched` garment.
    pub fn stitches(&self) -> &[crate::mesh::Stitch] {
        self.mesh.stitches()
    }
    /// The vertices shared by both panels (none when the seams are stitched).
    pub fn seam_vertices(&self) -> impl Iterator<Item = u32> + '_ {
        self.vertices
            .iter()
            .enumerate()
            .filter(|(_, v)| v.layer == Layer::Seam)
            .map(|(i, _)| i as u32)
    }
    /// Initial positions with the pattern centre at `origin` and the panels
    /// `layer_gap` apart along `y`: the front panel on the `+y` side, the back
    /// panel on the `-y` side and the seams midway. Two stacked layers need at
    /// least the contact thickness between them; thickness plus the activation
    /// margin starts them free of contact force. A single panel ignores the
    /// gap. Apply with [`crate::Cloth::set_positions`].
    pub fn placed_positions(&self, origin: Vec3, layer_gap: Real) -> Result<Vec<Vec3>, ClothError> {
        if !origin.is_finite() || !layer_gap.is_finite() || layer_gap < 0.0 {
            return Err(ClothError::InvalidParameter(
                "garment placement must be finite with a non-negative gap",
            ));
        }
        let gap = match self.pattern.layers {
            GarmentLayers::Single => 0.0,
            GarmentLayers::Sewn => layer_gap,
        };
        Ok(self
            .mesh
            .rest_positions()
            .iter()
            .zip(&self.vertices)
            .map(|(p, info)| {
                let lift = match info.layer {
                    Layer::Front => 0.5 * gap,
                    Layer::Back => -0.5 * gap,
                    Layer::Seam => 0.0,
                };
                *p + origin + Vec3::Y * lift
            })
            .collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{BTreeMap, BTreeSet};

    fn pi() -> Real {
        std::f64::consts::PI as Real
    }
    /// Rounding allowance for both precisions.
    fn tol() -> Real {
        1.0e3 * Real::EPSILON
    }

    /// Boundary edges (edges without a hinge) grouped into closed loops; every
    /// boundary vertex must have exactly two boundary edges.
    fn boundary_loops(mesh: &ClothMesh) -> usize {
        let hinged: BTreeSet<[u32; 2]> = mesh
            .hinges()
            .iter()
            .map(|h| {
                let mut e = [h.vertices[0], h.vertices[1]];
                e.sort();
                e
            })
            .collect();
        let mut adjacency: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for edge in mesh.edges() {
            let mut e = edge.vertices;
            e.sort();
            if hinged.contains(&e) {
                continue;
            }
            adjacency.entry(e[0]).or_default().push(e[1]);
            adjacency.entry(e[1]).or_default().push(e[0]);
        }
        for (v, n) in &adjacency {
            assert_eq!(
                n.len(),
                2,
                "boundary vertex {v} has {} boundary edges",
                n.len()
            );
        }
        let mut seen = BTreeSet::new();
        let mut loops = 0;
        for &start in adjacency.keys() {
            if !seen.insert(start) {
                continue;
            }
            loops += 1;
            let mut previous = start;
            let mut current = adjacency[&start][0];
            while current != start {
                seen.insert(current);
                let next = adjacency[&current]
                    .iter()
                    .copied()
                    .find(|&n| n != previous)
                    .unwrap();
                previous = current;
                current = next;
            }
        }
        loops
    }

    fn components(mesh: &ClothMesh) -> usize {
        let n = mesh.rest_positions().len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for edge in mesh.edges() {
            let a = root(&mut parent, edge.vertices[0] as usize);
            let b = root(&mut parent, edge.vertices[1] as usize);
            parent[a] = b;
        }
        (0..n).filter(|&i| root(&mut parent, i) == i).count()
    }

    fn pattern_area(cells: TShirtCells, spacing: Real, layer: Layer) -> Real {
        let body = cells.body[0] * cells.body[1];
        let sleeves = 2 * cells.sleeve[0] * cells.sleeve[1];
        let neck = cells.neck[0] * cells.neck_rows(layer) as usize;
        (body + sleeves - neck) as Real * spacing * spacing
    }

    #[test]
    fn the_default_pattern_is_a_sewn_shirt_with_four_openings() {
        let pattern = TShirtPattern::default();
        let garment = pattern.build().unwrap();
        let cells = garment.cells();
        assert_eq!(cells.body, [25, 35]);
        assert_eq!(cells.sleeve, [11, 10]);
        assert_eq!(cells.neck, [9, 4, 1]);
        assert_eq!(cells.neck_start, 8);
        let mesh = garment.mesh();
        assert_eq!(components(mesh), 1);
        assert_eq!(boundary_loops(mesh), 4, "neck, hem and two cuffs");
        let expected = pattern_area(cells, pattern.spacing, Layer::Front)
            + pattern_area(cells, pattern.spacing, Layer::Back);
        assert!((mesh.area() - expected).abs() < tol() * expected);
        // Every triangle is a right isosceles triangle with legs of one cell.
        for t in mesh.triangles() {
            let p = t.map(|i| mesh.rest_positions()[i as usize]);
            let mut lengths = [
                p[0].distance(p[1]),
                p[1].distance(p[2]),
                p[2].distance(p[0]),
            ];
            lengths.sort_by(|a, b| a.partial_cmp(b).unwrap());
            assert!((lengths[0] - pattern.spacing).abs() < tol() * pattern.spacing);
            assert!((lengths[1] - pattern.spacing).abs() < tol() * pattern.spacing);
            assert!(
                (lengths[2] - pattern.spacing * (2.0 as Real).sqrt()).abs()
                    < tol() * pattern.spacing
            );
        }
        // Seam hinges rest fully folded, panel hinges flat.
        let seams: BTreeSet<u32> = garment.seam_vertices().collect();
        let mut folded = 0;
        for hinge in mesh.hinges() {
            let across_seam =
                seams.contains(&hinge.vertices[0]) && seams.contains(&hinge.vertices[1]);
            if across_seam {
                folded += 1;
                assert!((hinge.rest_angle.abs() - pi()).abs() < tol(), "{hinge:?}");
            } else {
                assert!(hinge.rest_angle.abs() < tol(), "{hinge:?}");
            }
        }
        // Outline minus openings: two sides, two sleeve bottoms, two cuffs'
        // worth of nothing, two sleeve tops and two shoulders.
        let sides = 2 * (cells.body[1] - cells.sleeve[1]);
        let sleeve_bottoms = 2 * cells.sleeve[0];
        let tops = 2 * cells.sleeve[0] + (cells.body[0] - cells.neck[0]);
        assert_eq!(folded, sides + sleeve_bottoms + tops);
        // Layers: the back panel adds its own interior and opening vertices.
        let counts = garment.vertices().iter().fold([0; 3], |mut c, v| {
            c[match v.layer {
                Layer::Front => 0,
                Layer::Back => 1,
                Layer::Seam => 2,
            }] += 1;
            c
        });
        // Four seam chains (hem corner to cuff bottom, and cuff top to neck,
        // on each side); each has one more vertex than hinges.
        assert_eq!(counts[2], folded + 4);
        assert!(counts[1] > counts[0], "the back neckline is shallower");
        assert_eq!(counts.iter().sum::<usize>(), mesh.rest_positions().len());
    }

    #[test]
    fn landmarks_sit_where_the_pattern_says() {
        let pattern = TShirtPattern::default();
        let garment = pattern.build().unwrap();
        let rest = garment.mesh().rest_positions();
        let s = pattern.spacing;
        let at = |landmark: Landmark| rest[garment.landmark(landmark).unwrap() as usize];
        let (w, l) = (25.0 * s, 35.0 * s);
        let close = |a: Vec3, b: Vec3| assert!(a.distance(b) < tol(), "{a:?} vs {b:?}");
        close(
            at(Landmark::HemCorner(Side::Left)),
            Vec3::new(-w / 2.0, 0.0, -l / 2.0),
        );
        close(
            at(Landmark::HemCorner(Side::Right)),
            Vec3::new(w / 2.0, 0.0, -l / 2.0),
        );
        close(
            at(Landmark::CuffTop(Side::Left)),
            Vec3::new(-w / 2.0 - 11.0 * s, 0.0, l / 2.0),
        );
        close(
            at(Landmark::CuffBottom(Side::Right)),
            Vec3::new(w / 2.0 + 11.0 * s, 0.0, l / 2.0 - 10.0 * s),
        );
        close(
            at(Landmark::Underarm(Side::Left)),
            Vec3::new(-w / 2.0, 0.0, l / 2.0 - 10.0 * s),
        );
        close(
            at(Landmark::Shoulder(Side::Left)),
            Vec3::new(-w / 2.0 + 8.0 * s, 0.0, l / 2.0),
        );
        close(
            at(Landmark::NeckFront),
            Vec3::new(-w / 2.0 + 12.0 * s, 0.0, l / 2.0 - 4.0 * s),
        );
        close(
            at(Landmark::NeckBack),
            Vec3::new(-w / 2.0 + 12.0 * s, 0.0, l / 2.0 - s),
        );
        let info = garment.vertices();
        assert_eq!(
            info[garment.landmark(Landmark::NeckBack).unwrap() as usize].layer,
            Layer::Back
        );
        assert_eq!(
            info[garment.landmark(Landmark::Shoulder(Side::Right)).unwrap() as usize].layer,
            Layer::Seam
        );
        assert_eq!(
            info[garment.landmark(Landmark::HemCenter).unwrap() as usize].layer,
            Layer::Front
        );
        assert_eq!(
            info[garment.landmark(Landmark::CuffCenter(Side::Left)).unwrap() as usize].region,
            Region::LeftSleeve
        );
        assert_eq!(
            info[garment.landmark(Landmark::Chest).unwrap() as usize].region,
            Region::Body
        );
        // Seam corners resolve from either panel to the same vertex.
        assert_eq!(
            garment.vertex(Layer::Back, 0, 0),
            garment.vertex(Layer::Front, 0, 0)
        );
        assert_eq!(
            garment.vertex(Layer::Seam, 0, 0),
            garment.vertex(Layer::Front, 0, 0)
        );
        assert_eq!(garment.vertex(Layer::Seam, 3, 0), None, "the hem is open");
        assert_ne!(
            garment.vertex(Layer::Back, 3, 0),
            garment.vertex(Layer::Front, 3, 0)
        );
    }

    #[test]
    fn a_single_panel_is_one_open_sheet() {
        let pattern = TShirtPattern {
            layers: GarmentLayers::Single,
            spacing: 0.05,
            ..TShirtPattern::default()
        };
        let garment = pattern.build().unwrap();
        let mesh = garment.mesh();
        assert_eq!(components(mesh), 1);
        assert_eq!(boundary_loops(mesh), 1);
        let expected = pattern_area(garment.cells(), pattern.spacing, Layer::Front);
        assert!((mesh.area() - expected).abs() < tol() * expected);
        assert!(mesh.hinges().iter().all(|h| h.rest_angle.abs() < tol()));
        assert!(garment.vertices().iter().all(|v| v.layer == Layer::Front));
        assert_eq!(garment.seam_vertices().count(), 0);
        assert_eq!(garment.landmark(Landmark::NeckBack), None);
        assert!(garment.landmark(Landmark::NeckFront).is_some());
        let placed = garment
            .placed_positions(Vec3::new(1.0, 2.0, 3.0), 0.01)
            .unwrap();
        for (p, r) in placed.iter().zip(mesh.rest_positions()) {
            assert!(p.distance(*r + Vec3::new(1.0, 2.0, 3.0)) < tol() * 4.0);
        }
    }

    #[test]
    fn placement_separates_the_layers_about_the_seams() {
        let garment = TShirtPattern {
            spacing: 0.05,
            ..TShirtPattern::default()
        }
        .build()
        .unwrap();
        let origin = Vec3::new(0.2, 0.01, -0.3);
        let placed = garment.placed_positions(origin, 0.004).unwrap();
        for (p, info) in placed.iter().zip(garment.vertices()) {
            let rest = garment.mesh().rest_positions()[garment
                .vertex(
                    match info.layer {
                        Layer::Back => Layer::Back,
                        _ => Layer::Front,
                    },
                    info.cell[0],
                    info.cell[1],
                )
                .unwrap() as usize];
            let lift = match info.layer {
                Layer::Front => 0.002,
                Layer::Back => -0.002,
                Layer::Seam => 0.0,
            };
            assert!(p.distance(rest + origin + Vec3::Y * lift) < tol());
        }
        let mut cloth =
            crate::Cloth::new(garment.mesh().clone(), crate::ClothMaterial::default()).unwrap();
        cloth.set_positions(&placed).unwrap();
        assert!(garment.placed_positions(origin, -0.001).is_err());
        // Patches reach both layers; top patches leave the back panel out.
        let all = garment.patch(Landmark::CuffCenter(Side::Left), 0.11);
        let top = garment.top_patch(Landmark::CuffCenter(Side::Left), 0.11);
        assert!(all.len() > top.len());
        assert!(
            all.iter()
                .any(|&v| garment.vertices()[v as usize].layer == Layer::Back)
        );
        assert!(
            top.iter()
                .all(|&v| garment.vertices()[v as usize].layer != Layer::Back)
        );
        assert!(
            top.iter()
                .any(|&v| garment.vertices()[v as usize].layer == Layer::Seam)
        );
    }

    #[test]
    fn round_necklines_follow_the_curve_and_stay_well_shaped() {
        for (spacing, layers) in [
            (0.02, GarmentLayers::Sewn),
            (0.05, GarmentLayers::Sewn),
            (0.02, GarmentLayers::Single),
            (0.03, GarmentLayers::Single),
        ] {
            let notch = TShirtPattern {
                spacing,
                layers,
                ..TShirtPattern::default()
            };
            let round = TShirtPattern {
                neck: NeckShape::Round,
                ..notch
            };
            let garment = round.build().unwrap();
            let mesh = garment.mesh();
            // Fewer cells leave than with the rectangular notch.
            assert!(mesh.area() > notch.build().unwrap().mesh().area());
            assert_eq!(
                boundary_loops(mesh),
                if layers == GarmentLayers::Sewn { 4 } else { 1 }
            );
            let rest = mesh.rest_positions();
            for t in mesh.triangles() {
                let p = t.map(|i| rest[i as usize]);
                let area2 = (p[1] - p[0]).cross(p[2] - p[0]).length();
                let longest = [
                    p[0].distance(p[1]),
                    p[1].distance(p[2]),
                    p[2].distance(p[0]),
                ]
                .into_iter()
                .fold(0.0, Real::max);
                assert!(
                    area2 / longest >= 0.3 * spacing - tol() * spacing,
                    "thin triangle {t:?}"
                );
            }
            // Every front-panel vertex that left the grid sits on the ellipse.
            let cells = garment.cells();
            let mut snapped = 0;
            for (v, info) in garment.vertices().iter().enumerate() {
                if info.layer == Layer::Back {
                    continue;
                }
                let grid = Vec3::new(info.cell[0] as Real, 0.0, info.cell[1] as Real) * spacing
                    - Vec3::new(
                        cells.body[0] as Real * spacing * 0.5,
                        0.0,
                        cells.body[1] as Real * spacing * 0.5,
                    );
                if rest[v].distance(grid) > tol() * spacing {
                    let column = (rest[v].x + cells.body[0] as Real * spacing * 0.5) / spacing;
                    let row = (rest[v].z + cells.body[1] as Real * spacing * 0.5) / spacing;
                    let (xi, eta) = cells.neck_coordinates(Layer::Front, column, row);
                    assert!((xi * xi + eta * eta - 1.0).abs() < tol(), "{v}: {xi} {eta}");
                    snapped += 1;
                }
            }
            assert!(snapped > 0);
            assert!(garment.landmark(Landmark::NeckFront).is_some());
        }
    }

    #[test]
    fn stitched_seams_join_two_open_panels() {
        let shared = TShirtPattern {
            spacing: 0.05,
            ..TShirtPattern::default()
        };
        let stitched = TShirtPattern {
            seams: SeamJoin::Stitched,
            stitch_length: 0.001,
            ..shared
        };
        let a = shared.build().unwrap();
        let b = stitched.build().unwrap();
        assert_eq!(components(b.mesh()), 2);
        assert_eq!(boundary_loops(b.mesh()), 2);
        assert_eq!(b.stitches().len(), a.seam_vertices().count());
        assert_eq!(b.seam_vertices().count(), 0);
        let rest = b.mesh().rest_positions();
        for stitch in b.stitches() {
            let [f, k] = stitch.vertices;
            assert_eq!(b.vertices()[f as usize].layer, Layer::Front);
            assert_eq!(b.vertices()[k as usize].layer, Layer::Back);
            assert!(rest[f as usize].distance(rest[k as usize]) < 1e-12);
            assert_eq!(stitch.rest_length, 0.001);
        }
        // Placement separates the stitched pairs by the gap.
        let placed = b.placed_positions(Vec3::ZERO, 0.001).unwrap();
        for stitch in b.stitches() {
            let [f, k] = stitch.vertices;
            assert!((placed[f as usize].distance(placed[k as usize]) - 0.001).abs() < 1e-12);
        }
        // Seam stiffness does not apply without shared seams; no hinge is
        // scaled and no hinge rests folded.
        let stiff = TShirtPattern {
            seam_stiffness: 4.0,
            ..stitched
        }
        .build()
        .unwrap();
        assert!(
            stiff
                .mesh()
                .hinges()
                .iter()
                .all(|h| h.stiffness_scale == 1.0 && h.rest_angle.abs() < tol())
        );
        // Stitch validation.
        let mut mesh = a.mesh().clone();
        let n = mesh.rest_positions().len() as u32;
        assert!(
            mesh.add_stitches(vec![crate::mesh::Stitch {
                vertices: [0, 0],
                rest_length: 0.001
            }])
            .is_err()
        );
        assert!(
            mesh.add_stitches(vec![crate::mesh::Stitch {
                vertices: [0, n],
                rest_length: 0.001
            }])
            .is_err()
        );
        assert!(
            mesh.add_stitches(vec![crate::mesh::Stitch {
                vertices: [0, 1],
                rest_length: 0.0
            }])
            .is_err()
        );
        mesh.add_stitches(vec![crate::mesh::Stitch {
            vertices: [0, 1],
            rest_length: 0.001,
        }])
        .unwrap();
        assert!(
            mesh.add_stitches(vec![crate::mesh::Stitch {
                vertices: [1, 0],
                rest_length: 0.001
            }])
            .is_err()
        );
    }

    #[test]
    fn sleeve_stiffness_scales_only_the_sleeve_triangles() {
        let pattern = TShirtPattern {
            spacing: 0.05,
            sleeve_stiffness: 0.5,
            ..TShirtPattern::default()
        };
        let garment = pattern.build().unwrap();
        let regions = garment.triangle_regions();
        let scales = garment.mesh().triangle_stiffness_scales().unwrap();
        for (region, scale) in regions.iter().zip(scales) {
            assert_eq!(*scale, if *region == Region::Body { 1.0 } else { 0.5 });
        }
        assert!(regions.contains(&Region::LeftSleeve));
        assert!(regions.contains(&Region::RightSleeve));
        assert!(
            TShirtPattern::default()
                .build()
                .unwrap()
                .mesh()
                .triangle_stiffness_scales()
                .is_none()
        );
    }

    #[test]
    fn seam_stiffness_scales_only_the_seam_hinges() {
        let pattern = TShirtPattern {
            spacing: 0.05,
            seam_stiffness: 4.0,
            ..TShirtPattern::default()
        };
        let garment = pattern.build().unwrap();
        let seams: BTreeSet<u32> = garment.seam_vertices().collect();
        for hinge in garment.mesh().hinges() {
            let across_seam =
                seams.contains(&hinge.vertices[0]) && seams.contains(&hinge.vertices[1]);
            assert_eq!(hinge.stiffness_scale, if across_seam { 4.0 } else { 1.0 });
        }
        assert!(
            garment
                .mesh()
                .hinges()
                .iter()
                .any(|h| h.stiffness_scale == 4.0)
        );
        assert!(
            TShirtPattern {
                seam_stiffness: 0.0,
                ..pattern
            }
            .build()
            .is_err()
        );
        // A single panel has no seams to stiffen.
        let single = TShirtPattern {
            layers: GarmentLayers::Single,
            ..pattern
        }
        .build()
        .unwrap();
        assert!(
            single
                .mesh()
                .hinges()
                .iter()
                .all(|h| h.stiffness_scale == 1.0)
        );
    }

    #[test]
    fn patterns_are_validated() {
        let base = TShirtPattern::default();
        assert!(
            TShirtPattern {
                spacing: 0.0,
                ..base
            }
            .cells()
            .is_err()
        );
        assert!(
            TShirtPattern {
                spacing: Real::NAN,
                ..base
            }
            .cells()
            .is_err()
        );
        assert!(
            TShirtPattern {
                neck_width: 0.5,
                ..base
            }
            .cells()
            .is_err()
        );
        assert!(
            TShirtPattern {
                sleeve_width: 0.8,
                ..base
            }
            .cells()
            .is_err()
        );
        assert!(
            TShirtPattern {
                neck_depth_front: 0.7,
                ..base
            }
            .cells()
            .is_err()
        );
        assert!(
            TShirtPattern {
                spacing: 1e-6,
                ..base
            }
            .cells()
            .is_err()
        );
        // Odd body columns still centre the notch on whole cells.
        let odd = TShirtPattern {
            body_width: 0.46,
            ..base
        }
        .cells()
        .unwrap();
        assert_eq!(odd.body[0], 23);
        assert_eq!(odd.neck[0], 9);
        assert_eq!(odd.neck_start, 7);
        // A coarse sewn pattern still builds.
        let coarse = TShirtPattern {
            spacing: 0.1,
            ..base
        }
        .build()
        .unwrap();
        assert_eq!(boundary_loops(coarse.mesh()), 4);
    }
}
