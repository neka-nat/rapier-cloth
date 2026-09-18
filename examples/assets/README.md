# Towel folding commands

`towel-fold-25hz.json` and `towel-fold-10hz.json` contain a 0.5 m square,
32x32-vertex mesh and recorded world-space grasp targets. Each recording has
four simulated seconds of folding followed by four seconds with every grasp
released. The two sample rates are actual physical step sizes; the example
does not insert substeps.

These generated numerical fixtures adapt the trajectory construction in
[Effective cloth folding trajectories in simulation with only two parameters](https://doi.org/10.3389/fnbot.2022.989702)
and its [author implementation](https://github.com/Victorlouisdg/cloth-manipulation).
They were captured from our square-towel scene using the author C-IPC fork
`e31e7a47e79edf07ceba5f67473962eeea01fa75`. The mesh and two-gripper scene are
adaptations, not a claim to reproduce the paper's shirt geometry.

The coordinates are data; no third-party solver or task source is bundled.
The accompanying example and these generated fixtures use this project's MIT
license. The floor's midsurface is y=0 and its thickness is 0.318 mm; the example
represents its top surface with a fixed Rapier halfspace at y=0.159 mm.

The author trajectory's last-frame convention is retained separately at each
sample rate. Consequently, these are two complete task fixtures, not an exactly
matched continuous-input temporal-convergence experiment.
