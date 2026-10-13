# Licensing and publication scope

This research preview distributes source code and project-generated research evidence. It is a
new public snapshot prepared from integrated source `696d38d966b6a32b4901bf0aeb884d9778e834ca`
(2026-10-03). The private archive, its Git history, release attachments, executables,
shared libraries and Docker images are not part of this distribution.

## Licenses by component

| Component | License | Scope |
|---|---|---|
| FAST-LIVO2 host source and its modifications, `src/replay_main.cpp`, `ros/`, root build files | GPL-2.0-only | Root `LICENSE` and `LICENSES/GPL-2.0-only.txt`; upstream notices retained |
| Independently authored `gnss/`, `gnss_io/`, `tools/`, `docker/` | BSD-3-Clause | File SPDX identifiers and `LICENSES/BSD-3-Clause.txt` |
| `third_party/rtklib/upstream/` retained RTKLIB-EX source | BSD-2-Clause | Original `license.txt`, `readme.txt`, file notices and provenance retained |
| Project RTKLIB build files, stubs and patches | BSD-2-Clause | File SPDX identifiers; patching happens in the build directory |
| Generated `include/livox_ros_driver/CustomMsg.h` and `CustomPoint.h` | Third-party MIT/BSD notices | Livox message definitions and ROS generator ancestry; see `THIRD_PARTY_NOTICES.md` |
| Standalone `third_party/patches/rpg_vikit/` patch | GPL-3.0-only | External dependency context and changes; `LICENSES/GPL-3.0-only.txt` |
| M3DGR-derived configuration and calibration facts | MIT upstream; project changes GPL-2.0-only | Original M3DGR notice retained in `LICENSES/M3DGR-MIT.txt` |
| Other host configuration, launch files, scripts and repository documentation | GPL-2.0-only unless a file or notice specifies otherwise | Component-specific notices take precedence |

The top-level GPL license does not replace the separate licenses of independently authored
BSD components or third-party material. Preserve the relevant license text, copyright,
source provenance and modification notices when redistributing. The original FAST-LIVO2
source notices remain intact. Project modifications date from September–October 2026 and
add the GNSS core, I/O, ROS integration, deterministic replay and evaluation tools, as well
as the documented host reproducibility and initialization changes.

## Full ROS host dependency remains unresolved

The host's external dependency
[`xuankuzcr/rpg_vikit@6c886c8`](https://github.com/xuankuzcr/rpg_vikit/tree/6c886c8e5d83997806e00294826d528cea3581dd)
identifies its packages as GPLv3. The FAST-LIVO2 host is distributed here as GPL-2.0-only.
GNU's [GPL compatibility FAQ](https://www.gnu.org/licenses/gpl-faq.en.html#v2v3Compatibility)
explains that GPLv2-only and GPLv3 are incompatible when code is combined into one work.

This preview does not vendor the rpg_vikit package, an installed dependency tree, a linked
host executable, a shared-library bundle or a Docker image. The small standalone build-flags
patch includes upstream CMake context and is retained separately under GPL-3.0-only, with
its own license text; it is an independent patch rather than bundled dependency code.
The source-only scope does **not** establish that the complete linked ROS host can be
redistributed. Resolving that combination requires compatible
permission from the relevant rights holders or a compatible dependency implementation before
distributing it. A local build recipe and a Dockerfile are source instructions, not permission
for redistribution of their resulting image or binaries. The standalone ROS-free GNSS core
uses Eigen and the retained BSD RTKLIB source; it does not depend on rpg_vikit.

## GNSS message boundary

`gnss_comm` declares GPLv3. This public snapshot includes no gnss_comm implementation,
generated headers, `.msg` files, or full `.msgdef` fixture dumps. Runtime I/O accepts
message definitions supplied by the user's own ROS1 bag, checks the expected wire layout,
and decodes measurement bytes. The separately written layout tables describe the interface;
they are not an included gnss_comm package. The code does not compile or link gnss_comm.

The private archive contained connection-header message-definition dumps alongside test
fixtures. They have been omitted here, including those accompanying synthetic observations.
The retained synthetic generator takes message definitions and broadcast navigation data as
external inputs; generated files are not bundled. This distinction corrects the older
archive's broad claim that it contained no message-definition text.

## RTKLIB source subset

The retained source comes from RTKLIB-EX tag `v2.5.1`. Original source files, copyright notices,
`license.txt` and `readme.txt` are retained verbatim; two project patches are separate.
See [`third_party/rtklib/VENDORED.md`](../third_party/rtklib/VENDORED.md) for file hashes.

The unused SOFA-derived `upstream/src/sofa.c` and upstream `lib/` dependency trees are not
included. The project's CMake target has an explicit source list and does not enable
`SUNPOS_SOFA`, `MOONPOS_SOFA` or `IERS_MODEL`. No SOFA or IERS library is built or bundled by
that target. Keeping the upstream generic build files documents provenance; build this
subset through the project's `third_party/rtklib/CMakeLists.txt`.

## Data and results

The [M3DGR repository](https://github.com/sjtuyinjie/M3DGR/blob/main/LICENSE) licenses its
software and accompanying documentation under MIT. That notice is retained for copied
configuration. It does not establish the redistribution terms of every externally hosted
bag, ground-truth file, GNSS schema, IGS product, SmartPNT or MARS-LVIG download.

Raw bag slices, byte fixtures, message-definition dumps, broadcast navigation files,
third-party ground truth, the upstream supplementary PDF and inherited dataset logs are
omitted from this preview. Obtain datasets from their official publishers and follow their
terms. Published aggregate results and project-generated estimates record their source
experiment, configuration and scoring convention; they do not grant rights over the
underlying datasets. Test cases that require omitted fixtures may report a skip.

This file records the scope actually published and the identified dependency limitation.
It is not a declaration that all future combinations or private archival artifacts are
cleared for redistribution.
