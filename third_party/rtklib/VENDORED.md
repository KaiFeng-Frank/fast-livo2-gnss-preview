# RTKLIB-EX source subset and patches

Upstream: [rtklibexplorer/RTKLIB, tag v2.5.1](https://github.com/rtklibexplorer/RTKLIB/tree/v2.5.1).
The retained upstream files are unmodified and preserve their file notices,
[`upstream/license.txt`](upstream/license.txt) (BSD-2-Clause) and `readme.txt`.
This is a source subset, not the complete upstream distribution: the unused SOFA-derived
`upstream/src/sofa.c` and all upstream `lib/` trees are excluded. The project's explicit
CMake source list does not define `SUNPOS_SOFA`, `MOONPOS_SOFA` or `IERS_MODEL`.
Use this project's CMake target; upstream generic build files are retained for provenance.

## Project patches

Both patches were authored for this project in September 2026 and are BSD-2-Clause.
They are applied in filename order to a copy under the build directory; retained upstream
source remains unchanged. `LIVO_RTKLIB_APPLY_PATCHES=OFF` creates the unpatched comparison
build used by the golden-file checks.

| Patch | Change | Numerical scope |
|---|---|---|
| `P1_maxprncmp.patch` | Make `MAXPRNCMP` overrideable; default to 63 and assert `MAXSAT` fits `obsd_t.sat` | Same-satellite arithmetic unchanged; includes BeiDou C51–C63 and moves later system numbers |
| `P2_export.patch` | Export wrappers around `prange`, `varerr`, `gettgd` and the converged SPP linearization/result | Copies internal results for GNSS diagnostics; the project checks against unpatched golden output |

RTKLIB has global state and the P2 diagnostic buffer is not reentrant. The adapter uses
single-threaded calls. The static library's C symbols are hidden in the GNSS library
(`--exclude-libs`); this is symbol management and does not alter the license.

The adapter corrects its own code-bias selection for Galileo I/NAV/F/NAV. The upstream
`gettgd()` first-record behavior and RTKLIB health masks are documented in the core tests.
These are project adaptation notes, not additional upstream modifications.

## Golden checks and optional data

`golden/pntpos_unpatched_synth.hex` is a project-generated numerical reference, checked by
`test_u05_corrections`. It uses synthetic code/Doppler observations rounded to RINEX
precision (1 mm / 1 mHz). It is not an executable or a raw dataset slice.

Some U3/U4/U5 tests expect an external IGS navigation excerpt named
`BRDC00IGS_R_20250030000_01D_MN_0400-0900_GRECJ.rnx`, with complete header and G/R/E/J/C
records in the 2025-01-03 04:00–09:00 interval. Expected bytes: 1,768,516;
SHA256: `b9132287f7c3952101a9c5fc938fa81f1fb61b9d92e8193c190ca842cefd09c6`.
It is not redistributed here. Set `LIVO_GNSS_BRDC_DIR` to the directory containing an
independently obtained excerpt. Missing optional data produces an explicit skipped test;
a mismatched supplied file fails. Archived M3DGR fixture directories are also excluded
from this public snapshot.

To regenerate project goldens in a controlled environment, configure a separate build
with `-DLIVO_RTKLIB_APPLY_PATCHES=OFF`, then set `LIVO_GNSS_WRITE_GOLDEN=1` and
`LIVO_GNSS_GOLDEN_OUT` while running the relevant U5 golden test. Compare on the same
compiler/C-library profile before replacing a registered golden. Cross-platform bitwise
identity is not promised.

## Retained upstream SHA256 manifest

Hashes apply before the project patches. Every included file under `upstream/` is listed;
SOFA-derived `sofa.c` is intentionally absent.

| File | SHA256 |
|---|---|
| `upstream/license.txt` | `219747832d49ee958457b2934080ab8d94bd9d8e45fcb1c36f89776fd2c5ed8a` |
| `upstream/readme.txt` | `7f9d69d9082282c476b9a5599f1d2555f494a61da10dff6ebe577e129ce4efaf` |
| `upstream/src/CMakeLists.txt` | `a3ff481260efaa1668207023efe836ebb523d0c73500ff8f0b16cb4673f7d430` |
| `upstream/src/convgpx.c` | `fa9193a094c493b3a2b95b56254fda7520463b9e66c15ecea1878022e9a5ed06` |
| `upstream/src/convkml.c` | `c9e8a6431f7f74a72aef7684c5d2173e1e583e62e4775caabdd584ab0df4ffe8` |
| `upstream/src/convrnx.c` | `40a4cb9a92dff6e490ea223bffd5bd2ef9ddc9ad5b4145cdca6bb0941f5cdff0` |
| `upstream/src/datum.c` | `7c9e05346017a9f6fb0853d2ebdbaf18a04dc39d208f9b8c83be4005c6c14b85` |
| `upstream/src/download.c` | `08f2e903c33bb12fca0508f672f9bdfdd3fdbde2330b93e215a3c3a940d50b2a` |
| `upstream/src/ephemeris.c` | `3ff70282f5df82ab2f271abbfd2a9ddde069f00f7545660d6947f95f56ef281a` |
| `upstream/src/geoid.c` | `b33af035739078b8aa5a2d41d82d697f7a923ada1a1a807e0daa2bf366bedb81` |
| `upstream/src/gis.c` | `c4d2cb86400f550921dc58b756927c72c3eab3d78a9a9d0eb3719f62b0f0d983` |
| `upstream/src/ionex.c` | `bc2f3658f963f6fb6922ee4e1b9ad0db5bc9e9d35bd12964c535166a6a87dfe1` |
| `upstream/src/lambda.c` | `52555d2b64733869325d3dffd390bc0858b784c04bbf57a3d170138014a456e1` |
| `upstream/src/options.c` | `92751e9f926f0568020dc03c725ff1329e0be95bf9dd4f3717269bfc383de80c` |
| `upstream/src/pntpos.c` | `fdfce7425566fbc90bf0b39ce1aec6d85172922b014d63eac5e582bde849712f` |
| `upstream/src/postpos.c` | `047e4ff6fc52824f3eeffbbdd56acfa67703b36d9537fcf5db2454b4b3e54ebf` |
| `upstream/src/ppp.c` | `121ca78fc486f92d9d103e778cd5eaf394c2928744d071b1ab36d13927a928a5` |
| `upstream/src/ppp_ar.c` | `8360dbfc270556149a4b974e059bd451528a07862c67dfcd07d1a2c72851b534` |
| `upstream/src/preceph.c` | `84dc7dc6be91890e71070be44870ce72e7b6c4fad846c008a9b86be54f5f0123` |
| `upstream/src/rcv/binex.c` | `b147a754e0175a14eb488625e81a36c8dc3a120b03fe80ddea313dfcda4d87b0` |
| `upstream/src/rcv/comnav.c` | `9b8f6829a700da292b691f0505584f6fedcebe2cb4eef27e2f3ca32d6b772ad4` |
| `upstream/src/rcv/crescent.c` | `c4d4184e9ab2a4987e6ca86273dcfb1e038c489bd151d5ecc93f6d8c42f02159` |
| `upstream/src/rcv/javad.c` | `e750bd8b901d4fc0d45b377a8be2b70519f875eb69d6a16962d4ee2853d44cd4` |
| `upstream/src/rcv/novatel.c` | `71b8f6e828f4a0a242f07eec7578fb402750726e24a5bce04edf4f3ac71ceb03` |
| `upstream/src/rcv/nvs.c` | `5dc9f194d2e5d9ea1f844d0374e537fab4403bc6e7f0424d0d5ba90b41b662dc` |
| `upstream/src/rcv/rt17.c` | `3dd6bdc822624fb3d2d5cebfa201e6ae9a7c85c2491a30410ee4f57684d70c51` |
| `upstream/src/rcv/septentrio.c` | `88c851446e248eb2f8398a9741886259891371e52a6939986cb3fc79aca3b498` |
| `upstream/src/rcv/skytraq.c` | `0bb1c9418b66005bc7a93499a0d7d5413067cd984c8ba997a771fd89162b72e9` |
| `upstream/src/rcv/swiftnav.c` | `5c4f2c3f2aae0d4d75864827fbbb32e0702d9f3839b27f078530a0fdd6eea6db` |
| `upstream/src/rcv/tersus.c` | `d84e099ea1c06b7cea2e9ac1e47cda037603726709067e0f4b226915f9bd413d` |
| `upstream/src/rcv/ublox.c` | `880f15809bf652aa279d62b2d4beb7a8d7864536856182a916e5a88c432ca001` |
| `upstream/src/rcv/unicore.c` | `06ad88e1b091842588099dc33dee39f0c98a86c9a6274fbdd7b0797705c638ed` |
| `upstream/src/rcvraw.c` | `0c3875135961de3a5d852b3810efc689e1bab28934ec0a0070b3107e1a5a26df` |
| `upstream/src/rinex.c` | `19f27f3841e5c4ff3646b2ed232d57fb213841980302aa1d7ab9717f665bbf3c` |
| `upstream/src/rtcm.c` | `b6d063dd13b342e9c843f71052008aca919670e26b28d0b9117b6f9285ee644d` |
| `upstream/src/rtcm2.c` | `1d929423ca1d692e7bb0104aabbf713f88d066b1a0ad251cbc5bd52a0eb34212` |
| `upstream/src/rtcm3.c` | `6fbfd2399eee72df99f1bf92505c9b9b0edf7ce001a1f865eac841875c1c783b` |
| `upstream/src/rtcm3e.c` | `3ac46759ac80c45a23c96226448dc1f6837f4bc0c079cf80fda74e71883a3361` |
| `upstream/src/rtkcmn.c` | `6722c36f1bd49676a8cc385b583612276a836a0884b83fa7328fe303e493308a` |
| `upstream/src/rtklib.h` | `cb1c3e6b2f3bb1387740bd871f2d53dcac02bc51cd75907616c1e6fdd77938fa` |
| `upstream/src/rtkpos.c` | `6d094d5a065c820c99f0b890d7b829d11aed9538a3a4f2ca20ea5f4004f33799` |
| `upstream/src/rtksvr.c` | `b019ffa217401b1a6ff5ee8b927c6551b712f57938dc4aa29d8dab2d3e32caf0` |
| `upstream/src/sbas.c` | `75eabf57e89868f23d7afdb3acbc82d70032aac61c869eec280c23e9a1b51de4` |
| `upstream/src/solution.c` | `5747d98113441899a5f2f5da886618923441330058d3d22fdf97b0fcbb98634b` |
| `upstream/src/src.pro` | `2e68e3f56802983792818276053b9ba83733b698fd3838202dd008767d9479d2` |
| `upstream/src/stream.c` | `b5273424c74d7301dac9b7a22cc454765d5b3c4f5454932028d1c86697c8b255` |
| `upstream/src/streamsvr.c` | `a09cf6fee703ec7acec9533358dcaa0932364e787856114e48779c16a305cef1` |
| `upstream/src/tides.c` | `fe41d01eb43f231e9734d7d81de9df8a67a62c30e2c614387c4b9996bf7836ea` |
| `upstream/src/tle.c` | `8d0ee8919ae4eeb8fa321cc91952aac73e09cb8eed3c5a16666461323b765ee8` |
| `upstream/src/trace.c` | `89affdfdcb891e6a155975217985d9fd046c6cfaa010b7bd2d209c16be0d030e` |

## Project patch SHA256 manifest

| File | SHA256 |
|---|---|
| `patches/P1_maxprncmp.patch` | `ff6cd0634a43b1a67502169aca5c9db8569173f62ea11eb2433cf79290f9077a` |
| `patches/P2_export.patch` | `2d826e97681fa7b267a71d4c1acc8ffa8aa5647cd0f490b9648a2a08f9fbae74` |
