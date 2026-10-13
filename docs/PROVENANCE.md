# Source and evidence provenance

## One public source identity

The release source is the `v0.1.0-research-preview` tag in
`KaiFeng-Frank/fast-livo2-gnss-preview`. GitHub's automatic ZIP/tar archives
and `git checkout v0.1.0-research-preview` contain the same tree. The tag is
not an evidence placeholder and will not be moved to a different snapshot.

This publication is a curated source snapshot of the final 2026-10-03 private
integration, commit `696d38d966b6a32b4901bf0aeb884d9778e834ca`, tree
`473b025097ab7685010f1ac204af38f97242def5`. It includes A80–A82, which were
not on the old `main@c3f27dc`. Its public history begins with the reviewed
snapshot; it does not copy old branches, tags, Actions logs or Release assets.
The original private research repository remains separate.

Upstream FAST-LIVO2 base: `0d2c0346107b75b59934975adec9a6eeeb913c64`.
Upstream and dependency notices are retained in [THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md).

[PUBLICATION_MANIFEST.json](PUBLICATION_MANIFEST.json) records the baseline,
excluded paths and the SHA256 identities of retained baseline files, including
all current configuration overlays. It also lists changed files. This permits
an exact comparison with the private source if that archive is available; the
public release itself does not require access to it.

## Changes made for publication

- Replaced the homepage and next-step summary; added a reproducible core/demo
  path and numerical evidence documentation.
- Generalized personal workstation/server paths, removed hardcoded remote
  addresses, made remote connection settings explicit and removed named-SSD
  discovery from local helpers.
- Omitted private restoration/operations records, unreviewed raw receiver
  fixtures and stored third-party message definitions, old upstream media and
  ground-truth bundles. Omitted unused `sofa.c`, whose referenced license text
  was absent from the imported subset.
- Retained the research design and decision log as historical records, with
  illustrative paths and a banner distinguishing them from the release guide.
- Added current third-party notices and explicit missing-fixture test skips.
  The release verification reports those skips; it does not count them as passes.

The release does not retune the estimator, change experiment splits or relax
performance gates. Numerical evidence is presented with its original identity.

## Historical results are not runs of the public tag

The A79 table predates the A80–A82 integration. Its U/T builds, source identities,
configuration hashes, activation and scoring conventions are recorded with the
[results](RESULTS.md). A81/A82's failed development gate is also kept there.
A public tag identifying newer code does not make it the producer of older runs.

The reported historical Python checkpoint (482 passed, 6 skipped), 21/21 CTest
targets and five bitwise regression groups describe the original environment.
The public preview's portable checks are separate and are documented in
[REPRODUCING.md](REPRODUCING.md). Raw-fixture and ROS-dependent checks can skip in
the public environment. No full ROS replay or cross-machine bitwise equivalence
is claimed by this release.

## Publication boundary

Only the reviewed source snapshot and listed research evidence are published.
The old Docker image, executables, linked libraries, complete experiment logs,
raw sensor recordings, private Git history and remote-operation records stay
in the private archive. No archived Release asset is copied to this repository.
The full ROS host dependency license issue is described in [LICENSING](LICENSING.md).
