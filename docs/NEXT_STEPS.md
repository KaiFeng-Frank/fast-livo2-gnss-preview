# Research status and next steps

Current release: **v0.1.0-research-preview**, 2026-10-13.

The [README](../README.md), [results](RESULTS.md), [reproduction guide](REPRODUCING.md)
and [provenance](PROVENANCE.md) are the public entry points. Historical design and
decision records remain under `docs/dev/`; private archive restoration instructions
are outside this publication.

A79 showed inconsistent fusion gains. A81's one permitted noise recheck failed
the SmartPNT Doppler gate, so A82 was not followed by configuration freezing,
final-arm creation or sealed evaluation. Publishing this preview does not reopen
those stages or relax the registered thresholds.

Further research needs a new documented protocol. A future performance release
should demonstrate repeatable gains with GNSS actually active, frozen settings,
predeclared diverse sequences, clear failure boundaries and explanatory ablations.
Engineering test counts and inactive-GNSS target passes are not performance evidence.
