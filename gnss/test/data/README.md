# External test fixtures

This public preview omits the historical raw receiver byte slices, stored message
definitions, RINEX observations and broadcast-navigation files. Their provenance
and redistribution scope were not established for this release. Tests that need
these files report explicit skips; they are not part of the portable validation claim.

The source generator in `synthetic/` accepts message definitions and broadcast
navigation as user-provided inputs. It does not bundle those inputs.

The independent preview demo generates idealized satellite ephemerides locally
and needs no external fixture. See [the reproduction guide](../../../docs/REPRODUCING.md).
