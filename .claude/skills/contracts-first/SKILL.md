---
name: contracts-first
description: Design or change shared module APIs with explicit typed fields, units, invariants, error semantics, fixtures, and one-writer integration ownership.
---

# Contracts-first interface review

1. Read `docs/ARCHITECTURE.md`, `docs/CONTRACTS.md` and task PR/issue; identify actual producer and consumer with owner(s).
2. Define a tiny versioned interface: exact fields/types, required/optional, nil semantics, units, frames, event ordering and timestamps.
3. Declare preconditions/postconditions, timeouts, retry limits, idempotence, malformed-input response and physical safety bounds.
4. Create an executable fixture or type/schema test that captures both valid and invalid examples. Do this **before** independently implementing producers and consumers.
5. Agree with integration lead who changes the shared file. If change is breaking, raise version and update all readers, writers and tests together.
6. Verify actual integration test; if unavailable, mark unverified, not compatible by assertion.

Deliver: contract ID/version, implementation paths, invariant, fixture/test command, owner, compatibility decision.
