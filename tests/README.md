# Test conventions

`test_*.py` presently verifies only repo foundation utilities. `fixtures/` contains **synthetic evaluator data**, not evidence of any hackathon system. Product tests must be added in the chosen stack after the brief and architecture are confirmed. Test assertions must independently observe outcomes rather than restating an LLM's claims.

The `test_ricebot_foundation.py` module checks the **pure-Python rice.v1** interface (measured sample locality and fusion, task assignment, bounded stop, drop acknowledgement, scripted simulator port). It does **not** launch Isaac Sim or verify physical seed release, contact or crop outcomes.
