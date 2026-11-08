# AgentVerse 2026 — Antenna submission and demo

**Status:** submission-candidate **source branch prepared**. No official portal submission, uploaded one-page PDF, accepted video, or judging invitation has been verified by this file.

**Team:** The Kinematic Quintet · **Track:** Robotics / Embodied AI · **Product:** Antenna

## Required organiser deliverables

Based on the organiser's photographed Robotics slides, summarized with source qualifications in [EVENT](EVENT.md):

| Item | What to produce | Current evidence |
| --- | --- | --- |
| Source/project link | Share the tested submission-branch URL and exact commit, if the portal requests it | This branch; verify accessibility and portal fields |
| **One-page technical PDF** | Problem and beneficiary; sensing → decision → action; what is simulated; measurable outcomes; safety and limitations; model placement and actual latency if measured | **Not verified as uploaded**; prepare/export outside Git or commit a final team-approved PDF only if allowed |
| **≤60-second continuous simulation video** | Real single-take MuJoCo run with sensor/perception information and robot actions visible together, including an unexpected mid-episode event and adaptive response | **Not verified as recorded** |
| **Five-minute pitch if shortlisted** | Beneficiary, product action, live evidence, safety/refusal example, strongest measured comparison and limitations | Rehearse only against actual tested functionality |

**Deadline conflict:** participant BuilderBase dashboard supplied on 7 November indicated **Sunday 8 November 2026, 13:00 GMT**; the older public organiser timetable listed 14:00. Work against the **earlier deadline**, and check the live participant portal or organiser directly. This repository does not control submission receipt or eligibility.

## Recording the actual program

Use the exact commit selected for submission; do not record a moving feature branch without pinning the SHA.

Install requirements from repo root:

~~~powershell
python -m pip install -r requirements-mujoco.txt
~~~

Start the controlled physical rescue:

~~~powershell
python scripts/run_mujoco_paddypatch.py --presentation --scenario golden_rescue --policy cooperative --rescue-policy react_orchestrator --seed 7
~~~

Before recording, do one full rehearsal. Check that the program genuinely observes the immobilisation, selects a helper, physically docks/latches/extracts, and then records continued movement and seed contact. Use the Presentation / Engineering toggle to expose live perception and actions. The camera modes **Field**, **Rescue** and **Auto frame** alter the view, not physics or robot policy.

**Capture rules:** one continuous ≤60 s screen recording of the ongoing simulation, with no spliced scenarios, pre-rendered paths or manufactured control outputs. Observe an unplanned-for-the-controller event occurring *within* the run; the configured seeded mud incident is reproducible, not a claim that geological failures are fully emergent. Include visible timestamped decision and sensor evidence. Confirm actual duration with the final video file.

A refusal can be demonstrated separately (not spliced into the required continuous video):

~~~powershell
python scripts/run_mujoco_paddypatch.py --presentation --scenario high_risk_abstain --policy cooperative --seed 7
~~~

For the expanded collective fleet:

~~~powershell
python scripts/run_mujoco_paddypatch.py --dashboard --scenario randomized --policy mycelial --seed 8
~~~

Select 6 rovers and 1.6× field in the Engineering setup controls. Distinguish the **expanded fleet feature** from the **controlled golden rescue**, which uses the cooperative policy. Mycelial has different rescue reliability; do not imply all modes achieve the same outcome.

## Evidence the pitch can safely use

- The controlled cooperative golden fixture records a simulated rigid latch, physical displacement of at least 0.18 m, resumed independent motion and two subsequent viable seed placements. The high-risk scenario demonstrates safe refusal and no second casualty; an unconditional-rescue baseline can strand the helper.
- [Windows release verification](TESTING.md) at commit **32ce0a3** reported **142 passing tests**, no skipped GUI tests, and a tested local application window with six-rover playback.
- The later [Presentation Mode CI](https://github.com/als0fia/AgentVerse_Kinematic_Quintet/actions/runs/42478808632) at commit **60c7283** ran **149 tests successfully, with two headless GUI skips**. A fresh Windows visual check of *this exact presentation revision* is still required.
- The [18-run randomized report](../experiments/release_verification_20261108.json) records six matched Mycelial gossip/isolated pairs: viable seed-placement advantage in 5, tie in 1; map divergence lower with gossip in all 6. This is a small synthetic experiment, not significance testing.
- Failures remain visible: collisions in two four-rover Mycelial runs, one secondary casualty during six-rover rescue, incomplete missions and failed extractions. **No LLM API calls** occurred in those results.

See [independent test evidence](TESTING.md), [provenance and AI disclosure](PROVENANCE.md), and [run guide](RUN.md).

## Final human sign-off

- [ ] Pin a SHA and preserve the successful test report and actual Windows GUI result.
- [ ] Confirm the real camera, docking, extraction and planting correspond to what the captions say.
- [ ] Make a genuine, timed single-take video and confirm its content is legible.
- [ ] Export the approved one-page PDF; confirm one page, readable text and accurate architecture/limitations.
- [ ] Ensure the source branch is accessible at the URL supplied in the submission; confirm GitHub repository visibility satisfies the portal rules **without changing visibility here**.
- [ ] Review dependency licences and material AI/prior-work attribution against the **actual** event restrictions, which were not fully captured.
- [ ] Confirm names, team members, files and submission fields with the authorised team.
- [ ] Have the human team owner approve the final content and perform/authorise the portal submission.
- [ ] Save the actual organiser submission receipt/confirmation.

**No automatic submission or public deployment is performed by this branch-preparation task.**
