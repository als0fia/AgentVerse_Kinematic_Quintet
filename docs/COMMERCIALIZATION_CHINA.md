# Southern China rice robotics — commercial evidence and validation gates

**Researched 7 November 2026; reconciliation of PR #22.** This is external research for judging relevance and impact, not proof of market demand, the team's hardware capability, farm deployment, or a revised project scope. [PRODUCT](PRODUCT.md) remains canonical: **Antenna is rescue-first**, with measured mapping and planting as meaningful task context.

## Verified statistics and what they mean

| Source | Evidence | Qualified takeaway |
| --- | --- | --- |
| [China 2025 national agricultural machinery bulletin, released 22 Oct 2026](https://www.amic.agri.cn/secondLevelPage/info/30/227524) | National combined crop tillage/planting/harvest mechanisation 76.82%; all-crop planting 67.06%; approximately 80,600 agricultural machinery co-ops | A substantial mechanised farming/services ecosystem, but **neither a Southern rice planting rate nor an Antenna addressable market** |
| [MARA-linked hilly rice equipment report, 2025](https://nyncw.cq.gov.cn/wsdw/cqsjxh/mtbd/202508/t20250807_14884221_wap.html) | National *rice planting* mechanisation 67.6% as of 2024 | Rice establishment has an uneven mechanisation gap. The unmechanised share does **not** equal paying customers for small autonomous rescue rovers |
| [Zhao et al., Transactions of the CSAE (2025)](https://www.sciopen.com/article/10.11975/j.issn.1002-6819.202411006) | Hilly/mountainous rice regions face shortage of lightweight, efficient, stable equipment and uneven mechanisation | Real access and reliability challenge; no proof of specific mud depth, rescue frequency, unique capability or guaranteed demand |
| [Pi et al., Transactions of the CSAE (2026)](https://www.sciopen.com/article/10.11975/j.issn.1002-6819.202510103) | Comparison of transplanting, direct seeding and agricultural UAV seeding; bottlenecks include light equipment for hilly areas and agronomic suitability | A useful comparative research direction, **not** evidence that targeted patch replanting or robot-to-robot rescue increases farm profit |

**Related internal material:** The earlier unverified TAM/SAM/SOM calculations and commercialization prompts (retained in Git history, excluded from this submission snapshot) preserve the original calculations and proposed targets **as explicitly unverified inputs**; they are not alternative factual authorities. Their machine-local links have been repaired. This document is the evidence audit to consult before using any claim externally.

**Unsupported PR #22 numerical claims, removed from current truth:** 53.28% / 41.17% as *verified Southern rice-specific* planting rates, >10 million hectares of underserved Antenna demand, 15–30% emergence gaps, the 65% regional acreage share, exact farmer wages/ages, prevailing mud depths, a $4.28 billion national robotics market, equipment price bands, 2.5–5% fleet adoption and sub-year buyer payback. The original TAM/SAM/SOM tables combine unverified market populations with assumed fees, hardware prices and subsidies. They are neither independently sourced statistics nor evidence for judging.

## Credible competing approaches

| Alternative | Primary evidence | Competitive pressure / honest distinction |
| --- | --- | --- |
| [Kubota NW8SA autonomous rice transplanter](https://www.kubota.com/innovation/report/no56.html), [Yanmar YR8D auto rice transplanter](https://www.yanmar.com/global/about/technology/technical_review/2019/1001_1.html) | Manufacturers describe automated field operation and RTK-based control | Do not claim all incumbent planters are manual, blindly AB-line driven or invariably stuck. Field geometry and extraction costs need comparable evidence |
| [DJI Agras T50](https://ag.dji.com/t50) | Manufacturer documents autonomous mapping/spreading, sizeable granule payload and obstacle sensing | Strong high-throughput alternative; a ground rover's contact sensing must justify much slower travel |
| [NEWGREEN Aigamorobo IGAM2 trials](https://sp.coinext.tuat.ac.jp/wp-content/uploads/2025/09/2024AnnualReportTotal250904_compressed.pdf) and [2026 paddy weeding robot study](https://www.fujipress.jp/jrm/rb/robot003800020404/) | Small paddy autonomous machines already exist | Small size and bio-inspired agriculture alone are not novel; these tackle different jobs |
| Human retrieval or winch service, simple self-unblocking, **deterministic risk optimiser**, safe abandon-and-continue | Must quantify on actual farm or controlled physical scenes | Closest alternatives to **rescue**; do not benchmark only against a deliberately bad policy |

## Commercial viability hypotheses and evidence gate

1. **Beneficiary:** name a rice farming operation and season, document incidents per machine-hour, staff time and damage when stuck, field accessibility, and whether waiting for retrieval is cheaper. Evidence can come from operator interviews, public field studies or qualified maintenance logs, not an invented pain-point figure.
2. **Feasibility:** measure actual vehicle mass/traction, body pressure, safe crop rows, docking envelope, recovery forces, probability of second casualty, water sealing, service turnaround and unattended runtime. Present current MuJoCo rigid-plane soil as synthetic, **not** validated clay or plow-pan physics.
3. **Measurable added value:** compare total rice work completed, crop contact damage, robot time, recovery success and secondary immobilisation against **never rescue**, **always rescue**, manual retrieval assumptions and a calibrated numerical optimiser.
4. **Regulation and incentives:** [MARA's 2024–2026 machinery-purchase subsidy rules](https://njhs.moa.gov.cn/tzggjzcjd/202406/t20240604_6456671.htm) normally calculate provincial equipment amounts at up to **30%** of prior-year comparable sales price; some priority items may be raised up to **35%** subject to conditions ([full policy via Hubei government](https://nyt.hubei.gov.cn/bmdt/ztzl/wqzt/njgjbtzl/gjbtgs/202406/t20240605_5227981.shtml)). Antenna has **no verified qualifying category**, and 35–40% subsidy, BDS-only qualification, universal 3% crop-damage and 5% missed-placement standards, or automatic domestic mapping-storage mandates must **not** be asserted.
5. **Unit economics:** annual operator benefit = avoided retrieval labour + avoided crop loss + value of work completed minus fleet depreciation, maintenance, supervision, communications, energy and second-casualty loss. A one-time hardware sale, gross service transaction value and recurring company subscription revenue must not be summed as if all are annual profit. Test sensitivity to incident frequency, coverage rate, farmer budget and crop season.

**Pitch-safe claim:** The literature identifies real hilly-rice mechanisation bottlenecks; Antenna tests whether peer robots can handle immobilisation without sacrificing rescue safety or useful fieldwork. The code has simulated gossip and isolated scripted rigid extraction; the full autonomous rescue loop and field economics are unproven.

**Provenance:** PR #22's competitor list, Chinese terrain/use case, cooperative service model and failure-cost motivations are preserved as directions; unsupported numeric absolutes are deliberately excluded. Original source and authorship remain in PR history.
