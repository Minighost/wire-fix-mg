# v0.20 성능 최적화 기록

5493극 layout (layout_max_dim=5520) 기준 wire_fix 알고리즘
(`algo-v20.lua`)의 실행 시간 변화. v19 final (131s) 에서 시작해
hierarchical multi-scale + chain pruning + incremental enumeration
을 거치며 **421s → 185s**에 도달.

목표는 **5s 이내**. 현재 거리: 37x.

알고리즘 의미는 보존 — 매 단계 같은 accepted parl 집합과 같은
edge 집합을 산출하도록 동등성 유지.

## 시작점

- 베이스: v19 (131s, 5428극).
- v20 동기: v19은 `enumerate_parls`가 모든 미드포인트 페어를
  단일 패스로 처리. 채택된 parl이 늘어도 페어 수는 감소하지 않음.
  → hierarchical scale + dead pole pruning 으로 단계마다 검색
  공간을 줄이는 구조.

## 단계별 결과

| 커밋 | 변경 | total (ms) | 비고 |
|---|---|---:|---|
| v19 final ([1e32596]) | baseline | 131,000 | 5428극 |
| [1a50367] | v20 초기: hierarchical {32, 64, 128, 256} + dead pole 마킹 | (측정 안 함) | |
| [bc9c898] | SAT containment override (큰 사각형이 작은 거 흡수) | (측정 안 함) | |
| [b22a890] | dynamic scale schedule (32→layout_max) + per-scale timing | **399,500** | 9 스케일, 5493극 |
| [5363899] | per-scale insert/iterate/pair_checks instrumentation | (측정 안 함) | logic 변경 없음 |
| [8a47fce] | chain pruning: dead intermediate면 chain reject | (측정 안 함) | |
| [4e56e2a] | cell clamping: enumerate insert cell range를 layout bbox로 자름 | **421,600** | +22s 회귀, chain pruning re-walk overhead |
| [fc56268] | mark_dead_in_parl이 point_bucket 셀에서도 dead 점 물리 제거 | (측정 안 함) | |
| [67cf6d7] | **incremental midpoint matching** — mid_groups 스케일 간 유지, 새 페어×기존 페어만 검사 | **185,900** | **−235s, 큰 win** |

총 누적: **131s (v19) → 185s (v20 현재)**. v20은 v19보다 느리지만
hierarchical 구조를 갖춤. 다음 최적화 단계가 5s 목표를 위한 핵심.

## 단계 설명

### 1. Hierarchical multi-scale (1a50367)

**아이디어**: 한 번에 모든 페어를 검사하지 말고, 스케일 s ∈ {32, 64, 128, 256}별로
bbox ≤ s 페어만 enumerate. 작은 스케일에서 dense small parl 빨리 채택 →
dead pole 누적 → 큰 스케일에선 enumerate 가벼워짐.

- `pole_dead[i]` = parl 내부에 strict하게 들어간 극.
- `mid_groups`를 스케일마다 새로 빌드 (bbox + dead 필터).
- detect_phased_for_scale: phase greedy + SAT를 누적 accepted_parls 대상.

### 2. SAT containment override (bc9c898)

큰 사각형이 작은 lattice (이미 채택된 parl + expansions) 와 SAT-overlap 해도,
작은 것을 **완전히 포함**하면 accept. 작은 사각형들이 lattice를 채워서 큰 사각형이
못 들어가는 문제 해결.

- `parl_contains_parl(P, Q)` — Q의 4 코너가 모두 P 내부/경계에 있는지.
- `all_seen` pre-check 추가 (containment override가 자기자신도 포함하므로 중복 acceptance 방지).

### 3. Dynamic scale schedule + per-scale timing (b22a890)

`{32, 64, 128, 256}`이 hardcoded라 layout이 큰 경우 (5493극, layout_max_dim=5520)
거대 사각형 검출 못함. doubling으로 layout_max_dim 이상까지 자동 확장 →
{32, 64, 128, 256, 512, 1024, 2048, 4096, 8192}.

- 각 스케일별 시간 / candidates / dead_after / primary_after 측정.
- 첫 측정: **399.5s**. scale 2048+ 가 대부분의 시간 차지.

### 4. Per-scale instrumentation (5363899)

진단 강화: 각 스케일의 insert vs iterate 시간, pair_checks/angle_pass/chain_calls
delta 측정. 큰 스케일에서 어디 시간이 가는지 분리.

### 5. Chain pruning (8a47fce)

User 요청: chain의 intermediate가 accepted parl 내부 dead pole이면 chain 자체 reject.
SAT가 어차피 거절할 케이스를 chain validation 단계에서 더 빨리 차단.

- `_make_chain_inner`에서 chain build 후 intermediate 검사.
- cache hit path에도 lazy re-walk (스케일 간 dead 변화 처리).
- logic-preserving (SAT가 같은 결정을 더 늦게 내림).

### 6. Cell clamping (4e56e2a)

scale=8192에서 cell range가 layout 밖으로 펼쳐져 5K nil cell 룩업이
5493극마다 반복 → 1.4B 빈 lookup. Layout cell bbox로 clamp.

- `layout_cx_min/max`, `layout_cy_min/max` 한 번 계산.
- enumerate insert에서 cell 범위 layout 안으로 자름.

**측정 결과**: 421.6s (오히려 +22s). chain pruning의 cache hit re-walk가 큰
스케일에서 chain_calls 17M+에 누적되어 cell clamping의 이득을 상쇄.

### 7. Dead pole 셀 물리 제거 (fc56268)

User 지적: dead 마킹만 하고 cell에서 안 빼서 enumerate가 dead 극도 lookup.
`mark_dead_in_parl`이 dead 점을 해당 cell에서 `table.remove`로 빼냄.

- `lines` 버킷은 그대로 둠 (chain pruning logic이 dead intermediate 만나야 reject).
- 작은 win 추정 (수 초).

### 8. Incremental midpoint matching (67cf6d7) — **큰 win**

**문제**: 각 스케일이 mid_groups를 새로 빌드. 작은 bbox 페어가 매 스케일마다
재삽입되고 pair_checks 재실행. 4096 스케일은 2048에서 본 페어쌍의 대부분을 다시 봄.

| Scale | total pair_checks | 직전 스케일 누적 | 신규 추정 |
|---|---|---|---|
| 1024 | 26.5M | 5.5M | ~21M |
| 2048 | 91.7M | 26.5M | **+65M (대부분 중복)** |
| 4096 | 122.6M | 91.7M | +31M |
| 8192 | 127.5M | 122.6M | +5M |

**해결**: mid_groups를 스케일 loop 밖에서 유지. 각 스케일은 새 페어
(bbox in (prev_scale, scale]) 만 추가. iterate에서는 새 페어 × 모든 페어
조합만 검사.

- `group_old_sizes`, `touched_groups` 스케일마다 reset.
- iterate가 `for ti in touched_groups, for pi = old_size+1, glen, for pj = 1, pi-1`
  로 변경 → 각 페어쌍 정확히 한 번만 검사 (가장 작은 적합 스케일에서).

**측정 결과**: **185.9s** (−235s, 56% 감소).
- pair_checks: 375M → **130M** (v19 single-pass의 155M보다도 적음 — dead 필터 덕).
- iterate 시간이 큰 스케일에서 크게 감소.

## 패턴

성공한 변경의 공통점:
- **각 페어쌍을 한 번만 검사** (incremental). v18 perf 여행에서도 핵심 패턴.
- **검색 공간 자체 축소** (bbox filter + dead pole + cell clamp).
- **채택된 결과를 다음 검색에 반영** (dead pole 누적, scale 간 mid_groups 유지).

회귀한 변경의 공통점:
- **JIT trace에 영향**: chain pruning의 cache hit re-walk가 큰 iterate에서
  trace 효율 저하. v18 시절 side-bucket 카운터처럼 작은 logic 추가가
  큰 perf 영향.

## 측정 데이터 (현재 상태, 67cf6d7)

```
total: 185.9s
poles=5493 candidates=4405 primary=861 expansion=3544 iso=3 mst_cand=24470

Per-scale (incremental):
  scale=32   cand=6861  primary=575  expansion=2887  dead=190  time=1.18s
    pair_checks=25K     angle_pass=10K    chain_calls=31K    insert=81ms   iterate=126ms
  scale=64   cand=4995  primary=722  expansion=3333  dead=268  time=1.70s
    pair_checks=129K    angle_pass=28K    chain_calls=51K    insert=179ms  iterate=457ms
  scale=128  cand=4778  primary=785  expansion=3462  dead=336  time=2.63s
    pair_checks=379K    angle_pass=66K    chain_calls=97K    insert=456ms  iterate=476ms
  scale=256  cand=7442  primary=815  expansion=3525  dead=336  time=5.28s
    pair_checks=1.0M    angle_pass=125K   chain_calls=169K   insert=1.21s  iterate=1.37s
  scale=512  cand=10022 primary=849  expansion=3540  dead=336  time=11.22s
    pair_checks=3.5M    angle_pass=389K   chain_calls=459K   insert=3.30s  iterate=3.64s
  scale=1024 cand=12430 primary=855  expansion=3543  dead=417  time=37.51s
    pair_checks=21.6M   angle_pass=4.2M   chain_calls=5.1M   insert=7.58s  iterate=23.41s
  scale=2048 cand=14050 primary=859  expansion=3544  dead=417  time=79.61s
    pair_checks=67.1M   angle_pass=10.5M  chain_calls=11.8M  insert=14.42s iterate=61.31s
  scale=4096 cand=4055  primary=861  expansion=3544  dead=417  time=36.24s
    pair_checks=30.9M   angle_pass=763K   chain_calls=848K   insert=10.62s iterate=23.22s
  scale=8192 cand=0     primary=861  expansion=3544  dead=417  time=9.77s
    pair_checks=4.95M   angle_pass=3.8K   chain_calls=4.8K   insert=6.01s  iterate=3.76s

Total: pair_checks=130M  angle_pass=16.1M  chain_calls=18.5M  cache_hits=16.9M misses=1.66M
```

**남은 병목**:
- scale 2048: 80s (43% of total).
- scale 4096: 36s (19%). 새 primary 단 2개.
- scale 8192: 10s, cand=0 (낭비).
- iterate가 insert보다 2-3x 큼.

## 다음 작업 (planned)

5s 목표를 위한 단계별 계획. 작은 win부터 큰 변화까지.

### Plan A. 작은 정리 (예상 5-15s 절약)

- [A1] **scale 8192 (cand=0인 마지막 scale) skip**: 직전 스케일의 cand가 0이면
  다음 스케일에 새 candidate 나올 가능성 매우 낮음. 또는: scale s가 layout_max_dim
  이상이면 한 번만 돌리고 그 이후는 skip.
- [A2] **detect_phased_for_scale 호출 전 candidates 빈 체크**: 이미 빈 케이스
  early return 되지만 enumerate 자체 비용은 발생. 신규 페어 0이면 enumerate의
  iterate 단계 skip.

### Plan B. Length-ratio 필터 (예상 10-30s 절약, 위험)

평행사변형 규칙으로부터 도출: max(D1², D2²) ≤ 3 · min(D1², D2²) 가 필요조건.

- [B1] iterate에서 la²/lb²/dot 계산 전에 D1², D2² 계산하고 ratio 검사. 거절시
  sides 계산 skip. 거절률이 80% 이상이어야 net win이라 layout 의존적.

### Plan C. v1.21 side-search 알고리즘 (예상 50-150s 절약, 큰 작업)

User가 제안한 패러다임 변경: midpoint matching → side matching.

**핵심 통찰**: 페어 (i, j) 길이 L이 평행사변형 한 변일 때, 다른 평행 변의
오프셋 (a, b)는 `|a| ≤ L/2 AND b² ≥ 2L|a| - a²` 라는 좁은 영역 (lens 모양)
으로 제한됨.

- [C1] `algo-v21.lua` 신규 파일. v20 setup/iso/edge 코드 재사용.
- [C2] `enumerate_parls`를 phase-aware side-search로 교체:
  - Phase 1 (axis-rect): 수평/수직 페어, 평행 변 직접 lookup.
  - Phase 2/3 (sheared, one-axis): 1D 매개변수 탐색.
  - Phase 4 (general): 회전된 frame에서 탐색.
- [C3] Perimeter-greedy 정렬: 페어 길이 작은 것부터, 각 페어마다 partner 길이
  작은 것부터 처리. 이미 채택된 parl과 SAT 통과시 즉시 expand + dead 마킹.
- [C4] 중복 방지: 각 parl은 4개 변 중 1개만 canonical로 처리 (예: β 방향 한쪽만).

**위험**:
- Phase별 회전/탐색 코드가 길고 복잡.
- Output 동등성 보장 어려움 (phase greedy 순서가 달라짐).
- Dense regular grid에선 partner candidate가 v20만큼 많아 win 작을 수 있음.

### Plan D. 격자 패턴 직접 인식 (예상 100s+ 절약, 큰 작업)

5493극처럼 regular spacing이면 lattice 구조를 직접 인식해서 일반 enumerate
를 skip하는 fast path.

- [D1] Pre-detection: 인접 극 사이 거리 분포 분석. Dominant spacing이 있으면
  lattice 후보.
- [D2] Lattice 검증: 4-neighbor 구조가 일정한 영역 식별.
- [D3] Lattice fast path: enumerate 없이 직접 axis-aligned parl + expansion 채택.
  이후 일반 알고리즘으로 boundary와 irregular 영역 처리.

5s 목표에 가장 가까운 옵션. 다만 일반성 손실 — Factorio 사용자의 다양한
layout 중 lattice 아닌 케이스 처리 필요.

## 측정 방법

게임 안 selection 영역에서 v20 적용. dev_mode print가:
- total / scale_total / iso / phase별 시간.
- 스케일별 cand, primary, expansion, dead, insert/iterate 시간, pair_checks/angle_pass/chain_calls.
- candidates / primary / expansion 누적, edges_wanted 등.

이 출력으로 어디서 시간 쓰는지 / logic 보존됐는지 (primary, edges_wanted 동일성)
검증.

[1e32596]: ../../../commit/1e32596
[1a50367]: ../../../commit/1a50367
[bc9c898]: ../../../commit/bc9c898
[b22a890]: ../../../commit/b22a890
[5363899]: ../../../commit/5363899
[8a47fce]: ../../../commit/8a47fce
[4e56e2a]: ../../../commit/4e56e2a
[fc56268]: ../../../commit/fc56268
[67cf6d7]: ../../../commit/67cf6d7
