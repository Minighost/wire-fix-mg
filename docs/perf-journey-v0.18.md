# v0.18 성능 최적화 기록

5428극 layout 기준 wire_fix 알고리즘(`algo-v18.lua`)의 실행 시간을
**620s → 131s**(현재 측정값)로 줄이는 과정. 출력은 모두 bit-identical 유지
(`pair_checks=155M, candidates=107913, primary=407, expansion=2122` 등 카운터 동일).

알고리즘 로직(미드포인트 매칭 → 각도 60–120° 필터 → chain 검증 → phase greedy →
cascade 확장 → 이등변 보강 → MST closure)은 변경 없음. 코드 구조와 데이터
형태만 손댄 algorithm-equivalent 작업.

## 시작점

- 커밋: [`ea945a0`](../../../commit/ea945a0) — "Revert to v0.18 (drop segment-based experiment)"
- 5428극 selection: **620,130ms (≈620s)**.
- iterate가 594s를 차지. 그 안에서 `make_chain`의 cache miss 4.4M 건 중 long-side
  (side² > 4·reach²) 이 54M번 호출됨.
- `points_on_segment_strict`의 cell-scan이 long-side 미스에서 큰 bbox · 많은 셀을
  돌면서 사실상 O(n)에 가까운 일을 함.

## 단계별 결과

| 커밋 | 변경 | total (ms) | iterate (ms) | 비고 |
|---|---|---:|---:|---|
| `ea945a0` | 시작점 | 620,130 | 594,350 | |
| [`39e96fc`](../../../commit/39e96fc) | line bucket lookup | 235,742 | 203,533 | **−385s** |
| `3904e2a` | iterate를 3-stage로 분할 (측정용) | 725,850 | 701,491 | **+490s 회귀**, revert |
| `93cf0ff` | 위 revert | — | — | |
| `5c0db2e` | bit-based packing + 미드포인트 정수합 + lazy alloc | (실패) | — | 크래시: Factorio에 `bit` 없음 |
| [`9f8c633`](../../../commit/9f8c633) | bit packing 제거, 나머지 두 개 유지 | 179,678 | 150,289 | **−56s** |
| `a1c2bb9` | A/B 측정 스위치 (`measure_skip_chain`) | 100,048 (skip) | 76,866 | 측정용, 알고리즘 미변경 |
| `0d9cf94` | mid_groups 평탄화 + `pa_x/pa_y` hoist | 212,607 | 183,302 | **+33s 회귀**, revert |
| `feacde4` | 위 revert | — | — | |
| [`6664e06`](../../../commit/6664e06) | side-bucket 진단 카운터 제거 | 131,037 | 109,236 | **−48s** |
| `0fe21d6` | chain_cache row 사전할당 + `make_chain` wrapper 제거 | 140,679 | 116,767 | **+9s 회귀**, revert |
| [`b936247`](../../../commit/b936247) | 위 revert | — | — | |

총 누적: **620s → 131s, 약 4.7× 가속**.

## 단계 설명

### 1. Line bucket lookup (`39e96fc`)

**문제**: `points_on_segment_strict(lo, hi)`의 cell-scan이 long-side에서 segment
bbox 안의 모든 셀을 돌면서 각 극의 collinearity를 검사. 4.4M cache miss × ~135μs
≈ 590s.

**해결**: setup 단계에서 직선 키(canonical 방향 (ndx, ndy) + offset c) 기반 버킷을
미리 구축. 페어 reach 안에 있는 모든 페어를 line 버킷에 등록. 검색 시 O(1) 룩업
+ 정렬된 극 리스트에서 lo↔hi 사이를 슬라이스.

- 출력 동치성: chain의 일부가 되는 모든 극은 reach-near 인접 페어로 line 버킷에
  transitively 들어옴. lo 또는 hi가 line 밖이면 어차피 `make_chain`의 인접 갭
  검사에서 reject. Factorio의 grid-aligned 좌표라 정수 키 매칭과 부동소수점
  EPS_GEOM 매칭이 동일.

**효과**: 620s → 235s (**−385s**).

### 2. Iterate 3-stage 분할 측정 시도 (`3904e2a` → revert `93cf0ff`)

**의도**: iterate의 203s 내부 분포(extract+distinct, vector+angle, chain)를
실측하기 위해 inner loop를 3-pass로 재구조. 페어를 flat scratch buffer에
쓰고 다음 stage가 읽는 방식.

**결과**: iterate 203s → 701s (**3.5× 회귀**). 출력은 bit-identical이지만
중간 버퍼 R/W + JIT trace가 3개 작은 trace로 쪼개진 영향으로 추정.

**수확**: 회귀 상태에서 측정한 상대 비율 **distinct 22% : angle 44% : chain 33%**가
이후 최적화 우선순위 결정에 쓰임. 즉시 revert.

**교훈**: 측정용 코드 변경이 측정 대상 자체의 성능에 큰 영향을 줄 수 있음.
대규모 구조 변경보다 A/B 토글이 더 안전.

### 3. Lazy alloc + 미드포인트 정수합 (`9f8c633`, `5c0db2e`의 살아남은 부분)

#### 3-a. 미드포인트 키를 정수합으로

**문제**: insert 단계에서 `mfloor((pix + pxs[j]) * ROUND_SCALE + 0.5)` 식으로
미드포인트를 4-decimal 고정소수점 정수로 만듦. 페어당 mfloor 2회 + 부호 분기.

**해결**: `pixs[i] + pixs[j]` (정수 합) 자체를 키로 사용. 같은 미드포인트를
가진 두 페어는 정확히 같은 정수 합을 가지므로 키 동치. mfloor 4회와 부호
분기 제거.

#### 3-b. corners/chains lazy alloc

**문제**: 각도 필터를 통과한 49.5M개 후보마다 `corners_ccw = {...}`와
`chains = {}` 두 테이블을 즉시 할당. 그 중 ~83%가 chain 검증에서 떨어지는데
이미 할당한 테이블은 버려짐.

**해결**: 4개 chain의 corner index와 chain 결과를 로컬 변수에 보관, 4개 모두
성공한 시점에서만 `corners_ccw`와 `chains` 테이블 할당.

#### 3-c. 시도했다 폐기: bit-based pair unpacking

K를 2의 거듭제곱으로 padding하고 `mfloor(pa/K)`을 `bit.rshift(pa, K_shift)`로
대체하려 함. **Factorio mod sandbox에 `bit` 라이브러리 미노출**로 크래시.
`bit32`도 미확인. 폐기 후 `mfloor` 유지.

**효과 (3-a + 3-b)**: 235s → 179s (**−56s**).

### 4. A/B 측정 스위치 (`a1c2bb9`)

**의도**: iterate의 chain block(4 `make_chain` + 후보 등록)과 그 외(extract +
distinct + vector + 각도 필터)의 비율을 알고리즘 변경 없이 실측.

**구현**: `compute(points, reaches, opts)`에 `opts.skip_chain_block` 추가.
true면 각도 필터 통과 후 chain block을 통째로 skip. config-local.lua에서
토글.

**측정 결과 (5428극)**:
- 전체 실행 (chain block 포함): **iterate 150s** (`9f8c633` 시점, 179s total)
- chain block skip: **iterate 76s**
- 차이: **chain block ≈ 74s**, **나머지(distinct + vector + 각도) ≈ 76s**

거의 50/50. 두 절반 모두 추가 최적화 대상.

### 5. mid_groups 평탄화 + `pa_x/pa_y` hoist 시도 (`0d9cf94` → revert `feacde4`)

**의도**: 외곽 `for _, ry_row in pairs(mid_groups)` 중첩을 numeric for로 평탄화
하고, `pxs[a]/pys[a]` per-pi hoist.

**결과**: iterate 150s → 183s (**+33s 회귀**). 카운터는 동일하지만 LuaJIT의
trace 컴파일이 깨진 것으로 보임.

**교훈**: LuaJIT은 우리 직관과 다른 방식으로 trace를 만든다. "더 단순해 보이는"
구조가 항상 빠르지 않음. 측정 없이 적용하면 오히려 느려질 수 있음.

### 6. side-bucket 카운터 제거 (`6664e06`)

**문제**: `_make_chain_inner` 진입 시마다 chain_short/medium/long을 분류하는
진단용 카운터(side² 계산 + `pair_reach_sq` 호출 + 3-way 분기 + counter inc).
58M번 실행되는 핫패스에 있음. 알고리즘과 무관.

**해결**: 카운터 + 분류 로직 제거. 진단 정보는 git history에 남음.

**효과**: 179s → 131s (**−48s**, 추정 5s의 ~10배). iterate가 150s → 109s,
**insert까지 6s 감소** (직접 호출되지 않는데도) — LuaJIT trace 전체가 영향
받은 듯.

**교훈**: "값싸고 진단용으로 남겨도 되는" 카운터가 실제로는 trace 효율을
크게 떨어뜨릴 수 있음. 핫패스의 모든 상수가 아닌 일은 의심해볼 가치 있음.

### 7. chain_cache row 사전할당 + make_chain wrapper 제거 (`0fe21d6` → revert `b936247`)

**의도**:
- `_make_chain_inner` 진입마다 `local row = chain_cache[lo]; if row then ... else row = {}; chain_cache[lo] = row end`. 처음 lo가 보일 때마다 분기 + 빈 테이블 할당. 이걸 시작 시 `for i = 1, n do chain_cache[i] = {} end`로 사전할당해서 분기 제거.
- `make_chain → _make_chain_inner` 두 함수를 하나로 합쳐 canonicalize도 inline. 함수 frame 1개 절약.

**결과**: 131s → 140s (**+9s 회귀**). iterate +7s, insert +1.4s. 카운터 동일.

**교훈**: lazy lookup이 즉시 hit 되는 패턴(첫 lo 이후 모두 hit)이라 LuaJIT이 이미 최적화하고 있었던 듯. 사전할당으로 모든 lo가 빈 테이블 룩업을 추가로 통과하면서 trace가 오히려 무거워졌을 가능성. wrapper inline도 비슷한 영향 추정.

## 패턴

성공한 변경의 공통점:
- **데이터 구조 변경으로 본질적인 work 줄이기** (line bucket: O(cells × poles) → O(1) 룩업)
- **핫패스 일 줄이기** (lazy alloc, mfloor 제거)
- **트레이스를 단순하게** (side-bucket 카운터 제거, 함수 frame 줄이기)

회귀한 변경의 공통점:
- **iterate 구조 자체를 재배치** (3-stage split, mid_groups 평탄화) — LuaJIT trace를 깨뜨림
- 사람 직관에 맞는 "정리"가 LuaJIT 입장에선 더 비싸지는 경우

## 측정 방법

- 게임 안에서 같은 selection 영역 반복 적용. config-local.lua에 `dev_mode = true`로
  활성화된 dev_mode print가 stage별 timer + 카운터를 출력.
- 카운터 (`pair_checks`, `angle_pass`, `chain_calls`, `chain_success`, `candidates`,
  `primary`, `expansion`, `cache_hits`, `cache_misses`)가 변경 전후 일치하면
  출력 bit-identical.
- 알고리즘 동등 영역 안의 변경만. 의미적 변화 (input/output 변화) 가능성이
  있으면 별도 검토.
