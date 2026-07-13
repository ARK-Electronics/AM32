"""Candidate scoring, noise-floor tie-breaks, and hill-climb search.

Softer multi-repeat DQ policy: when a candidate has more than one trial
entry, a single fluke disqualification no longer kills the whole candidate.
Score and secondary metrics are taken from the clean (non-DQ) entries only;
the candidate is eliminated only when every entry is disqualified (or none
have a usable score).
"""
from __future__ import annotations

import math
import statistics
from typing import Callable, Optional


def clean_entries(entries: list[dict]) -> list[dict]:
    """Entries that were not disqualified (and not discarded)."""
    return [e for e in entries
            if not e.get("disqualified") and not e.get("discarded")]


def entry_score(e: dict) -> Optional[float]:
    """Prefer anchor-normalized score; fall back to raw."""
    if e.get("score_norm") is not None:
        return e["score_norm"]
    return e.get("score_raw")


def candidate_metric(entries: list[dict]) -> Optional[float]:
    """Median score across clean entries, or None if the candidate is dead.

    Single-entry candidates still die on any DQ (only one sample, nothing
    to salvage). Multi-entry candidates keep scoring from remaining clean
    legs so one fluke demag does not erase a real signal.
    """
    clean = clean_entries(entries)
    if not clean:
        return None
    vals = [v for e in clean if (v := entry_score(e)) is not None]
    return statistics.median(vals) if vals else None


def median_of(entries: list[dict], key: str) -> Optional[float]:
    vals = [e[key] for e in entries if e.get(key) is not None]
    return statistics.median(vals) if vals else None


def normalize(entries: list[dict], anchors: list[tuple[int, float]],
              positions: dict[int, int]) -> None:
    """Set score_norm on each entry: raw scaled by the first anchor over
    the linear interpolation between surrounding anchors (cancels pack
    drift). No/one usable anchor -> raw is kept as-is."""
    usable = [(p, s) for p, s in anchors if s is not None and s > 0]
    for e in entries:
        raw = e.get("score_raw")
        if raw is None:
            continue
        pos = positions[e["index"]]
        e["score_norm"] = round(raw * drift_factor(usable, pos), 5)


def drift_factor(anchors: list[tuple[int, float]], pos: int) -> float:
    if not anchors:
        return 1.0
    ref = anchors[0][1]
    if pos <= anchors[0][0]:
        interp = anchors[0][1]
    elif pos >= anchors[-1][0]:
        interp = anchors[-1][1]
    else:
        interp = anchors[0][1]
        for (p0, s0), (p1, s1) in zip(anchors, anchors[1:]):
            if p0 <= pos <= p1:
                frac = (pos - p0) / max(1, (p1 - p0))
                interp = s0 + (s1 - s0) * frac
                break
    return ref / interp if interp > 0 else 1.0


def _score_candidates(cands: list[dict]) -> list[dict]:
    """Attach score/jitter/fet on each cand; return the scored subset."""
    scored = []
    for c in cands:
        score = candidate_metric(c["entries"])
        if score is None:
            c["score"] = None
            continue
        clean = clean_entries(c["entries"])
        c["score"] = score
        c["jitter"] = median_of(clean, "jitter_pct")
        c["fet"] = median_of(clean, "fet_temp_c")
        scored.append(c)
    return scored


def efficiency_argmax(cands: list[dict]) -> dict | None:
    """Pure efficiency winner (highest candidate_metric), ignoring tie-breaks."""
    scored = _score_candidates(cands)
    if not scored:
        return None
    return max(scored, key=lambda c: (c["score"], -c["order"]))


def pick_winner(cands: list[dict], *, noise_floor_pct: float,
                distance_fn: Callable[[dict], float]) -> dict | None:
    """cands: [{overrides, entries, order}] -> winner cand or None.

    Within ``noise_floor_pct`` of the best score the tie breaks toward lower
    jitter, then lower FET temp, then the settings closest to default.
    Multi-repeat fluke DQs are ignored (see :func:`candidate_metric`).
    """
    scored = _score_candidates(cands)
    if not scored:
        return None
    best = max(c["score"] for c in scored)
    floor = best * (1.0 - noise_floor_pct / 100.0)
    tied = [c for c in scored if c["score"] >= floor]
    tied.sort(key=lambda c: (
        c["jitter"] if c.get("jitter") is not None else math.inf,
        c["fet"] if c.get("fet") is not None else math.inf,
        distance_fn(c["overrides"]),
        c["order"]))
    return tied[0]


def winner_reason(winner: dict | None, argmax: dict | None) -> str | None:
    """Why ``winner`` was chosen vs pure efficiency argmax."""
    if winner is None:
        return None
    if argmax is None:
        return "max_score"
    if winner is argmax or winner.get("overrides") == argmax.get("overrides"):
        return "max_score"
    # Inside noise floor: report the first differing secondary key.
    wj, aj = winner.get("jitter"), argmax.get("jitter")
    if wj is not None and aj is not None and wj < aj:
        return "noise_floor_tiebreak:jitter"
    wf, af = winner.get("fet"), argmax.get("fet")
    if wf is not None and af is not None and wf < af:
        return "noise_floor_tiebreak:fet_temp"
    return "noise_floor_tiebreak:closer_to_default"


def climb(ordered: list[int], start_val: int,
          test_value: Callable[[int], dict],
          *, score_fn: Callable[[dict], Optional[float]] | None = None
          ) -> None:
    """Hill-climb a sorted value list from the value nearest ``start_val``.

    Valid for unimodal responses (advance, pwm frequency): walk in the
    first improving direction and stop at the first non-improvement.

    ``score_fn`` defaults to :func:`candidate_metric` (prefers normalized
    scores on clean entries). Callers that re-normalize after each trial
    make climb direction match the final ranking basis.
    """
    if score_fn is None:
        score_fn = lambda c: candidate_metric(c["entries"])

    def better(a: dict, b: dict) -> bool:
        ra, rb = score_fn(a), score_fn(b)
        return rb is not None and (ra is None or rb > ra)

    i = min(range(len(ordered)), key=lambda k: abs(ordered[k] - start_val))
    cur = test_value(ordered[i])
    moved = False
    for direction in (1, -1):
        j = i + direction
        while 0 <= j < len(ordered):
            nxt = test_value(ordered[j])
            if not better(cur, nxt):
                break
            cur, moved = nxt, True
            j += direction
        if moved:
            break   # went uphill one way; the other side is downhill


def argmax_value(cands: list[dict]) -> Optional[int]:
    """Value of the best-scoring qualified sweep candidate.

    Uses the same multi-repeat-soft metric as pick_winner / climb so refine
    centers on the true ranking neighborhood, not a raw-score fluke.
    """
    best_v, best_s = None, None
    for c in cands:
        s = candidate_metric(c["entries"])
        if s is None:
            continue
        if best_s is None or s > best_s:
            best_v, best_s = c.get("value"), s
    return best_v
