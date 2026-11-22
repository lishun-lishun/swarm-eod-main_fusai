"""The time-score curve shared by the red and blue teams.

Both teams are scored on how much of the time limit a finished round used, with
the same shape: no penalty at or before ``best_ratio * limit``, zero at the
limit, linear in between.  The curve lives here once so the two scorers cannot
drift apart, and so the meaning of the ratio's end values is stated in a single
place:

* ``best_ratio == 1`` -- the full-mark plateau reaches the limit, so any round
  that ends before the limit scores full marks.
* ``best_ratio == 0`` -- the plateau has zero width: every step of elapsed time
  costs score, and only an instant finish would reach full marks.  A real round
  always needs at least the warm-up step, so the time score is always strictly
  below full marks and falls linearly to zero at the limit.
"""

from __future__ import annotations

from swarm_rescue.simulation.utils.utils import clamp


def time_score_fraction(
    elapsed: float,
    time_limit: float,
    best_ratio: float,
) -> float:
    """Score a round's elapsed time as a fraction in [0, 1].

    ``fraction = clamp((limit - elapsed) / (limit - best), 0, 1)`` with
    ``best = best_ratio * limit``.  A non-positive limit scores 0 (there is no
    time budget to measure against).

    Args:
        elapsed: Time used, in the same unit as ``time_limit``.
        time_limit: The round's time limit.
        best_ratio: Fraction of ``time_limit`` at or before which the score is
            full. Accepts the whole [0, 1] range, including 0 and 1.

    Returns:
        float: The time score, 0.0 through 1.0.
    """
    limit = float(time_limit)
    used = float(elapsed)
    ratio = float(best_ratio)

    if limit <= 0:
        return 0.0

    best = ratio * limit
    if best >= limit:
        # The full-mark plateau covers the whole limit (ratio == 1).
        return 1.0 if used <= best else 0.0

    # ``best`` may be 0, which is the intended "no plateau" case: the divisor is
    # the whole limit and the score falls from the first step onwards.
    fraction = (limit - used) / (limit - best)
    return float(clamp(fraction, 0.0, 1.0))
