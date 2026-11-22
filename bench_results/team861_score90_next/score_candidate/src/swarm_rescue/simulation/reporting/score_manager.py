from swarm_rescue.simulation.reporting.time_score import time_score_fraction


class ScoreManager:
    """
    Tool to compute the final score in a drone rescue simulation.

     It takes into account the number of drones, time
     limits, and the number of bombs to calculate the score based on
     rescue percentage, exploration score, health returned and time taken to
     rescue all the bombs.

    Attributes:
        number_drones (int): Number of drones in the map.
        max_timestep_limit (int): Max number of timesteps.
        max_walltime_limit (int): Max wall time in seconds.
        total_number_bombs (int): Number of bombs to dispose.
        w_disposal (float): Weight for rescue score.
        w_exploration (float): Weight for exploration score.
        w_score_health_returned (float): Weight for health return score.
        w_time (float): Weight for time score.
        time_best_ratio (float): Fraction of ``max_timestep_limit`` at (or
            before) which the time score is full (default 0.5). ``0`` removes
            the full-mark plateau, so every elapsed step costs score.
    """

    def __init__(self,
                 number_drones: int,
                 max_timestep_limit: int,
                 max_walltime_limit: int,
                 total_number_bombs: int,
                 exploration_threshold_for_time_bonus: float = 97.0,
                 time_best_ratio: float = 0.3):
        """
        Initialize the ScoreManager.

        Args:
            number_drones (int): Number of drones.
            max_timestep_limit (int): Max timesteps.
            max_walltime_limit (int): Max wall time in seconds.
            total_number_bombs (int): Number of bombs.
            exploration_threshold_for_time_bonus (float): Exploration rate the
                (informational) trajectory figure is measured against.
            time_best_ratio (float): Fraction of the timestep limit at or
                before which the time score is full, within [0, 1]. ``0``
                means no round can earn full marks and every step costs score.

        Raises:
            ValueError: If ``time_best_ratio`` is outside [0, 1].
        """

        ratio = float(time_best_ratio)
        if not 0.0 <= ratio <= 1.0:
            raise ValueError(
                f"time_best_ratio must be within [0, 1], got {time_best_ratio}"
            )

        # 'number_drones' is the number of drones that will be generated in the
        # map
        self.number_drones = number_drones

        # 'max_timestep_limit' is the number of timesteps after which the
        # session will end.
        self.max_timestep_limit = max_timestep_limit

        # 'max_walltime_limit' is the elapsed time (in seconds) after which the
        # session will end.
        self.max_walltime_limit = max_walltime_limit

        # 'time_best_ratio' is the fraction of 'max_timestep_limit' at (or
        # before) which the time score is full. Both teams read it from their
        # own YAML section and share the curve with the red team
        # (``place_scoring.time_best_ratio``).
        self.time_best_ratio = ratio

        # 'number_bombs' is the number of bombs that should
        # be retrieved by the drones.
        self.total_number_bombs = total_number_bombs

        # weight for the different parts of the score. The sum must be equal
        # to 1.
        self.w_disposal = 0.6
        self.w_exploration = 0.0
        self.w_score_health_returned = 0.2
        self.w_time = 0.2
        self.exploration_threshold_for_time_bonus = float(
            exploration_threshold_for_time_bonus
        )

    def compute_score(self, number_disposed_bombs: int, score_exploration: float,
                      score_health_returned: float, elapsed_timestep: int,
                      has_crashed: bool = False) -> tuple:
        """
        Compute the final score out of 100.

        The time score is awarded only to a round that disposed every bomb; a
        round that leaves bombs behind earns 0 on time whatever ended it. It
        may run to the timestep limit (``elapsed_timestep`` at the limit) or,
        on a host slow enough to reach the walltime limit first, stop below the
        limit; the completion gate holds both at 0, so the time score never
        rewards an incomplete round and never depends on host speed.

        A completed round uses the same curve as the red team
        (``time_score.time_score_fraction``): at or before
        ``time_best_ratio`` of the timestep limit the score is full (out of
        100), it drops to 0 at the limit and is linear in between; a crashed
        round earns 0 on time.

        Args:
            number_disposed_bombs (int): Number of disposed bombs.
            score_exploration (float): Exploration score.
            score_health_returned (float): Health return score.
            elapsed_timestep (int): Timesteps elapsed at the end of the round.
            has_crashed (bool): Whether the round crashed (zeroes the time
                score, like the red team).

        Returns:
            tuple: (score, percentage_disposed, score_timestep)
        """
        if self.total_number_bombs > 0:
            percentage_disposed = (number_disposed_bombs /
                                 self.total_number_bombs * 100.0)
        else:
            percentage_disposed = 100.0

        all_bombs_disposed = (
            self.total_number_bombs <= 0
            or number_disposed_bombs >= self.total_number_bombs
        )
        if all_bombs_disposed:
            score_timestep = self._compute_time_score(elapsed_timestep,
                                                      has_crashed)
        else:
            score_timestep = 0.0

        score = self.w_disposal * percentage_disposed + \
                self.w_exploration * score_exploration + \
                self.w_score_health_returned * score_health_returned + \
                self.w_time * score_timestep

        return score, percentage_disposed, score_timestep

    def _compute_time_score(self, elapsed_timestep: int,
                            has_crashed: bool = False) -> float:
        """
        Time score out of 100 for a round that disposed every bomb, mirroring
        the red team's time score.

        ``score_timestep = time_score_fraction(elapsed, max_timestep_limit,
        time_best_ratio) * 100``. A crashed round earns 0, same as the red
        team. The completion gate lives in ``compute_score``, which is the only
        caller and applies this curve only to a round with no bomb left.
        """
        if has_crashed:
            return 0.0

        fraction = time_score_fraction(
            float(elapsed_timestep),
            float(self.max_timestep_limit),
            self.time_best_ratio,
        )
        return 100.0 * fraction
