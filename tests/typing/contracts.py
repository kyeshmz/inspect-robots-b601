"""Static-only checks against Inspect's public protocols; never connect to hardware."""

from pathlib import Path
from typing import TYPE_CHECKING

from inspect_robots import Action, Embodiment, Observation, Scene, StepResult, Task
from inspect_robots.policy import Policy
from inspect_robots.rollout import TrialRecord
from inspect_robots.scorer import Scorer
from typing_extensions import assert_type

from inspect_robots_b601 import B601Embodiment, MotorbridgeB601Embodiment
from inspect_robots_b601.benchmarks import JointGoalPolicy, MeasuredJointGoal, hold, joint1_nudge

if TYPE_CHECKING:

    def check_public_contracts(
        scene: Scene,
        action: Action,
        observation: Observation,
        record: TrialRecord,
        profile: Path,
    ) -> None:
        ros: Embodiment = B601Embodiment()
        direct: Embodiment = MotorbridgeB601Embodiment(profile=profile)
        scripted = JointGoalPolicy()
        scripted.bind(ros.info)
        policy: Policy = scripted
        scorer: Scorer = MeasuredJointGoal()
        tasks: tuple[Task, Task] = (hold(), joint1_nudge())
        assert_type(ros.reset(scene), Observation)
        assert_type(direct.step(action), StepResult)
        policy.reset(scene)
        policy.act(observation)
        scorer(record, tasks[0].scenes[0].target)
        ros.close()
        direct.close()
