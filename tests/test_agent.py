"""Exercise the real pinned agent client through a mocked Responses transport."""

import json

import httpx
import numpy as np
from inspect_robots import eval
from inspect_robots.approver import DeltaLimitApprover
from inspect_robots_agent.policy import LLMAgentPolicy

from inspect_robots_b601.benchmarks import joint1_nudge


def response(call_id, name, arguments):
    return httpx.Response(
        200,
        json={
            "status": "completed",
            "output": [
                {
                    "type": "function_call",
                    "call_id": call_id,
                    "name": name,
                    "arguments": json.dumps(arguments),
                }
            ],
        },
    )


def test_agent_tool_call_round_trip_preserves_b601_units_and_measured_success(fake_ros, tmp_path):
    requests = []

    def serve(request):
        payload = json.loads(request.content)
        requests.append(payload)
        assert payload["model"] == "b601-test-model"
        assert str(request.url).endswith("/responses")
        tools = {tool["name"]: tool for tool in payload["tools"]}
        assert (
            "joint1, joint2, joint3, joint4, joint5, joint6"
            in tools["move_joints"]["parameters"]["properties"]["targets"]["description"]
        )
        if len(requests) == 1:
            return response(
                "move",
                "move_joints",
                {
                    "targets": {"joint1": 0.0015},
                    "note": "Fake measured-radian nudge; hold other joints",
                },
            )
        return response(
            "finish",
            "done",
            {"summary": "Fake nudge complete", "hindsight": "Check measured scoring"},
        )

    transport = httpx.MockTransport(serve)
    policy = LLMAgentPolicy(
        model="openai/b601-test-model",
        wire="responses",
        transport=transport,
        env={"OPENAI_API_KEY": "unit-test-not-a-real-key"},
        max_speed_frac=0.0004,
        max_llm_calls=3,
        max_retries=1,
    )
    try:
        (log,) = eval(
            joint1_nudge(max_steps=6),
            policy,
            fake_ros,
            log_dir=str(tmp_path),
            approver=DeltaLimitApprover(fake_ros.info.action_space, max_delta=0.002),
        )
        assert log.status == "success"
        assert log.results.metrics["b601_measured_goal"] == 1.0
        assert len(requests) == 2
        np.testing.assert_allclose(fake_ros._session.positions, [0.0015, 1, 1, 0, 0, 0])
        assert policy.transcript()
    finally:
        transport.close()


def test_agent_done_does_not_create_a_false_task_success(fake_ros, tmp_path):
    transport = httpx.MockTransport(
        lambda _: response(
            "finish",
            "done",
            {"summary": "Claimed success without moving", "hindsight": "Synthetic test"},
        )
    )
    policy = LLMAgentPolicy(
        model="openai/b601-test-model",
        wire="responses",
        transport=transport,
        env={"OPENAI_API_KEY": "unit-test-not-a-real-key"},
        max_llm_calls=1,
        max_retries=1,
    )
    try:
        (log,) = eval(joint1_nudge(), policy, fake_ros, log_dir=str(tmp_path))
        assert log.status == "success"
        assert log.results.metrics["b601_measured_goal"] == 0.0
    finally:
        transport.close()
