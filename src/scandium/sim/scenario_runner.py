"""
Scenario runner for Scandium testing.

Executes predefined test scenarios and collects results.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional
import time

from scandium.utils.io import load_yaml
from scandium.logging.setup import get_logger

logger = get_logger(__name__)


@dataclass
class ScenarioResult:
    """
    Result of a scenario execution.

    Attributes:
        scenario_id: Scenario identifier.
        passed: Whether scenario passed.
        duration_s: Execution duration in seconds.
        metrics: Collected metrics.
        errors: List of errors encountered.
        logs: Scenario logs.
    """

    scenario_id: str
    passed: bool
    duration_s: float
    metrics: dict[str, Any] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)


@dataclass
class ScenarioStep:
    """
    Single step in a scenario.

    Attributes:
        name: Step name.
        action: Action to perform.
        params: Action parameters.
        expected: Expected outcome.
        timeout_s: Step timeout.
    """

    name: str
    action: str
    params: dict[str, Any] = field(default_factory=dict)
    expected: dict[str, Any] = field(default_factory=dict)
    timeout_s: float = 30.0


@dataclass
class Scenario:
    """
    Test scenario definition.

    Attributes:
        id: Scenario identifier.
        name: Human-readable name.
        description: Scenario description.
        setup: Setup configuration.
        steps: List of scenario steps.
        teardown: Teardown configuration.
        pass_criteria: Pass/fail criteria.
    """

    id: str
    name: str
    description: str = ""
    setup: dict[str, Any] = field(default_factory=dict)
    steps: list[ScenarioStep] = field(default_factory=list)
    teardown: dict[str, Any] = field(default_factory=dict)
    pass_criteria: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: Path) -> "Scenario":
        """Load scenario from YAML file."""
        data = load_yaml(path)

        steps = []
        for step_data in data.get("steps", []):
            steps.append(
                ScenarioStep(
                    name=step_data.get("name", ""),
                    action=step_data.get("action", ""),
                    params=step_data.get("params", {}),
                    expected=step_data.get("expected", {}),
                    timeout_s=step_data.get("timeout_s", 30.0),
                )
            )

        return cls(
            id=data.get("id", path.stem),
            name=data.get("name", path.stem),
            description=data.get("description", ""),
            setup=data.get("setup", {}),
            steps=steps,
            teardown=data.get("teardown", {}),
            pass_criteria=data.get("pass_criteria", {}),
        )


class ScenarioRunner:
    """
    Executes test scenarios and collects results.
    """

    def __init__(self, scenarios_dir: Optional[Path] = None) -> None:
        """
        Initialize scenario runner.

        Args:
            scenarios_dir: Directory containing scenario YAML files.
        """
        self._scenarios_dir = scenarios_dir or Path("configs/scenarios")
        self._results: list[ScenarioResult] = []

    def load_scenario(self, scenario_id: str) -> Optional[Scenario]:
        """
        Load a scenario by ID.

        Args:
            scenario_id: Scenario identifier.

        Returns:
            Scenario or None if not found.
        """
        scenario_path = self._scenarios_dir / f"{scenario_id}.yaml"

        if not scenario_path.exists():
            logger.error("scenario_not_found", id=scenario_id)
            return None

        try:
            return Scenario.from_yaml(scenario_path)
        except Exception as e:
            logger.error("scenario_load_failed", id=scenario_id, error=str(e))
            return None

    def run(self, scenario: Scenario) -> ScenarioResult:
        """
        Execute a scenario.

        Args:
            scenario: Scenario to execute.

        Returns:
            ScenarioResult with execution details.
        """
        logger.info("scenario_start", id=scenario.id, name=scenario.name)

        start_time = time.time()
        errors: list[str] = []
        logs: list[str] = []
        metrics: dict[str, Any] = {}
        passed = True

        try:
            # Setup phase
            logs.append(f"Setup: {scenario.setup}")
            self._execute_setup(scenario.setup)

            # Execute steps
            for step in scenario.steps:
                try:
                    step_result = self._execute_step(step)
                    logs.append(f"Step '{step.name}': {step_result}")

                    if not step_result.get("success", False):
                        errors.append(f"Step '{step.name}' failed")
                        passed = False

                except Exception as e:
                    errors.append(f"Step '{step.name}' error: {str(e)}")
                    passed = False

            # Check pass criteria
            if passed:
                passed = self._check_criteria(scenario.pass_criteria, metrics)

            # Teardown
            logs.append(f"Teardown: {scenario.teardown}")
            self._execute_teardown(scenario.teardown)

        except Exception as e:
            errors.append(f"Scenario error: {str(e)}")
            passed = False

        duration = time.time() - start_time

        result = ScenarioResult(
            scenario_id=scenario.id,
            passed=passed,
            duration_s=duration,
            metrics=metrics,
            errors=errors,
            logs=logs,
        )

        self._results.append(result)

        logger.info(
            "scenario_complete",
            id=scenario.id,
            passed=passed,
            duration_s=round(duration, 2),
        )

        return result

    def _execute_setup(self, setup: dict[str, Any]) -> None:
        """Execute scenario setup."""
        # Placeholder for setup actions
        pass

    def _execute_step(self, step: ScenarioStep) -> dict[str, Any]:
        """
        Execute a single step.

        Returns:
            Step result dictionary.
        """
        logger.debug("step_execute", name=step.name, action=step.action)

        # Placeholder for step execution logic
        # In real implementation, this would dispatch to action handlers

        return {"success": True, "action": step.action}

    def _execute_teardown(self, teardown: dict[str, Any]) -> None:
        """Execute scenario teardown."""
        # Placeholder for teardown actions
        pass

    def _check_criteria(
        self,
        criteria: dict[str, Any],
        metrics: dict[str, Any],
    ) -> bool:
        """Check if pass criteria are met."""
        for key, expected in criteria.items():
            if key not in metrics:
                continue
            actual = metrics[key]
            if actual != expected:
                return False
        return True

    def run_by_id(self, scenario_id: str) -> Optional[ScenarioResult]:
        """
        Load and run a scenario by ID.

        Args:
            scenario_id: Scenario identifier.

        Returns:
            ScenarioResult or None if scenario not found.
        """
        scenario = self.load_scenario(scenario_id)
        if scenario is None:
            return None
        return self.run(scenario)

    def get_summary(self) -> dict[str, Any]:
        """Get summary of all executed scenarios."""
        total = len(self._results)
        passed = sum(1 for r in self._results if r.passed)

        return {
            "total": total,
            "passed": passed,
            "failed": total - passed,
            "pass_rate": passed / total if total > 0 else 0.0,
            "scenarios": [
                {
                    "id": r.scenario_id,
                    "passed": r.passed,
                    "duration_s": round(r.duration_s, 2),
                }
                for r in self._results
            ],
        }

    def clear_results(self) -> None:
        """Clear collected results."""
        self._results.clear()
