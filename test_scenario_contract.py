import copy
import json
from pathlib import Path
import unittest

from scenario_contract import (
    SCENARIO_SCHEMA_VERSION,
    ScenarioContractError,
    validate_scenario_catalog,
)
from warehouse_astar_sim import (
    GOAL,
    GRID_HEIGHT,
    GRID_WIDTH,
    MAX_PLANNING_TIME,
    SCENARIO_CONFIG,
    SHELVES,
    START,
)

ROOT = Path(__file__).resolve().parent


class ScenarioContractTest(unittest.TestCase):
    def setUp(self):
        self.catalog = json.loads((ROOT / "scenarios.json").read_text(encoding="utf-8"))

    def test_checked_in_catalog_is_valid_and_drives_python_planner_constants(self):
        validate_scenario_catalog(self.catalog)
        warehouse = self.catalog["warehouse"]
        self.assertEqual(self.catalog["schemaVersion"], SCENARIO_SCHEMA_VERSION)
        self.assertEqual(GRID_WIDTH, warehouse["width"])
        self.assertEqual(GRID_HEIGHT, warehouse["height"])
        self.assertEqual(START, (warehouse["start"]["x"], warehouse["start"]["y"]))
        self.assertEqual(GOAL, (warehouse["goal"]["x"], warehouse["goal"]["y"]))
        self.assertEqual(MAX_PLANNING_TIME, warehouse["maxPlanningTime"])
        self.assertEqual(
            SHELVES,
            [
                (shelf["name"], shelf["x"], shelf["y"], shelf["width"], shelf["height"])
                for shelf in warehouse["shelves"]
            ],
        )
        self.assertEqual(SCENARIO_CONFIG, self.catalog)

    def test_schema_document_matches_runtime_version_and_fields(self):
        schema = json.loads(
            (ROOT / "scenarios.schema.json").read_text(encoding="utf-8")
        )
        self.assertEqual(
            schema["properties"]["schemaVersion"]["const"],
            SCENARIO_SCHEMA_VERSION,
        )
        self.assertEqual(
            set(schema["required"]),
            {
                "schemaVersion",
                "coordinateSystem",
                "warehouse",
                "defaultScenarioId",
                "scenarios",
            },
        )

    def assert_invalid(self, mutate, message):
        catalog = copy.deepcopy(self.catalog)
        mutate(catalog)
        with self.assertRaisesRegex(ScenarioContractError, message):
            validate_scenario_catalog(catalog)

    def test_unsupported_schema_version_fails_before_planning(self):
        self.assert_invalid(
            lambda catalog: catalog.update(schemaVersion=2),
            "unsupported scenario schemaVersion",
        )

    def test_wrong_coordinate_semantics_are_rejected(self):
        self.assert_invalid(
            lambda catalog: catalog["coordinateSystem"].update(yAxisDirection="up"),
            "coordinateSystem",
        )

    def test_boolean_grid_dimension_is_not_accepted_as_integer(self):
        self.assert_invalid(
            lambda catalog: catalog["warehouse"].update(width=True),
            "warehouse.width must be an integer",
        )

    def test_safe_integer_string_and_field_limits_are_enforced(self):
        self.assert_invalid(
            lambda catalog: catalog["warehouse"]["start"].update(x=(1 << 53)),
            "must be a safe integer",
        )
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][0].update(name="   "),
            "must be a non-empty string",
        )
        self.assert_invalid(
            lambda catalog: catalog["warehouse"].update(width=513),
            "warehouse.width must be at most 512",
        )
        self.assert_invalid(
            lambda catalog: catalog["warehouse"].update(maxPlanningTime=10_001),
            "warehouse.maxPlanningTime must be at most 10000",
        )

    def test_grid_state_space_and_joint_route_period_are_bounded(self):
        self.assert_invalid(
            lambda catalog: catalog["warehouse"].update(width=512, height=129),
            "grid must contain at most 65536 cells",
        )
        self.assert_invalid(
            lambda catalog: catalog["warehouse"].update(
                width=64, height=40, maxPlanningTime=10_000
            ),
            "exceed 1000000 time-expanded states",
        )

        def exceed_joint_period(catalog):
            catalog["scenarios"][0]["movingObstacles"] = [
                {"name": "stationary-a", "route": [{"x": 2, "y": 0}] * 317},
                {"name": "stationary-b", "route": [{"x": 3, "y": 0}] * 331},
            ]

        self.assert_invalid(exceed_joint_period, "joint mover period exceeds 100000 steps")

    def test_out_of_bounds_endpoints_and_shelves_are_rejected(self):
        self.assert_invalid(
            lambda catalog: catalog["warehouse"]["goal"].update(x=24),
            "warehouse.goal must be inside the grid",
        )
        self.assert_invalid(
            lambda catalog: catalog["warehouse"]["shelves"][0].update(x=23),
            "must fit entirely inside the grid",
        )

    def test_overlapping_shelves_and_blocked_endpoints_are_rejected(self):
        self.assert_invalid(
            lambda catalog: catalog["warehouse"]["shelves"][1].update(x=6, y=1),
            "overlaps another shelf",
        )
        self.assert_invalid(
            lambda catalog: catalog["warehouse"]["shelves"][0].update(
                x=1, y=1, width=1, height=1
            ),
            "warehouse.start is blocked by a shelf",
        )

    def test_invalid_scenario_ids_and_defaults_are_rejected(self):
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][1].update(id="Peak Traffic"),
            "must contain lowercase letters",
        )
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][0].update(
                id=catalog["scenarios"][0]["id"] + "\n"
            ),
            "must contain lowercase letters",
        )
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][1].update(id="standard"),
            "duplicate scenario id",
        )
        self.assert_invalid(
            lambda catalog: catalog.update(defaultScenarioId="missing"),
            "does not name a scenario",
        )

    def test_mover_routes_must_be_valid_and_consistent(self):
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][0]["movingObstacles"][0]["route"][0].update(x=24),
            "outside the grid",
        )
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][0]["movingObstacles"][0]["route"][0].update(x=5, y=0),
            "intersects a shelf",
        )
        self.assert_invalid(
            lambda catalog: catalog["scenarios"][0]["movingObstacles"][0].update(
                route=[{"x": 1, "y": 1}]
            ),
            "starts with mover .* on the robot",
        )

    def test_duplicate_names_and_periodic_mover_collisions_are_rejected(self):
        self.assert_invalid(
            lambda catalog: catalog["warehouse"]["shelves"][1].update(name="A"),
            "duplicate shelf name",
        )

        def overlap_movers(catalog):
            scenario = catalog["scenarios"][0]
            scenario["movingObstacles"][1]["route"] = copy.deepcopy(
                scenario["movingObstacles"][0]["route"]
            )

        self.assert_invalid(overlap_movers, "occupying the same cell")

        def edge_swap_movers(catalog):
            movers = catalog["scenarios"][0]["movingObstacles"]
            movers[0]["route"] = [{"x": 18, "y": 9}, {"x": 19, "y": 9}]
            movers[1]["route"] = [{"x": 19, "y": 9}, {"x": 18, "y": 9}]

        self.assert_invalid(edge_swap_movers, "head-on edge swap")

    def test_discontinuous_moving_route_and_extra_fields_are_rejected(self):
        def make_route_jump(catalog):
            catalog["scenarios"][0]["movingObstacles"][0]["route"] = [
                {"x": 7, "y": 9},
                {"x": 9, "y": 9},
            ]

        self.assert_invalid(make_route_jump, "move at most one cell")
        self.assert_invalid(
            lambda catalog: catalog.update(unversioned=True),
            "unexpected unversioned",
        )


if __name__ == "__main__":
    unittest.main()
