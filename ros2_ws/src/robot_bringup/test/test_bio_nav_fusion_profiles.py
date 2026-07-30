from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
GENERATOR_PATH = WORKSPACE_ROOT / "scripts" / "generate_bionav_fusion_profile.py"
DYNAMIC_PROFILE = (
    PACKAGE_ROOT.parent
    / "robot_navigation"
    / "config"
    / "nav2_dynamic_avoidance.yaml"
)

SPEC = spec_from_file_location("generate_bionav_fusion_profile", GENERATOR_PATH)
GENERATOR = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(GENERATOR)

MAP_SHA = "1" * 64
PLANNING_QUALIFICATION_SHA = "2" * 64
RISK_MODEL_SHA = "3" * 64
RISK_QUALIFICATION_SHA = "4" * 64


def _profile(variant):
    return GENERATOR.build_profile(
        DYNAMIC_PROFILE,
        variant=variant,
        module3_map_sha256=MAP_SHA,
        planning_qualification_sha256=PLANNING_QUALIFICATION_SHA,
        risk_model_sha256=RISK_MODEL_SHA,
        risk_qualification_sha256=RISK_QUALIFICATION_SHA,
    )


def _global_parameters(profile):
    return profile["global_costmap"]["global_costmap"]["ros__parameters"]


def test_planning_only_uses_custom_planner_without_cognitive_risk():
    profile = _profile("planning_only")

    assert "cognitive_risk_layer" not in _global_parameters(profile)["plugins"]
    planner = profile["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "bio_nav_fusion::BioNavGridBased"
    assert planner["planner_profile"] == "bio_nav_planning_only"
    assert planner["expected_module3_map_sha256"] == MAP_SHA
    assert (
        planner["expected_qualification_sha256"]
        == PLANNING_QUALIFICATION_SHA
    )


def test_risk_only_uses_stock_planner_and_identity_bound_global_layer():
    profile = _profile("risk_only")

    assert "planner_server" not in profile
    global_parameters = _global_parameters(profile)
    assert "cognitive_risk_layer" in global_parameters["plugins"]
    layer = global_parameters["cognitive_risk_layer"]
    assert layer["expected_risk_model_sha256"] == RISK_MODEL_SHA
    assert (
        layer["expected_qualification_sha256"]
        == RISK_QUALIFICATION_SHA
    )


def test_combined_enables_both_components_without_changing_local_costmap():
    baseline = GENERATOR.yaml.safe_load(
        DYNAMIC_PROFILE.read_text(encoding="utf-8")
    )
    profile = _profile("combined")

    assert "cognitive_risk_layer" in _global_parameters(profile)["plugins"]
    assert (
        profile["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
        == "bio_nav_fusion::BioNavGridBased"
    )
    assert profile["local_costmap"] == baseline["local_costmap"]


def test_enabled_components_reject_missing_or_malformed_identity():
    for variant in ("planning_only", "combined"):
        try:
            GENERATOR.build_profile(
                DYNAMIC_PROFILE,
                variant=variant,
                module3_map_sha256="",
                planning_qualification_sha256=PLANNING_QUALIFICATION_SHA,
                risk_model_sha256=RISK_MODEL_SHA,
                risk_qualification_sha256=RISK_QUALIFICATION_SHA,
            )
        except ValueError as error:
            assert "module3_map_sha256" in str(error)
        else:
            raise AssertionError("planning profile accepted an empty map identity")

    try:
        GENERATOR.build_profile(
            DYNAMIC_PROFILE,
            variant="risk_only",
            risk_model_sha256="not-a-sha",
            risk_qualification_sha256=RISK_QUALIFICATION_SHA,
        )
    except ValueError as error:
        assert "risk_model_sha256" in str(error)
    else:
        raise AssertionError("risk profile accepted a malformed model identity")
