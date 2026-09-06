from openpilot.selfdrive.ui.layouts.settings.software_helpers import visible_target_branches


def test_visible_target_branches_filters_and_orders_personalized_choices():
  available = [
    "feature/test",
    "carrot",
    "carrot-bmr_v6",
    "dkcarrot-wip",
    "carrot-wip",
  ]

  assert visible_target_branches(available, "dkcarrot-wip") == [
    "dkcarrot-wip",
    "carrot-wip",
    "carrot",
  ]


def test_visible_target_branches_keeps_an_existing_model_target():
  available = ["carrot", "carrot-bmr_v6", "dkcarrot-wip", "carrot-wip"]

  assert visible_target_branches(available, "carrot-bmr_v6") == [
    "carrot-bmr_v6",
    "dkcarrot-wip",
    "carrot-wip",
    "carrot",
  ]


def test_visible_target_branches_does_not_offer_missing_branches():
  assert visible_target_branches(["dkcarrot-wip", "feature/test"], "missing-target") == ["dkcarrot-wip"]
