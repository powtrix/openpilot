# Keep the personalized device branch picker focused without changing the
# updater's complete remote branch map. The active target remains visible on
# model-specific installs so those branches can continue updating normally.
PREFERRED_TARGET_BRANCHES = ("dkcarrot-wip", "carrot-wip", "carrot")


def visible_target_branches(available_branches: list[str], current_target: str = "") -> list[str]:
  available = set(available_branches)
  branches = [branch for branch in PREFERRED_TARGET_BRANCHES if branch in available]
  if current_target in available and current_target not in branches:
    branches.insert(0, current_target)
  return branches
