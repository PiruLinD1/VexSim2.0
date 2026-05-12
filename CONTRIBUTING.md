# Contributing

Thanks for helping improve VexSim 2.0.

The repository is intended to be open to outside contributions, but changes should be reviewed and tested before they land in `main`.

## Workflow

1. Fork the repository.
2. Create a branch for your change.
3. Keep the change focused and easy to review.
4. Run the checks locally.
5. Open a pull request using the template.

## Local Checks

```bash
python -m pip install -r requirements.txt
python -m py_compile main.py config.py goals.py input_manager.py leaderboard.py loaders.py physics.py render.py render3d.py render_gpu.py robot.py team_config.py triball_mesh.py vec2.py
```

## Pull Request Expectations

- Explain what changed and why.
- Keep unrelated refactors out of gameplay/physics fixes.
- Include screenshots or short clips for visual changes when possible.
- Do not commit generated files, local caches, personal leaderboard data, or controller profiles.
- Make sure CI passes before requesting review.

## Code Quality

- Prefer simple, readable Python over clever shortcuts.
- Keep physics and rendering changes small enough to test in-game.
- Preserve existing controls, scoring rules, and file formats unless the PR clearly explains the migration.
- Add comments only where the behavior would otherwise be difficult to understand.

## Review Policy

Contributors can propose anything through pull requests. Maintainers should require passing CI and at least one review before merging to `main`.
