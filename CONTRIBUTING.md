# Contributing

Use issue-linked branch names, for example `fix-linker-bot/linkerhand-telop-python#23`.

For local checks:

```bash
cd src/linkerhand_retarget
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
```

For packaging checks:

```bash
colcon build --symlink-install
```

