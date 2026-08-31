# Pure localization contract

This package contains reusable runtime checks for localization graph
ownership. `tf_ownership_probe.py` verifies the exact `/tf` publisher endpoint
multiset, each owner's configured parent/child frames, and the observed dynamic
TF edges.

The package does not start a localizer and does not depend on Autoware. Its
registered system test launches every supported repository profile in an
isolated ROS domain and checks that each dynamic edge has exactly one owner.

```bash
ros2 run pure_localization_contract tf_ownership_probe.py --help
```
