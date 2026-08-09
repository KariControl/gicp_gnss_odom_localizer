# Security policy

This project processes untrusted sensor messages and configuration files. Report vulnerabilities privately to the repository maintainers rather than opening a public issue when disclosure could enable denial of service, memory corruption, unsafe frame interpretation, or localization discontinuities.

Include the affected commit, ROS distribution, reproduction steps, input data characteristics, and impact. Do not include production credentials or confidential rosbag data.

## Operational scope

This software is not safety-certified. Deployments must independently supervise localization freshness, covariance, frame consistency, jumps, and estimator health, and must transition to a safe state when requirements are not met.
