# Published evaluation assets

This directory contains the reviewed plots, normalized metrics, and README demo
used by the public documentation:

- `autoware_lsim_hesai_course_2`: a representative RViz poster used by the
  repository README.
- `velodyne_32line_external_imu`: LiDAR/IMU-only results for an external IMU.
- `livox_mid360_internal_imu`: LiDAR/IMU-only results for an integrated IMU.
- `hesai_32line_imu_rtk_gnss_course_2`: LiDAR/IMU/GNSS local and global error
  results.

The README links to a GitHub-hosted RViz replay beside the corresponding Hesai
evaluation plots. The replay is visualization-only: it is not accuracy,
performance, or passing-run evidence. It is retained because it gives readers a
compact view of the localization output. The reviewed WebM has no audio,
capture date, or embedded capture metadata. The JSON files contain only the
metrics needed to interpret the evaluation plots. Source bags, raw logs,
sample-level CSV files, host paths, private run identifiers, and generated
result directories are intentionally not published.

`manifest.json` records the byte size and SHA-256 digest of every published
artifact. Verify a clean checkout with:

```bash
python3 tools/check_publication_assets.py
```

The check also rejects unlisted files, assets larger than 10 MiB, unsupported
file types, personal absolute paths, and known private provenance fields in
text assets. When an artifact is deliberately replaced, review the new content
and update its size and digest in the manifest.

See [the evaluation overview](../README.md) and
[the common methodology](../methodology.md) for interpretation. GLIM is a
correlated LiDAR/IMU pseudo-reference, not independent ground truth.
