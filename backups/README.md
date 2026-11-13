# Code backups

The current source mirror was captured on 2026-11-14 from Jetson ROS workspace
source and the active PC dashboard. The repository root uses the existing ROS
package layout. `firmware/kabot_actuator` contains the verified uploaded Arduino
source with both RC input centers and ESC neutral outputs at 1500 us, RC deadband
+/-50 us, RC pins D6/D3/D4, and ESC pins D9/D10.

## Historical source snapshots

| Folder | Captured content |
| --- | --- |
| `20261110-2200/code` | Earlier Jetson core source backup |
| `isv-water-trials-dashboard-backup-20261111-010444` | Earlier outdoor dashboard source |
| `dashboard-live-status-backup-20261111-034558` | Earlier sensor dashboard source |
| `dashboard-rocket-priority-backup-20261111-154311` | Historical connection and startup source |
| `arduino-current-backup-20261113` | Earlier Arduino source archives |
| `arduino-rc-right-20261113` | Earlier right-channel firmware source snapshots |
| `arduino-rc-right-rollback-20261113` | Earlier firmware rollback source snapshot |
| `arduino-spare-20261113` | Spare Arduino board source backup |
| `20261114-010132-jetson-code` | Source extracted from both full Jetson backup archives, including additional projects and deployed dashboard copies |
| `20261114-021354-arduino` | RC and ESC neutral 1500 us source snapshot |

`PUBLIC_CODE_MANIFEST_20261114.json` records SHA-256 hashes for the published
source files and identifies their capture groups. Historical snapshots retain
their original versions and are separate from the current root source mirror.

## Public backup scope

This backup includes source code, build definitions, package metadata and
licenses. Saved GPS routes, SLAM maps, runtime state, raw sensor recordings,
logs, model weights, compiled images, toolchains, machine configuration and
credentials are retained only in the original local backups.

Some source files embed recorded coordinates. The public copies use empty
waypoint tuples and GPS mark arrays, or `None` mission coordinate placeholders.
`PUBLIC_REDACTIONS_20261114.json` lists these files. Supply local mission
coordinates before using those templates. Synthetic test coordinates and source
already present in the public repository are preserved. Local source and the
original backup archives retain the original coordinate data.

Verification covers capture consistency, published-file hashes, secret-pattern
inspection and syntax checks of the coordinate-redacted source. Firmware build,
upload and flash-readback verification were completed for the captured current
Arduino image before this backup. Historical snapshots include their original
tests and are not represented as passing the current test suite.
