# Scene authoring and reference package

`xgc2_scene_runtime` provides pure Python document validation, assembly,
generation, motion, persistence and undo/replay behavior. It is also the offline
oracle for the C++ Scene adapter's recorded corpus. It does not start a ROS Scene
publisher or choose a simulator. The live owner and finite preparation tool are
`xgc_scene_ros1_adapter/xgc2_scene_adapter` and `xgc2-scene-prepare`, managed by
the Core Scene workflow.

## Pure authoring semantics

`SceneStore` loads an authored scene and retains its initial poses. An optional
working file isolates experiment edits from the source asset: an existing copy
is loaded first, and the first accepted edit creates the copy. Edits,
undo/redo, save and reload use that target. External file changes reject edits
until an explicit reload. A successful live apply and a successful file save
remain distinct facts; saving does not commit a repository.

Obstacle poses use the scene frame; part poses are local to their obstacle.
Quaternions are XYZW and lengths are metres. Document geometry remains
parametric or explicitly indexed; geometry codec/math ownership is in the
common Math library, not a Python ROS projection.

A frozen store rejects geometry edits, undo/redo, save and reload explicitly.
Reads, playback and resync retain their existing semantics. Motion supports
hold, world-frame constant twist, ping-pong and XY-circle definitions. Playback
starts paused at scene time zero; current poses never overwrite authored
initial poses. Reset rewinds playback without resetting geometry or robots.

`generation.py` preserves the finite random-source contract and per-Run
working-file receipt. `python3 -m xgc2_scene_runtime.assembly` reads one JSON
request from stdin and returns `{sceneDocument, task}` without opening a ROS
node. The `integrations` modules are retained as offline preparation/protocol
references used by the golden recorder; they are not installed Scene launch
entries. `scene_command.py` and `simulation_ready.py` remain finite clients of
an explicitly selected service, not publishers.

`scene_generate.py` is a finite offline producer for the same scene asset
contract. `--describe` prints the installed source catalog and parameter
definitions. Otherwise it reads a JSON request containing `name`, `source`,
`parameters`, and optional uploaded `input` or `treeModels` entries (`file` and
base64 bytes). It returns an ordinary Scene create request; the Scene asset
writer seals the result. It does not write the asset archive or start another
program. Sources include parameterized columns/rings, reusable branching tree
models, and PCD/PLY/XYZ imports. Imported clouds retain their original bytes
and require an explicit resolution and surface/occupied interpretation.

Pure store, frozen, generation, source, assembly, motion and geometry tests
remain in this package. Native projection/publication and application receipt
contracts belong to the C++ Adapter's `scene_node_test`,
`scene_compilation_test`, `simulation_client_test` and LiDAR edge suites.
