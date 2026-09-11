# Obstacle scene runtime

`xgc2_scene_runtime` loads and owns an obstacle scene independently of the
planner and simulator. The algorithm project owns the YAML file. To start a
scene, run the following in a sourced ROS environment:

```bash
roslaunch xgc2_scene_runtime scene.launch \
  scene_file:=/absolute/project/scene.yaml \
  namespace:=/xgc/scene gazebo:=true
```

With Gazebo enabled, first start the `scene_editable/scene_editable.world`
asset from `gazebo_sim_worlds`. The world provides an adapter; it does not select
or load YAML. This Gazebo adapter accepts the `world` frame and uses the
namespace declared by its world plugin (the supplied asset uses `/xgc/scene`).
Other frames are explicitly rejected; no implicit frame conversion is applied.
Set `gazebo:=false` to run without that simulator. Set
`save_directory` to a writable project directory when it differs from the
source YAML's parent directory. The GCS Scene user workflow owns this process;
starting or stopping the algorithm does not start or stop it.

```yaml
schema: xgc2.scene.v1
id: example
frame: world
obstacles:
  - id: box-1
    name: Box
    pose:
      position: [1, 2, 0.5]
      orientation: [0, 0, 0, 1]
    parts:
      - id: body
        geometry: {type: box, size: [1, 1, 1]}
        color: [1.0, 0.5, 0.1, 1.0]
    motion: {type: hold}
```

Obstacle poses are in the declared scene frame. Part poses are local to their
obstacle and default to identity. Quaternions are XYZW; lengths are metres.
IDs remain stable through edits and undo. Copying creates a new obstacle ID.

| Geometry type | Parameters |
| --- | --- |
| `box` | `size`: three full side lengths |
| `sphere` | `radius` |
| `cylinder` | `radius`, full `height`, local Z axis |
| `capsule` | `radius`, straight section `height`, local Z axis; end hemispheres add `2 * radius` |
| `convex` | `vertices`: XYZ arrays; `triangles`: flat vertex index triplets |

Convex meshes must have a closed convex surface and nonzero volume. Scaling is
baked into vertices. Multiple parts form a union: doorways and concavities are
preserved. Unsupported shapes are rejected, never replaced with another shape.

Endpoints below are relative to the selected namespace:

| Endpoint | ROS type | Purpose |
| --- | --- | --- |
| `snapshot` | `xgc2_geometry_msgs/SceneSnapshot` | Latched definitions and initial poses |
| `state` | `xgc2_geometry_msgs/SceneState` | Current poses/twists and playback time, 30 Hz |
| `document` | `std_msgs/String` | Latched JSON authoring document and status |
| `markers` | `visualization_msgs/MarkerArray` | Scene-owned display projection, independent of the algorithm |
| `consumer_status` | `xgc2_geometry_msgs/SceneConsumerStatus` | Consumer acknowledgements |
| `command` | `xgc2_geometry_msgs/SceneCommand` | JSON authoring request and response |
| `gazebo/apply` | `xgc2_geometry_msgs/ApplyScene` | Simulator adapter's verified full-snapshot application |

`get` returns the current document and epoch/revision. Every mutation requires
`requestId`, `expectedEpoch`, and `expectedRevision`. Supported operations are
`add`/`update` with a complete `obstacle`, `delete` with `id`, `clear`, `replace`
with a complete `document`, `undo`, `redo`, `save`, `play`, `pause`, `reset`, and
`resync`, and `reload`. A drag previews locally and submits once on release.

The response reports `success`, an optional `error`, `epoch`, `revision`,
`document`, `savedRevision`, `dirty`, `playing`, `sceneTime`, `online`,
`synchronized`, `syncRetryable`, and consumer statuses. An accepted edit and a
saved file are separate facts. Each live consumer reports explicit `applied`
(this epoch/revision is in that consumer) and `operational` (it can perform its
operation). `success` on a consumer is only a derived copy of `applied`.
`capability` is `""`, `ok`, or `unsupported`. `synchronized` is the conjunction of
live consumers' `applied` plus matching epoch/revision. `syncRetryable` is true
only for version lag or apply transport failure. A declared `unsupported`
capability is not retryable and must not be presented as a sync miss. Exited
members expire after 3 s of receipt silence (monotonic clock). `resync` reapplies
the accepted document at a new revision after partial simulator failure or a lost
reply without adding undo history or changing the saved document.

Publishers of `SceneConsumerStatus` must set `applied` and `operational`
explicitly and set `success` equal to `applied`. Receivers ignore `success`.

Every accepted geometry edit, including undo/redo, atomically writes back the loaded
YAML. `save` retries a failed write to that same file. External file changes reject
further edits until `reload` loads and applies the project YAML; reload clears undo
history. A failed write retains the accepted live geometry and reports dirty state.
Saving does not commit the project repository.

Motion definitions support `hold`, world-frame `constant_twist` (`linear`,
`angular`), `ping_pong` (`point_a`, `point_b`, positive `speed`), and an XY
`circle` (`center`, `radius`, `angular_speed`, optional `phase`). Playback starts
paused at time zero. Ping-pong endpoints and circle centers are absolute scene
coordinates. The initial obstacle position must equal the path start; moving a
whole obstacle in the editor translates its path with it. `reset` rewinds playback; it does not reset geometry or
robots. Current positions never overwrite saved initial poses. Consumers must
declare which motion models they support.
