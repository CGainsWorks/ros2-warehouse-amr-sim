# Fleet interface: VDA 5050 over MQTT

A warehouse robot is dispatched by a master control (a fleet manager, often
part of a WMS), not by someone typing goals into RViz. VDA 5050 is the
intralogistics standard for that link: the master control publishes an
*order*, a graph of nodes and edges, and the vehicle drives it and reports its
*state*. `amr_fleet` is the vehicle side of version 2.0 of that contract, so
this robot can be driven by any VDA 5050 master control.

## Topics

MQTT, one broker, topic prefix `uagv/v2/<manufacturer>/<serialNumber>/`:

| Topic | Direction | Content |
|---|---|---|
| `order` | master to vehicle | the order to drive |
| `instantActions` | master to vehicle | `cancelOrder` |
| `state` | vehicle to master | every second and on every change |
| `connection` | vehicle to master | `ONLINE` (retained), `OFFLINE` on shutdown, `CONNECTIONBROKEN` as the MQTT last will |

Identity and broker are in `src/amr_fleet/config/vda5050.yaml`:
manufacturer `CGainsWorks`, serial number `amr-001`, map `warehouse`.

## What is implemented, and what is not

The adapter implements the subset a master control needs to dispatch single
orders, and says so rather than accepting what it cannot do.

Implemented:

* Orders whose released (base) nodes carry a `nodePosition` on map
  `warehouse`. Nodes are driven in sequence through Nav2's `navigate_to_pose`
  and reported as traversed one by one: each leaves `nodeStates`, the edge
  leading to it leaves `edgeStates`, and `lastNodeId` and `lastNodeSequenceId`
  advance.
* Unreleased (horizon) nodes are accepted and reported but not driven.
* Validation of everything the vehicle relies on: required fields and types,
  the interleaved node and edge sequence ids, edges joining their neighbouring
  nodes, the base preceding the horizon, finite coordinates, the vehicle's own
  serial number and map.
* Refusal with the standard error types: `validationError`, `orderError`
  (vehicle busy, node actions, unknown map), `noRouteError` (first node not
  where the vehicle is), `orderUpdateError`, `noOrderToCancel`,
  `actionNotSupported`, and `navigationError` when Nav2 fails a node.
* `cancelOrder` as an instant action.
* Every field the 2.0 state schema marks required. `agvPosition` is
  `map -> base_footprint` from TF, which is what Nav2 drives on.

Not implemented, and refused rather than ignored where they would arrive:

* Node and edge actions (pick, drop and the rest): an order containing any is
  refused with `orderError`, because silently skipping a pick would report a
  pick that never happened.
* Order updates (same `orderId`, higher `orderUpdateId`) and order stitching:
  refused with `orderUpdateError`.
* `factsheet`, `visualization`, trajectories, loads, and every instant action
  other than `cancelOrder`.
* The battery is not simulated. `batteryState` reports a constant 100% and an
  `information` entry says so.

## Where the logic lives

`amr_fleet/core/vda5050.py` has no ROS or MQTT imports. It parses and
validates orders, decides whether one may be accepted, tracks traversal and
builds the state message; `tests/test_vda5050.py` covers it offline (39 tests).
`amr_fleet/nodes/vda5050_adapter.py` only moves messages between MQTT, Nav2
and TF. paho runs its network loop on its own thread, so incoming messages are
queued and handled on the ROS executor.

## Running it

With Docker:

```bash
docker compose -f docker/docker-compose.yml up mosquitto fleet
```

Natively, with a broker on localhost:

```bash
ros2 launch amr_fleet fleet.launch.py broker_host:=localhost
```

Then publish an order. The first node must be where the robot is (it spawns
at `(-12, 0)`):

```python
import json
import paho.mqtt.client as mqtt
from amr_fleet.core.vda5050 import OrderBuilder, topic

order = OrderBuilder("CGainsWorks", "amr-001", "warehouse", "order-1",
                     [(-12.0, 0.0, 0.0), (-9.45, 0.0, 0.0), (-7.95, 0.0, 0.0)]).build()
client = mqtt.Client(client_id="master-control")
client.connect("localhost", 1883)
client.publish(topic("CGainsWorks", "amr-001", "order"), json.dumps(order))
```

and watch `uagv/v2/CGainsWorks/amr-001/state`, for example with
`mosquitto_sub -t 'uagv/v2/CGainsWorks/amr-001/#' -v`.

## How it is tested in simulation

`src/amr_fleet/test/test_vda5050_order.launch.py` runs in CI: it starts a
private Mosquitto broker, gz-sim with the full stack and the adapter, waits for
`ONLINE` and an initialised position, publishes the three-node order above and
waits for the state to report it complete. It then checks the reported position
and simulator ground truth are both within 0.35 m of the last node. The
summary is uploaded with the CI run's `integration-results` artifact. Every
number it produces is simulated.
