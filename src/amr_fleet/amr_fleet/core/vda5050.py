"""VDA 5050 (version 2.0) order handling and state reporting, with no ROS and no MQTT.

VDA 5050 is the interface German intralogistics standardised between a master
control (the fleet manager, often inside a WMS) and the vehicles it dispatches.
The master control publishes an *order*, a graph of nodes and edges, on
``<interface>/<major>/<manufacturer>/<serialNumber>/order``; the vehicle drives
it and reports progress on ``.../state``. This module holds the part of that
contract that can be wrong without any robot attached: parsing and validating
an order, deciding whether it may be accepted, tracking which nodes have been
traversed, and building a state message. ``amr_fleet.nodes.vda5050_adapter``
wires it to MQTT and to Nav2.

IMPLEMENTED SUBSET, stated rather than implied:

* ``order``: a new order with released (base) nodes, each with a
  ``nodePosition`` on this vehicle's map. Nodes are driven in sequence and
  reported as traversed one by one. Unreleased (horizon) nodes are accepted and
  reported in ``nodeStates`` but not driven, as the standard requires.
* Orders are refused, with an error in the state, when they fail validation,
  when the vehicle is busy with another order, when the first node is not where
  the vehicle is, or when they carry node or edge actions (none are supported).
* Order updates (same ``orderId``, higher ``orderUpdateId``) are refused with
  ``orderUpdateError``. A duplicate of the current update is ignored.
* ``instantActions``: ``cancelOrder`` only. Anything else is refused.
* ``state`` with every field the 2.0 schema marks required. The battery is not
  simulated, so ``batteryState`` reports a constant and says so in
  ``information``.
* ``connection``: ONLINE on connect, CONNECTIONBROKEN as the MQTT last will.
* Not implemented: ``factsheet``, ``visualization``, node and edge actions,
  trajectories, order stitching, loads.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional, Sequence, Tuple

VERSION = "2.0.0"
INTERFACE_NAME = "uagv"
MAJOR_VERSION = "v2"

# Error levels and the error types this adapter raises.
WARNING = "WARNING"
FATAL = "FATAL"

# Distance from the first node within which the vehicle counts as "on" it when
# the order gives no allowedDeviationXY. 0.5 m is the goal tolerance the
# navigation stack itself works to, rounded up.
DEFAULT_ALLOWED_DEVIATION_XY = 0.5

SUPPORTED_INSTANT_ACTIONS = ("cancelOrder",)


def topic(manufacturer: str, serial_number: str, name: str,
          interface_name: str = INTERFACE_NAME, major_version: str = MAJOR_VERSION) -> str:
    """The MQTT topic for one VDA 5050 message type, per section 6.5 of the standard."""
    for label, value in (("manufacturer", manufacturer), ("serial_number", serial_number)):
        if not value or any(c in value for c in "/+#"):
            raise ValueError("%s %r is not a valid topic level" % (label, value))
    return "/".join((interface_name, major_version, manufacturer, serial_number, name))


def timestamp(now: Optional[datetime] = None) -> str:
    """ISO 8601 UTC with milliseconds, the format the schema asks for."""
    now = now or datetime.now(timezone.utc)
    return now.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (now.microsecond // 1000)


class OrderRejected(Exception):
    """An order or instant action that cannot be accepted, carrying the VDA error."""

    def __init__(self, error_type: str, description: str, references: Sequence[Tuple[str, str]] = ()):
        super().__init__(description)
        self.error_type = error_type
        self.description = description
        self.references = list(references)

    def to_error(self) -> dict:
        return {
            "errorType": self.error_type,
            "errorLevel": WARNING,
            "errorDescription": self.description,
            "errorReferences": [
                {"referenceKey": key, "referenceValue": value} for key, value in self.references
            ],
        }


@dataclass
class NodePosition:
    x: float
    y: float
    theta: Optional[float]
    map_id: str
    allowed_deviation_xy: Optional[float] = None
    allowed_deviation_theta: Optional[float] = None

    def to_dict(self) -> dict:
        out = {"x": self.x, "y": self.y, "mapId": self.map_id}
        if self.theta is not None:
            out["theta"] = self.theta
        if self.allowed_deviation_xy is not None:
            out["allowedDeviationXY"] = self.allowed_deviation_xy
        if self.allowed_deviation_theta is not None:
            out["allowedDeviationTheta"] = self.allowed_deviation_theta
        return out


@dataclass
class OrderNode:
    node_id: str
    sequence_id: int
    released: bool
    position: Optional[NodePosition]
    description: str = ""

    def state(self) -> dict:
        out = {"nodeId": self.node_id, "sequenceId": self.sequence_id, "released": self.released}
        if self.description:
            out["nodeDescription"] = self.description
        if self.position is not None:
            out["nodePosition"] = self.position.to_dict()
        return out


@dataclass
class OrderEdge:
    edge_id: str
    sequence_id: int
    released: bool
    start_node_id: str
    end_node_id: str

    def state(self) -> dict:
        return {"edgeId": self.edge_id, "sequenceId": self.sequence_id, "released": self.released}


@dataclass
class Order:
    order_id: str
    order_update_id: int
    nodes: List[OrderNode]
    edges: List[OrderEdge]
    header_id: int = 0


def _require(payload: dict, key: str, kind, where: str):
    if key not in payload:
        raise OrderRejected("validationError", "%s is missing required field %s" % (where, key),
                            [("field", key)])
    value = payload[key]
    if kind is float:
        ok = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    elif kind is int:
        ok = isinstance(value, int) and not isinstance(value, bool)
    else:
        ok = isinstance(value, kind)
    if not ok:
        raise OrderRejected("validationError", "%s field %s has the wrong type" % (where, key),
                            [("field", key)])
    return value


def parse_order(payload: dict, manufacturer: str, serial_number: str, map_id: str) -> Order:
    """Validate an order message and return it in typed form.

    Raises :class:`OrderRejected` with a ``validationError`` for anything that
    is malformed, and with ``orderError`` for a well-formed order this subset
    does not support.
    """
    if not isinstance(payload, dict):
        raise OrderRejected("validationError", "order is not a JSON object")
    for key in ("manufacturer", "serialNumber"):
        _require(payload, key, str, "order")
    if payload["manufacturer"] != manufacturer or payload["serialNumber"] != serial_number:
        raise OrderRejected("validationError", "order is addressed to another vehicle",
                            [("serialNumber", str(payload["serialNumber"]))])
    order_id = _require(payload, "orderId", str, "order")
    update_id = _require(payload, "orderUpdateId", int, "order")
    header_id = payload.get("headerId", 0)
    raw_nodes = _require(payload, "nodes", list, "order")
    raw_edges = _require(payload, "edges", list, "order")
    if not raw_nodes:
        raise OrderRejected("validationError", "order has no nodes", [("orderId", order_id)])
    if len(raw_edges) != len(raw_nodes) - 1:
        raise OrderRejected("validationError", "an order with n nodes must have n - 1 edges",
                            [("orderId", order_id)])

    nodes: List[OrderNode] = []
    for raw in raw_nodes:
        if not isinstance(raw, dict):
            raise OrderRejected("validationError", "node is not an object", [("orderId", order_id)])
        node_id = _require(raw, "nodeId", str, "node")
        where = "node %s" % node_id
        sequence_id = _require(raw, "sequenceId", int, where)
        released = _require(raw, "released", bool, where)
        if raw.get("actions"):
            raise OrderRejected("orderError", "node actions are not supported by this vehicle",
                                [("nodeId", node_id)])
        position = None
        if "nodePosition" in raw:
            pos = _require(raw, "nodePosition", dict, where)
            theta = pos.get("theta")
            if theta is not None:
                theta = float(_require(pos, "theta", float, where + " nodePosition"))
            position = NodePosition(
                x=float(_require(pos, "x", float, where + " nodePosition")),
                y=float(_require(pos, "y", float, where + " nodePosition")),
                theta=theta,
                map_id=_require(pos, "mapId", str, where + " nodePosition"),
                allowed_deviation_xy=pos.get("allowedDeviationXY"),
                allowed_deviation_theta=pos.get("allowedDeviationTheta"),
            )
            if position.map_id != map_id:
                raise OrderRejected("orderError", "node %s is on map %r, not %r"
                                    % (node_id, position.map_id, map_id), [("nodeId", node_id)])
        elif released:
            # Free-navigating vehicles need a position for every node they are
            # asked to drive to. Line-guided ones may not; this is not one.
            raise OrderRejected("orderError", "released node %s has no nodePosition" % node_id,
                                [("nodeId", node_id)])
        nodes.append(OrderNode(node_id, sequence_id, released, position, raw.get("nodeDescription", "")))

    edges: List[OrderEdge] = []
    for raw in raw_edges:
        if not isinstance(raw, dict):
            raise OrderRejected("validationError", "edge is not an object", [("orderId", order_id)])
        edge_id = _require(raw, "edgeId", str, "edge")
        where = "edge %s" % edge_id
        if raw.get("actions"):
            raise OrderRejected("orderError", "edge actions are not supported by this vehicle",
                                [("edgeId", edge_id)])
        edges.append(OrderEdge(
            edge_id,
            _require(raw, "sequenceId", int, where),
            _require(raw, "released", bool, where),
            _require(raw, "startNodeId", str, where),
            _require(raw, "endNodeId", str, where),
        ))

    # Sequence ids interleave: node 0, edge 1, node 2, edge 3, ... and each edge
    # joins the nodes on either side of it.
    for index, node in enumerate(nodes):
        if node.sequence_id != nodes[0].sequence_id + 2 * index:
            raise OrderRejected("validationError", "node sequenceIds must increase in steps of 2",
                                [("nodeId", node.node_id)])
    for index, edge in enumerate(edges):
        if edge.sequence_id != nodes[index].sequence_id + 1:
            raise OrderRejected("validationError", "edge %s is out of sequence" % edge.edge_id,
                                [("edgeId", edge.edge_id)])
        if (edge.start_node_id, edge.end_node_id) != (nodes[index].node_id, nodes[index + 1].node_id):
            raise OrderRejected("validationError", "edge %s does not join its neighbouring nodes"
                                % edge.edge_id, [("edgeId", edge.edge_id)])

    # The base (released part) must come first and be contiguous: a released
    # node after an unreleased one has no defined meaning.
    if not nodes[0].released:
        raise OrderRejected("validationError", "the first node must be released",
                            [("nodeId", nodes[0].node_id)])
    for earlier, later in zip(nodes, nodes[1:]):
        if later.released and not earlier.released:
            raise OrderRejected("validationError", "released node follows the horizon",
                                [("nodeId", later.node_id)])
    for index, edge in enumerate(edges):
        if edge.released != nodes[index + 1].released:
            raise OrderRejected("validationError", "edge %s release does not match its end node"
                                % edge.edge_id, [("edgeId", edge.edge_id)])

    return Order(order_id, update_id, nodes, edges, header_id)


class Vehicle:
    """The vehicle side of VDA 5050: which order is active and how far it has got.

    Pure state. The adapter feeds it orders, positions and navigation results
    and asks it for the next goal and for state messages.
    """

    def __init__(self, manufacturer: str, serial_number: str, map_id: str) -> None:
        self.manufacturer = manufacturer
        self.serial_number = serial_number
        self.map_id = map_id

        self.order_id = ""
        self.order_update_id = 0
        self.last_node_id = ""
        self.last_node_sequence_id = 0
        self.node_states: List[OrderNode] = []
        self.edge_states: List[OrderEdge] = []
        self.driving = False
        self.errors: List[dict] = []
        self.action_states: List[dict] = []
        self.position: Optional[Tuple[float, float, float]] = None
        self.velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._header_id = 0

    # ------------------------------------------------------------------ orders
    @property
    def busy(self) -> bool:
        """True while any released node is still to be traversed."""
        return any(node.released for node in self.node_states)

    def receive_order(self, payload: dict) -> Optional[Order]:
        """Accept an order or record why not. Returns the accepted order, or None."""
        try:
            order = parse_order(payload, self.manufacturer, self.serial_number, self.map_id)
            self._check_acceptable(order)
        except OrderRejected as rejected:
            self.errors.append(rejected.to_error())
            return None
        if order.order_id == self.order_id and order.order_update_id == self.order_update_id:
            return None  # a duplicate of what is already running: ignore, no error
        self._start(order)
        return order

    def _check_acceptable(self, order: Order) -> None:
        if order.order_id == self.order_id:
            if order.order_update_id < self.order_update_id:
                raise OrderRejected("orderUpdateError", "orderUpdateId %d is older than %d"
                                    % (order.order_update_id, self.order_update_id),
                                    [("orderId", order.order_id)])
            if order.order_update_id > self.order_update_id:
                raise OrderRejected("orderUpdateError", "order updates are not supported by this vehicle",
                                    [("orderId", order.order_id)])
            return
        if self.busy:
            raise OrderRejected("orderError", "vehicle is still executing order %s" % self.order_id,
                                [("orderId", order.order_id)])
        first = order.nodes[0].position
        if first is not None and self.position is not None:
            allowed = first.allowed_deviation_xy
            if allowed is None or allowed <= 0.0:
                allowed = DEFAULT_ALLOWED_DEVIATION_XY
            distance = math.hypot(first.x - self.position[0], first.y - self.position[1])
            if distance > allowed:
                raise OrderRejected("noRouteError", "first node %s is %.2f m away; the vehicle must be "
                                    "on it (within %.2f m)" % (order.nodes[0].node_id, distance, allowed),
                                    [("nodeId", order.nodes[0].node_id)])

    def _start(self, order: Order) -> None:
        self.order_id = order.order_id
        self.order_update_id = order.order_update_id
        self.node_states = list(order.nodes)
        self.edge_states = list(order.edges)
        # The errors that belong to the previous order no longer apply.
        self.errors = []
        # The vehicle is on the first node already, so it is traversed at once.
        self.node_reached(order.nodes[0].node_id)

    def next_goal(self) -> Optional[OrderNode]:
        """The next released node to drive to, or None when the base is done."""
        for node in self.node_states:
            if node.released:
                return node
            break
        return None

    def node_reached(self, node_id: str) -> None:
        """Mark a node traversed: it and the edge leading to it leave the state."""
        if not self.node_states or self.node_states[0].node_id != node_id:
            raise ValueError("node %s is not the next node of order %s" % (node_id, self.order_id))
        node = self.node_states.pop(0)
        self.last_node_id = node.node_id
        self.last_node_sequence_id = node.sequence_id
        if self.edge_states and self.edge_states[0].end_node_id == node_id:
            self.edge_states.pop(0)
        if not self.busy:
            self.driving = False

    def navigation_failed(self, node_id: str, reason: str) -> None:
        """The drive to a node failed. The order stays, the vehicle stops and reports it."""
        self.driving = False
        self.errors.append({
            "errorType": "navigationError",
            "errorLevel": WARNING,
            "errorDescription": reason,
            "errorReferences": [{"referenceKey": "nodeId", "referenceValue": node_id},
                                {"referenceKey": "orderId", "referenceValue": self.order_id}],
        })

    # ---------------------------------------------------------- instant actions
    def receive_instant_actions(self, payload: dict) -> List[str]:
        """Handle an instantActions message. Returns the action types executed."""
        executed: List[str] = []
        actions = payload.get("actions") if isinstance(payload, dict) else None
        if not isinstance(actions, list):
            self.errors.append(OrderRejected("validationError", "instantActions has no actions list").to_error())
            return executed
        for action in actions:
            action_type = action.get("actionType", "") if isinstance(action, dict) else ""
            action_id = action.get("actionId", "") if isinstance(action, dict) else ""
            if action_type not in SUPPORTED_INSTANT_ACTIONS:
                self.errors.append(OrderRejected(
                    "actionNotSupported" if action_type else "validationError",
                    "instant action %r is not supported by this vehicle" % action_type,
                    [("actionId", str(action_id))]).to_error())
                self.action_states.append({"actionId": action_id, "actionType": action_type,
                                           "actionStatus": "FAILED"})
                continue
            if action_type == "cancelOrder":
                if not self.node_states and not self.edge_states:
                    self.errors.append(OrderRejected("noOrderToCancel", "there is no order to cancel",
                                                     [("actionId", str(action_id))]).to_error())
                    self.action_states.append({"actionId": action_id, "actionType": action_type,
                                               "actionStatus": "FAILED"})
                    continue
                self.node_states = []
                self.edge_states = []
                self.driving = False
                self.action_states.append({"actionId": action_id, "actionType": action_type,
                                           "actionStatus": "FINISHED"})
                executed.append(action_type)
        return executed

    # -------------------------------------------------------------------- state
    def state(self, now: Optional[datetime] = None) -> dict:
        """A state message with every field VDA 5050 2.0 marks required."""
        self._header_id += 1
        position = None
        if self.position is not None:
            x, y, theta = self.position
            position = {"x": x, "y": y, "theta": theta, "mapId": self.map_id,
                        "positionInitialized": True}
        vx, vy, omega = self.velocity
        message = {
            "headerId": self._header_id,
            "timestamp": timestamp(now),
            "version": VERSION,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "lastNodeId": self.last_node_id,
            "lastNodeSequenceId": self.last_node_sequence_id,
            "nodeStates": [node.state() for node in self.node_states],
            "edgeStates": [edge.state() for edge in self.edge_states],
            "driving": self.driving,
            "paused": False,
            "newBaseRequest": False,
            "operatingMode": "AUTOMATIC",
            "actionStates": list(self.action_states),
            # Not simulated. Reported as full and labelled, rather than omitted,
            # because the schema requires the object.
            "batteryState": {"batteryCharge": 100.0, "charging": False},
            "errors": list(self.errors),
            "information": [{
                "infoType": "batteryNotSimulated",
                "infoLevel": "INFO",
                "infoDescription": "batteryState is a constant; the simulation has no battery model",
            }],
            "safetyState": {"eStop": "NONE", "fieldViolation": False},
            "velocity": {"vx": vx, "vy": vy, "omega": omega},
            "loads": [],
        }
        if position is not None:
            message["agvPosition"] = position
        return message

    def connection(self, state: str, now: Optional[datetime] = None) -> dict:
        """A connection message: ONLINE, OFFLINE or CONNECTIONBROKEN."""
        if state not in ("ONLINE", "OFFLINE", "CONNECTIONBROKEN"):
            raise ValueError("unknown connectionState %r" % state)
        self._header_id += 1
        return {
            "headerId": self._header_id,
            "timestamp": timestamp(now),
            "version": VERSION,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "connectionState": state,
        }


def order_complete(state: Dict) -> bool:
    """True when a state message reports an order fully driven: what a master control checks."""
    return (bool(state.get("orderId")) and not state.get("nodeStates") and not state.get("edgeStates")
            and not state.get("driving") and not state.get("errors"))


@dataclass
class OrderBuilder:
    """Build a straight-line order through a list of (x, y, theta) poses. Used by the
    tests and by anyone wanting to drive the vehicle by hand from a script."""

    manufacturer: str
    serial_number: str
    map_id: str
    order_id: str
    poses: List[Tuple[float, float, float]] = field(default_factory=list)
    header_id: int = 1

    def build(self) -> dict:
        nodes, edges = [], []
        for index, (x, y, theta) in enumerate(self.poses):
            nodes.append({
                "nodeId": "n%d" % index,
                "sequenceId": 2 * index,
                "released": True,
                "nodePosition": {"x": x, "y": y, "theta": theta, "mapId": self.map_id},
                "actions": [],
            })
            if index:
                edges.append({
                    "edgeId": "e%d" % index,
                    "sequenceId": 2 * index - 1,
                    "released": True,
                    "startNodeId": "n%d" % (index - 1),
                    "endNodeId": "n%d" % index,
                    "actions": [],
                })
        return {
            "headerId": self.header_id,
            "timestamp": timestamp(),
            "version": VERSION,
            "manufacturer": self.manufacturer,
            "serialNumber": self.serial_number,
            "orderId": self.order_id,
            "orderUpdateId": 0,
            "nodes": nodes,
            "edges": edges,
        }
