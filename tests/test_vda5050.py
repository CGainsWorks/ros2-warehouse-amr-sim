"""VDA 5050 order handling, without MQTT or ROS.

The adapter node only moves messages; every protocol decision is in
amr_fleet.core.vda5050, and these tests pin the ones a master control would
notice: what gets accepted, what gets refused and with which error, how
traversal is reported, and that a state message carries every required field.
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone

import pytest

from amr_fleet.core.vda5050 import (
    OrderBuilder,
    OrderRejected,
    Vehicle,
    order_complete,
    parse_order,
    timestamp,
    topic,
)

MANUFACTURER = "CGainsWorks"
SERIAL = "amr-001"
MAP = "warehouse"

# Fields the VDA 5050 2.0 state schema marks required.
REQUIRED_STATE_FIELDS = {
    "headerId", "timestamp", "version", "manufacturer", "serialNumber", "orderId",
    "orderUpdateId", "lastNodeId", "lastNodeSequenceId", "nodeStates", "edgeStates",
    "driving", "operatingMode", "actionStates", "batteryState", "errors", "safetyState",
}


def make_order(order_id="order-1", poses=((-12.0, 0.0, 0.0), (-9.45, 0.0, 0.0), (-1.35, 0.0, 0.0))):
    return OrderBuilder(MANUFACTURER, SERIAL, MAP, order_id, list(poses)).build()


@pytest.fixture
def vehicle():
    v = Vehicle(MANUFACTURER, SERIAL, MAP)
    v.position = (-12.0, 0.0, 0.0)
    return v


class TestTopicsAndFormats:
    def test_topic_layout_follows_the_standard(self):
        assert topic(MANUFACTURER, SERIAL, "order") == "uagv/v2/CGainsWorks/amr-001/order"

    def test_topic_levels_reject_wildcards(self):
        with pytest.raises(ValueError):
            topic("a/b", SERIAL, "state")
        with pytest.raises(ValueError):
            topic(MANUFACTURER, "amr+1", "state")

    def test_timestamp_is_iso8601_utc_with_milliseconds(self):
        stamp = timestamp(datetime(2026, 10, 16, 9, 30, 5, 123456, tzinfo=timezone.utc))
        assert stamp == "2026-10-16T09:30:05.123Z"


class TestOrderValidation:
    def test_builder_output_parses(self):
        order = parse_order(make_order(), MANUFACTURER, SERIAL, MAP)
        assert [n.node_id for n in order.nodes] == ["n0", "n1", "n2"]
        assert [e.edge_id for e in order.edges] == ["e1", "e2"]

    @pytest.mark.parametrize("field", ["orderId", "orderUpdateId", "nodes", "edges"])
    def test_missing_required_field_is_a_validation_error(self, field):
        payload = make_order()
        del payload[field]
        with pytest.raises(OrderRejected) as caught:
            parse_order(payload, MANUFACTURER, SERIAL, MAP)
        assert caught.value.error_type == "validationError"

    def test_order_for_another_vehicle_is_refused(self):
        payload = make_order()
        payload["serialNumber"] = "amr-002"
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)

    def test_edge_count_must_match(self):
        payload = make_order()
        payload["edges"].pop()
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)

    def test_sequence_ids_must_interleave(self):
        payload = make_order()
        payload["nodes"][1]["sequenceId"] = 3
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)

    def test_edge_must_join_its_neighbours(self):
        payload = make_order()
        payload["edges"][0]["endNodeId"] = "n2"
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)

    def test_released_node_after_the_horizon_is_invalid(self):
        payload = make_order()
        payload["nodes"][1]["released"] = False
        payload["edges"][0]["released"] = False
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)

    def test_horizon_is_accepted(self):
        payload = make_order()
        payload["nodes"][2]["released"] = False
        payload["edges"][1]["released"] = False
        order = parse_order(payload, MANUFACTURER, SERIAL, MAP)
        assert [n.released for n in order.nodes] == [True, True, False]

    def test_released_node_needs_a_position(self):
        payload = make_order()
        del payload["nodes"][1]["nodePosition"]
        with pytest.raises(OrderRejected) as caught:
            parse_order(payload, MANUFACTURER, SERIAL, MAP)
        assert caught.value.error_type == "orderError"

    def test_node_on_another_map_is_refused(self):
        payload = make_order()
        payload["nodes"][1]["nodePosition"]["mapId"] = "mezzanine"
        with pytest.raises(OrderRejected) as caught:
            parse_order(payload, MANUFACTURER, SERIAL, MAP)
        assert caught.value.error_type == "orderError"

    def test_actions_are_refused_rather_than_ignored(self):
        """Silently skipping a pick action would report a pick that never happened."""
        payload = make_order()
        payload["nodes"][1]["actions"] = [{"actionType": "pick", "actionId": "a1", "blockingType": "HARD"}]
        with pytest.raises(OrderRejected) as caught:
            parse_order(payload, MANUFACTURER, SERIAL, MAP)
        assert caught.value.error_type == "orderError"

    def test_non_finite_coordinates_are_invalid(self):
        payload = make_order()
        payload["nodes"][1]["nodePosition"]["x"] = float("nan")
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)

    def test_boolean_is_not_an_integer(self):
        payload = make_order()
        payload["orderUpdateId"] = True
        with pytest.raises(OrderRejected):
            parse_order(payload, MANUFACTURER, SERIAL, MAP)


class TestOrderExecution:
    def test_accepting_an_order_traverses_the_first_node(self, vehicle):
        assert vehicle.receive_order(make_order()) is not None
        assert vehicle.last_node_id == "n0"
        assert [n.node_id for n in vehicle.node_states] == ["n1", "n2"]
        assert [e.edge_id for e in vehicle.edge_states] == ["e1", "e2"]
        assert vehicle.next_goal().node_id == "n1"

    def test_nodes_and_edges_leave_the_state_as_they_are_reached(self, vehicle):
        vehicle.receive_order(make_order())
        vehicle.driving = True
        vehicle.node_reached("n1")
        assert vehicle.last_node_id == "n1" and vehicle.last_node_sequence_id == 2
        assert [e.edge_id for e in vehicle.edge_states] == ["e2"]
        vehicle.node_reached("n2")
        state = vehicle.state()
        assert state["nodeStates"] == [] and state["edgeStates"] == []
        assert state["driving"] is False
        assert order_complete(state)

    def test_nodes_must_be_reached_in_order(self, vehicle):
        vehicle.receive_order(make_order())
        with pytest.raises(ValueError):
            vehicle.node_reached("n2")

    def test_horizon_nodes_are_reported_but_not_driven(self, vehicle):
        payload = make_order()
        payload["nodes"][2]["released"] = False
        payload["edges"][1]["released"] = False
        vehicle.receive_order(payload)
        vehicle.node_reached("n1")
        assert vehicle.next_goal() is None
        assert not vehicle.busy
        assert [n["nodeId"] for n in vehicle.state()["nodeStates"]] == ["n2"]

    def test_a_new_order_while_busy_is_refused(self, vehicle):
        vehicle.receive_order(make_order("order-1"))
        assert vehicle.receive_order(make_order("order-2")) is None
        assert vehicle.order_id == "order-1"
        assert vehicle.errors[-1]["errorType"] == "orderError"

    def test_a_new_order_after_completion_is_accepted(self, vehicle):
        vehicle.receive_order(make_order("order-1", poses=((-12.0, 0.0, 0.0), (-9.45, 0.0, 0.0))))
        vehicle.node_reached("n1")
        vehicle.position = (-9.45, 0.0, 0.0)
        assert vehicle.receive_order(make_order("order-2", poses=((-9.45, 0.0, 0.0), (-1.35, 0.0, 0.0))))
        assert vehicle.order_id == "order-2"

    def test_first_node_must_be_where_the_vehicle_is(self, vehicle):
        vehicle.position = (-5.0, 0.0, 0.0)
        assert vehicle.receive_order(make_order()) is None
        assert vehicle.errors[-1]["errorType"] == "noRouteError"

    def test_allowed_deviation_widens_the_first_node(self, vehicle):
        payload = make_order()
        payload["nodes"][0]["nodePosition"]["allowedDeviationXY"] = 3.0
        vehicle.position = (-10.0, 0.0, 0.0)
        assert vehicle.receive_order(payload) is not None

    def test_duplicate_order_is_ignored_without_error(self, vehicle):
        payload = make_order()
        vehicle.receive_order(payload)
        assert vehicle.receive_order(copy.deepcopy(payload)) is None
        assert vehicle.errors == []

    def test_order_update_is_refused_as_unsupported(self, vehicle):
        payload = make_order()
        vehicle.receive_order(payload)
        update = copy.deepcopy(payload)
        update["orderUpdateId"] = 1
        assert vehicle.receive_order(update) is None
        assert vehicle.errors[-1]["errorType"] == "orderUpdateError"

    def test_stale_order_update_is_refused(self, vehicle):
        payload = make_order()
        payload["orderUpdateId"] = 3
        vehicle.receive_order(payload)
        stale = copy.deepcopy(payload)
        stale["orderUpdateId"] = 2
        assert vehicle.receive_order(stale) is None
        assert vehicle.errors[-1]["errorType"] == "orderUpdateError"

    def test_navigation_failure_is_reported_and_stops_the_vehicle(self, vehicle):
        vehicle.receive_order(make_order())
        vehicle.driving = True
        vehicle.navigation_failed("n1", "planner found no path")
        state = vehicle.state()
        assert state["driving"] is False
        assert state["errors"][-1]["errorType"] == "navigationError"
        assert not order_complete(state)


class TestInstantActions:
    def test_cancel_order_clears_the_order(self, vehicle):
        vehicle.receive_order(make_order())
        executed = vehicle.receive_instant_actions(
            {"actions": [{"actionType": "cancelOrder", "actionId": "c1", "blockingType": "HARD"}]})
        assert executed == ["cancelOrder"]
        assert vehicle.node_states == [] and not vehicle.busy
        assert vehicle.action_states[-1]["actionStatus"] == "FINISHED"

    def test_cancel_without_an_order_is_an_error(self, vehicle):
        vehicle.receive_instant_actions({"actions": [{"actionType": "cancelOrder", "actionId": "c1"}]})
        assert vehicle.errors[-1]["errorType"] == "noOrderToCancel"

    def test_unsupported_instant_action_fails_visibly(self, vehicle):
        vehicle.receive_instant_actions({"actions": [{"actionType": "startCharging", "actionId": "x"}]})
        assert vehicle.action_states[-1]["actionStatus"] == "FAILED"
        assert vehicle.errors[-1]["errorType"] == "actionNotSupported"


class TestStateMessage:
    def test_state_has_every_required_field(self, vehicle):
        assert REQUIRED_STATE_FIELDS <= set(vehicle.state())

    def test_header_id_increases(self, vehicle):
        assert vehicle.state()["headerId"] < vehicle.state()["headerId"]

    def test_position_is_reported_on_the_map(self, vehicle):
        position = vehicle.state()["agvPosition"]
        assert position == {"x": -12.0, "y": 0.0, "theta": 0.0, "mapId": MAP, "positionInitialized": True}

    def test_unsimulated_battery_is_labelled(self, vehicle):
        info = vehicle.state()["information"]
        assert any(i["infoType"] == "batteryNotSimulated" for i in info)

    def test_connection_states(self, vehicle):
        assert vehicle.connection("ONLINE")["connectionState"] == "ONLINE"
        with pytest.raises(ValueError):
            vehicle.connection("ASLEEP")
