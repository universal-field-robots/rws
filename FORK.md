# UFR fork of `rws` — differences from upstream

This is Universal Field Robots' fork of [`v-kiniv/rws`](https://github.com/v-kiniv/rws).
This file records how our `jazzy` line differs from upstream `jazzy`, so anyone
vendoring it (currently `xs040_ws` and `lh209l_ws`, both under `src/rws`) knows
what they're getting and why.

Both lines descend from upstream `fa93e85`. **Ours** = `fa93e85` + org `jazzy-patch`
(PR #5) + the fixes below. **Upstream `jazzy`** has moved on with its own commits.
We do **not** track upstream wholesale — we cherry-pick.

---

## Changes only in our fork

### Transport / server (`src/server_node.cpp`)
- **permessage-deflate** WebSocket compression (`deflate_server_config`). Upstream has none — large messages to the GUI are compressed.
- **TCP no-delay** (`set_tcp_pre_init_handler`, Nagle off) and **listen backlog 128** — lower latency, more pending connections.
- **`nlohmann::ordered_json`** everywhere (upstream uses `nlohmann::json`) — preserves field order in responses.
- **Safer locking** — `connection_lock_` is held across all `process_messages` branches, with `if (cd)` null-guards; `ping_all_clients` takes the lock and skips expired handles.

### Typesupport (`src/typesupport_helpers.cpp`, `include/rws/generic_client.hpp`)
- **`*_msgs` library fallback** — `get_typesupport_library_path` and
  `get_service_typesupport_handle` try `pkg__…` then `pkg_msgs__…`, and
  `generic_client` routes service typesupport through `rws::get_typesupport_library`.
  Upstream uses stock `rclcpp::get_typesupport_library` with no fallback. **This is
  what lets our `*_msgs` services resolve** (e.g. `ufr2_msgs/srv/*`, `automine_msgs/srv/*`).

### Wire format for the GUI (`src/translate.cpp`)
- **UUID `uint8[]` as a map** — serialized by iterating the JSON value
  (`{"0":..,"1":..}`), because our GUI sends/expects that shape. **Do not revert** —
  the GUI depends on it. (Trade-off: a fixed-size `T[N]` is emitted with
  `field.size()` elements, so it relies on the GUI always sending the full length.)
- **`secs`/`nsecs` timestamp field mapping**, both directions, to match roslibjs.
  Upstream uses singular `nsec`.
- Empty message arrays emit `[]` explicitly.

### Connector (`include/rws/connector.hpp`)
- **`subscribers_mutex_` held in `topic_message_callback`** — upstream removed this
  lock (it iterates `subscribers_` unlocked, racing subscribe/unsubscribe). We keep it.
- **Latched-topic replay (bounded one-shot)** — when a late subscriber reuses an
  existing shared subscription it would miss the latched sample. We spin up a
  temporary transient-local subscription that replays the latched value from DDS to
  just that subscriber, then tear it down. The original blocked on `future.wait()`
  forever (leaking the thread/subscription if nothing was ever latched); it now uses
  a bounded `future.wait_for(2s)` — long enough for DDS to deliver an existing
  latched sample, after which the temp subscription is reset. The "subscribe before
  the publisher exists" case is handled by the shared subscription's fan-out, not by
  this one-shot. (Upstream removed this replay entirely.)

### Native rosapi param services (`src/rosapi_params.cpp`, upstream has nothing like it)
- rws now answers `/rosapi/get_param`, `set_param`, `has_param`, `delete_param` and
  `get_param_names` itself (intercepted in `call_service` before the external-service
  fallthrough), replacing the python `rosapi` node for params. The python version
  serialized every call behind a global lock with a blocking 5s `wait_for_service`,
  so one lookup of a missing param stalled all other GUI param calls for 5s.
- Each call goes through an **async generic client** to the target node's own
  parameter services (`<node>/get_parameters` etc.); nodes not on the graph are
  answered **immediately** with the default. `get_param_names` fans out to all nodes
  concurrently with a 2s watchdog so one hung node can't hang the request.
- Wire contract is byte-compatible with python rosapi (`"<node>:<param>"` names,
  JSON-encoded string values), so `roslibjs`/GUI needed **no changes**. One deliberate
  divergence: python's `set_param` re-parsed a JSON *string* value through YAML (so
  `"5"` silently became the integer 5); we map JSON types directly.

### Remaining rosapi services in C++ (`src/rosapi_introspection.cpp`)
- Completes the rosapi surface so the python rosapi node / rosbridge can be dropped
  entirely: `/rosapi/services`, `topics_for_type`, `action_servers`,
  `service_providers`, `service_node`, `get_time`, `get_ros_version`,
  `message_details`, `service_request_details`, `service_response_details`. All
  answered synchronously from the local graph cache / introspection typesupport.
- TypeDef responses: `constnames`/`constvalues` are always empty (introspection
  typesupport doesn't expose constants; python read them off the message class).
  Field/type names use `pkg/msg/Type` form self-consistently, and
  `builtin_interfaces/msg/Time` fields are emitted as `secs`/`nsecs` when
  rosbridge-compatible, matching the wire mapping in `translate.cpp`.
- Deliberate fix vs python: `service_providers` matches by service **name** (the
  .srv field); python accidentally matched by service *type*.

---

## Fixes on branch `jazzy-service-subscribe-fixes` (commit `04d9aa0`)

Targeted at the two problems the GUI hit. Builds clean on Jazzy
(`colcon build --merge-install --packages-select rws`).

- **External service calls never hang** (`call_external_service`). The service type
  is now resolved from the **ROS graph** (a generic client must use the real
  advertised type, not the client's ROS1-style hint). Every failure path now sends a
  `service_response` so the GUI always gets a reply; the async response callback is
  wrapped in try/catch (it runs on an executor thread); `wait_for_service` is bounded
  so a down service can't block the single processing thread. Upstream has **not**
  fixed this — its `call_external_service` is still the pristine original.
- **Subscribe before the publisher exists** (`subscribe_to_topic`). If a topic isn't
  on the graph yet but the client supplies a `type`, we subscribe anyway (rosbridge
  subscribe-ahead semantics) so it starts delivering once a publisher appears; the
  genuine no-type error is throttled. A repeated subscribe is also acknowledged.
  (Upstream `b451caa` fixes the same class differently — prefers the client type and
  validates it against the graph.)

---

## What upstream has that we deliberately have **not** taken

- **No-publisher QoS fallback** (best-effort/volatile when no publisher; otherwise
  match all publishers). Not adopted — it's wrong for transient-local topics. We
  intend to drive subscription QoS from the GUI instead (planned, see below).
- Upstream's removal of the one-shot latched replay, its plain `nlohmann::json`, and
  its nicer typesupport error strings.

## Known issues / planned (not yet implemented)

- **Typesupport library-cache data race** — `get_typesupport_library` reads
  `g_shared_libs_` outside the lock; UB under the MultiThreadedExecutor. Present
  upstream too. *Planned: lock the whole function.*
- **Client-specified subscription QoS** (reliability + durability) plumbed through
  `topic_params` → connector. *Planned — the deterministic replacement for upstream's
  QoS heuristic.*
- **`subscribe`/`advertise` should always respond** — a bad client-supplied type
  currently throws and the client gets no reply (same hang class as the service fix).
  *Planned: try/catch + error response.*
- **`DROP` branch null-deref** in `process_messages` (`cd->is_alive` with no guard).
  *Planned: add `if (cd)`.*

A single GUI refresh drops all subscriptions and re-subscribes, which always creates
fresh transient-local subscriptions — latched values come straight from DDS on that
path, so refresh is unaffected by the one-shot.
