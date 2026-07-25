from __future__ import annotations

from types import SimpleNamespace

import pytest

from kinova_teleop.kortex_transport import (
    KortexConfig,
    KortexConnection,
    KortexFactories,
)


def _factories(events: list[object], *, session_error: Exception | None = None):
    class SendOptions:
        def __init__(self):
            self.timeout_ms = 10_000

    class Transport:
        def connect(self, host, port):
            events.append(("transport.connect", host, port))

        def disconnect(self):
            events.append("transport.disconnect")

    class Router:
        basicErrorCallback = object()

        def __init__(self, transport, callback):
            events.append(("router.create", callback))

        def SetActivationStatus(self, active):
            events.append(("router.active", active))

    class SessionManager:
        def __init__(self, router):
            events.append("session.create")

        def CreateSession(self, info):
            events.append(
                (
                    "session.open",
                    info.username,
                    info.password,
                    info.session_inactivity_timeout,
                    info.connection_inactivity_timeout,
                )
            )
            if session_error is not None:
                raise session_error

        def CloseSession(self):
            events.append("session.close")

    class BaseClient:
        def __init__(self, router):
            events.append("base.create")

        def Stop(self, *, options=None):
            events.append(("stop.timeout", options.timeout_ms))
            events.append("base.stop")

    class BaseCyclicClient:
        def __init__(self, router):
            events.append("cyclic.create")

    class CreateSessionInfo:
        username = ""
        password = ""
        session_inactivity_timeout = 0
        connection_inactivity_timeout = 0

    return KortexFactories(
        transport=Transport,
        router=Router,
        session_manager=SessionManager,
        base_client=BaseClient,
        base_cyclic_client=BaseCyclicClient,
        create_session_info=CreateSessionInfo,
        create_send_options=SendOptions,
        base_pb2=SimpleNamespace(),
    )


def test_connect_uses_tcp_10000_and_close_stops_then_reverses_lifecycle():
    events: list[object] = []
    connection = KortexConnection(
        KortexConfig("192.0.2.10", "operator", "secret"),
        factories=_factories(events),
    )

    connection.connect()
    connection.close()
    connection.close()

    assert events == [
        ("transport.connect", "192.0.2.10", 10000),
        ("router.create", connection.factories.router.basicErrorCallback),
        "session.create",
        ("session.open", "operator", "secret", 10_000, 2_000),
        "base.create",
        "cyclic.create",
        ("stop.timeout", 100),
        "base.stop",
        "session.close",
        ("router.active", False),
        "transport.disconnect",
    ]


def test_connect_error_redacts_password_and_cleans_up_partial_connection():
    events: list[object] = []
    connection = KortexConnection(
        KortexConfig("192.0.2.10", "operator", "top-secret"),
        factories=_factories(events, session_error=RuntimeError("bad top-secret")),
    )

    with pytest.raises(RuntimeError) as error:
        connection.connect()

    assert "top-secret" not in str(error.value)
    assert error.value.__cause__ is None
    assert error.value.__context__ is None
    assert events[-3:] == [
        "session.close",
        ("router.active", False),
        "transport.disconnect",
    ]


def test_base_cyclic_creation_failure_stops_before_disconnect():
    events: list[object] = []
    factories = _factories(events)

    def fail_cyclic(router):
        events.append("cyclic.create.failed")
        raise RuntimeError("cyclic unavailable")

    factories = KortexFactories(
        transport=factories.transport,
        router=factories.router,
        session_manager=factories.session_manager,
        base_client=factories.base_client,
        base_cyclic_client=fail_cyclic,
        create_session_info=factories.create_session_info,
        create_send_options=factories.create_send_options,
        base_pb2=factories.base_pb2,
    )
    connection = KortexConnection(
        KortexConfig("192.0.2.10", "operator", "secret"),
        factories=factories,
    )

    with pytest.raises(RuntimeError, match="cyclic unavailable"):
        connection.connect()

    assert events[-6:] == [
        "cyclic.create.failed",
        ("stop.timeout", 100),
        "base.stop",
        "session.close",
        ("router.active", False),
        "transport.disconnect",
    ]
