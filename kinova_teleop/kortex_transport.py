"""Lazy, injectable lifecycle management for the Kortex TCP transport."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

RPC_TIMEOUT_MS = 100


@dataclass(frozen=True)
class KortexConfig:
    """Credentials and session settings for one Kortex TCP connection."""

    host: str
    username: str
    password: str
    port: int = 10000
    session_inactivity_timeout_ms: int = 10_000
    connection_inactivity_timeout_ms: int = 2_000


@dataclass(frozen=True)
class KortexFactories:
    """SDK construction boundary, injectable for offline tests."""

    transport: Callable[[], Any]
    router: Callable[[Any, Any], Any]
    session_manager: Callable[[Any], Any]
    base_client: Callable[[Any], Any]
    base_cyclic_client: Callable[[Any], Any]
    create_session_info: Callable[[], Any]
    create_send_options: Callable[[], Any]
    base_pb2: Any


def _sdk_factories() -> KortexFactories:
    """Import Kortex only when a hardware connection is explicitly requested."""

    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import Base_pb2, Session_pb2

    return KortexFactories(
        transport=TCPTransport,
        router=RouterClient,
        session_manager=SessionManager,
        base_client=BaseClient,
        base_cyclic_client=BaseCyclicClient,
        create_session_info=Session_pb2.CreateSessionInfo,
        create_send_options=RouterClientSendOptions,
        base_pb2=Base_pb2,
    )


class KortexConnection:
    """Own the Kortex transport, session, and high-level clients."""

    def __init__(
        self,
        config: KortexConfig,
        *,
        factories: KortexFactories | None = None,
    ):
        self.config = config
        self._factories = factories
        self.transport: Any | None = None
        self.router: Any | None = None
        self.session_manager: Any | None = None
        self.base: Any | None = None
        self.base_cyclic: Any | None = None
        self.base_pb2: Any | None = None
        self._connected = False
        self._closed = False
        self.stop_confirmed = True

    @property
    def factories(self) -> KortexFactories:
        if self._factories is None:
            self._factories = _sdk_factories()
        return self._factories

    def connect(self) -> KortexConnection:
        if self._connected:
            return self
        if self._closed:
            raise RuntimeError("Cannot reconnect a closed Kortex connection")

        failure_message: str | None = None
        try:
            factories = self.factories
            self.transport = factories.transport()
            self.transport.connect(self.config.host, self.config.port)

            callback = getattr(
                factories.router,
                "basicErrorCallback",
                lambda error: None,
            )
            self.router = factories.router(self.transport, callback)
            self.session_manager = factories.session_manager(self.router)

            session_info = factories.create_session_info()
            session_info.username = self.config.username
            session_info.password = self.config.password
            session_info.session_inactivity_timeout = (
                self.config.session_inactivity_timeout_ms
            )
            session_info.connection_inactivity_timeout = (
                self.config.connection_inactivity_timeout_ms
            )
            self.session_manager.CreateSession(
                session_info,
                options=self.rpc_options(),
            )

            self.base = factories.base_client(self.router)
            self.base_cyclic = factories.base_cyclic_client(self.router)
            self.base_pb2 = factories.base_pb2
            self._connected = True
        except BaseException as error:
            stop_confirmed = self._cleanup(stop=self.base is not None)
            if not isinstance(error, Exception):
                raise
            safe_message = str(error)
            if self.config.password:
                safe_message = safe_message.replace(self.config.password, "[REDACTED]")
            failure_message = f"Failed to connect to Kortex robot: {safe_message}"
            if not stop_confirmed:
                failure_message += "; Stop attempted but unconfirmed during cleanup"
        if failure_message is not None:
            raise RuntimeError(failure_message) from None
        return self

    def rpc_options(self) -> Any:
        options = self.factories.create_send_options()
        options.timeout_ms = RPC_TIMEOUT_MS
        return options

    def _cleanup(self, *, stop: bool) -> bool:
        stop_confirmed = not stop
        if stop and self.base is not None:
            try:
                self.base.Stop(options=self.rpc_options())
                stop_confirmed = True
            except BaseException:
                stop_confirmed = False
        if self.session_manager is not None:
            try:
                self.session_manager.CloseSession(options=self.rpc_options())
            except BaseException:
                pass
        if self.router is not None:
            try:
                self.router.SetActivationStatus(False)
            except BaseException:
                close = getattr(self.router, "close", None)
                if close is not None:
                    try:
                        close()
                    except BaseException:
                        pass
        if self.transport is not None:
            try:
                self.transport.disconnect()
            except BaseException:
                pass

        self.base_cyclic = None
        self.base = None
        self.session_manager = None
        self.router = None
        self.transport = None
        self._connected = False
        self.stop_confirmed = stop_confirmed
        return stop_confirmed

    def close(self) -> bool:
        """Stop first, then close all SDK resources in reverse order."""

        if self._closed:
            return self.stop_confirmed
        self._closed = True
        return self._cleanup(stop=True)
