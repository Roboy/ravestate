import subprocess
import secrets
import sqlite3
import time
import os
import psutil
import signal
from typing import Dict, List, Set, Optional, Tuple
from threading import RLock, Thread

from reggol import get_logger
logger = get_logger(__name__)

# Get path to python interpreter, such that we can use it to launch ravestate
py_interpreter = os.path.join(os.__file__.split("lib/")[0], "bin", "python")


# Encapsulates a single raveboard session
class Session:
    def __init__(self, *, secret, process, port, sio_url):
        self.secret = secret
        self.port = port
        self.process: psutil.Process = process
        self.sio_url = sio_url


# Valid value for state column
IDLE_STATE = "idle"
ALLOCATED_STATE = "allocated"
IN_USE_STATE = "in_use"
KILLME_STATE = "killme"


# Manages session sqlite db and associated running sessions
class SessionManager:

    def __init__(self, *,
                 db_path,
                 refresh_iv_secs,
                 session_launch_args,
                 num_idle_instances,
                 usable_ports,
                 hostname,
                 zombie_heartbeat_threshold=20.):
        self.db_path = db_path
        self.refresh_iv_secs = refresh_iv_secs
        self.session_launch_args: List[str] = session_launch_args
        self.num_idle_instances = num_idle_instances
        self.usable_ports = usable_ports
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.current_sessions: Dict[int, Session] = {}
        self.hostname = hostname
        self.zombie_heartbeat_threshold = zombie_heartbeat_threshold
        self.lock = RLock()
        self.update_thread = Thread(target=self.update)
        self.created_sessions = 0

        if not self.conn:
            logger.error("Failed to open SQLite DB at {}!".format(self.db_path))

        if not self.num_idle_instances:
            logger.warn("At least one idle instance is required!")
            self.num_idle_instances = 1

        # -- prepare db: create table
        self.conn.execute("""
        create table if not exists sessions (
            port integer primary key, state text, url text, heartbeat real, secret text, since real, use_before real
        )
        """)

        # -- prepare db: clean out old entries
        self.conn.execute(f"""
        delete from sessions
        """)

        while len(self.idle_ports()) < self.num_idle_instances:
            self.create_idle_session()
        self.update_thread.start()

    def used_ports(self) -> Set[int]:
        with self.lock:
            used_port_rows = self.conn.execute(f"""
            select port from sessions
            """)
            return {port_row[0] for port_row in used_port_rows}

    def idle_ports(self) -> Set[int]:
        with self.lock:
            idle_port_rows = self.conn.execute(f"""
            select port from sessions
            where state = "{IDLE_STATE}"
            """)
            return {port_row[0] for port_row in idle_port_rows}

    def free_ports(self) -> Set[int]:
        return self.usable_ports - self.used_ports()

    def has_idle_session(self) -> bool:
        return len(self.idle_ports()) > 0

    def pop_idle_session(self) -> Optional[Session]:
        with self.lock:
            idle_rows = self.conn.execute(f"""
            select port from sessions
            where state = "{IDLE_STATE}"
            limit 1
            """).fetchall()
            if len(idle_rows) < 1:
                logger.error("An idle session was requested but not found in the database!")
                return None
            port = int(idle_rows[0][0])
            result = self.current_sessions[port]
            self.conn.execute(f"""
            update sessions
            set state = "{ALLOCATED_STATE}",
                use_before = {time.time() + 10.}
            where port = {port}
            """)
            self.conn.commit()
            self.create_idle_session()
            return result

    def create_idle_session(self):
        with self.lock:
            free_ports = self.free_ports()
            if not free_ports:
                logger.warn("Not starting required idle session, because free ports are exhausted.")
                return
            new_session = Session(
                secret=secrets.token_urlsafe(16),
                port=free_ports.pop(),
                process=None,
                sio_url="")
            # enter into current sessions
            new_session.sio_url = f"{self.hostname}:{new_session.port}"
            self.current_sessions[new_session.port] = new_session
            self.conn.execute(f"""
            insert or replace into sessions (port, state, url, heartbeat, secret, since, use_before)
            values (
                {new_session.port},
                "{IDLE_STATE}",
                "{new_session.sio_url}",
                {time.time()},
                "{new_session.secret}",
                {time.time()},
                {0})
            """)
            # commit before starting the new process, such that the entry is definitely seen
            self.conn.commit()
            # start raveboard on the selected port
            new_session.process = psutil.Popen([
                arg.replace(
                    "{port}", str(new_session.port)
                ).replace(
                    "{python}", py_interpreter
                ).replace(
                    "{session_db}", self.db_path)
                for arg in self.session_launch_args
            ])
            self.created_sessions += 1

    def is_authorized(self, url, secret_token):
        with self.lock:
            token_rows = self.conn.execute(f"""
            select secret from sessions where url = "{url}"
            """).fetchall()
            if not token_rows:
                logger.warn(f"Session not authorized: {url}, {secret_token}")
                return False
            return token_rows[0][0] == secret_token

    def update(self):
        while True:
            with self.lock:
                # the grim reaper has arrived
                dead_sessions = self.conn.execute(f"""
                select port from sessions where
                    state = "{KILLME_STATE}" or
                    heartbeat < {time.time() - self.zombie_heartbeat_threshold} or
                    (state = "{ALLOCATED_STATE}" and {time.time()} > use_before)
                """).fetchall()
                dead_ports = set()
                for port_row in dead_sessions:
                    port = int(port_row[0])
                    dead_ports.add(port)
                    if port not in self.current_sessions:
                        logger.error("A port was marked as `KILLME`, but is not recorded in `current_sessions`.")
                        continue
                    sess = self.current_sessions[port]
                    del self.current_sessions[port]
                    for proc in (*sess.process.children(recursive=True), sess.process):
                        print(f"Killing {proc.pid} for port {port}.")
                        proc.send_signal(signal.SIGKILL)
                    sess.process.wait()
                if dead_ports:
                    self.conn.execute(f"""
                    delete from sessions where {" or ".join(f"port = {port}" for port in dead_ports)}
                    """)
                num_sessions_to_start = self.num_idle_instances - len(self.idle_ports())
                while num_sessions_to_start > 0:
                    self.create_idle_session()
                    num_sessions_to_start -= 1
                self.conn.commit()
            time.sleep(self.refresh_iv_secs)

    def sessions(self) -> List[Tuple[int, int, str]]:
        with self.lock:
            return self.conn.execute(f"""
            select port, {time.time()} - since as uptime, state from sessions
            """).fetchall()


# For session instance processes, encapsulates calls to ...
# * Update session state to `killme`.
# * Update session heartbeat.
# * Authorize session token
class SessionClient:

    def __init__(self, *, db_path, port):
        self.port = int(port)
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.lock = RLock()
        if not self.conn:
            logger.error("Failed to open SQLite DB at {}!".format(self.db_path))
        self.is_dead = False
        self.heartbeat()

    def heartbeat(self):
        with self.lock:
            self.conn.execute(f"""
            update sessions set heartbeat = {time.time()} where port = {self.port}
            """)
            self.conn.commit()

    def killme(self):
        with self.lock:
            self.conn.execute(f"""
            update sessions set state = "{KILLME_STATE}" where port = {self.port}
            """)
            self.conn.commit()
            self.is_dead = True

    def dead(self) -> bool:
        with self.lock:
            return self.is_dead

    def authorized(self, token) -> bool:
        with self.lock:
            secret = self.conn.execute(f"""
            select secret from sessions where port = {self.port} limit 1
            """).fetchall()
            if len(secret) < 1:
                return False
            if secret[0][0] == token:
                self.conn.execute(f"""
                update sessions set state = "{IN_USE_STATE}" where port = {self.port}
                """)
                self.conn.commit()
                return True
            return False
