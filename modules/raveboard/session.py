import subprocess
import secrets
import sqlite3
from datetime import datetime
from typing import Dict, List, Set


# Encapsulates a single raveboard session
class Session:
    def __init__(self, *, secret, process, port, sio_url):
        self.secret = secret
        self.port = port
        self.process = process
        self.sio_url = sio_url


# Valid value for state column
IDLE_STATE = "idle"
IN_USE_STATE = "in_use"
KILLME_STATE = "killme"


# Manages session sqlite db and associated running sessions
class SessionManager:

    def __init__(self, *, db_path, refresh_iv_ms, session_launch_args, num_idle_instances, usable_ports, hostname):
        self.db_path = db_path
        self.refresh_iv_ms = refresh_iv_ms
        self.session_launch_args = session_launch_args
        self.num_idle_instances = num_idle_instances
        self.usable_ports = usable_ports
        self.conn = sqlite3.connect(db_path)
        self.current_sessions: Dict[int, Session] = {}
        self.hostname = hostname
        assert self.conn
        assert self.num_idle_instances > 0

        # -- prepare db: create table
        self.conn.execute("""
        create table if not exists sessions (
            port real primary key, state text, sio_uri text, last_used text, secret text,
        )
        """)

        # -- prepare db: clean out old entries
        self.conn.execute(f"""
        delete from sessions
        """)

        while self.idle_ports() < num_idle_instances:
            self.create_idle_session()

    def used_ports(self) -> Set[int]:
        used_port_rows = self.conn.execute(f"""
        select port from sessions
        """)
        return {port_row[0] for port_row in used_port_rows}

    def idle_ports(self) -> Set[int]:
        idle_port_rows = self.conn.execute(f"""
        select port from sessions
        where state = "{IDLE_STATE}"
        """)
        return {port_row[0] for port_row in idle_port_rows}

    def free_ports(self) -> Set[int]:
        return self.usable_ports - self.used_ports()

    def has_idle_session(self) -> bool:
        return len(self.idle_ports()) > 0

    def pop_idle_session(self) -> Session:
        idle_rows = self.conn.execute(f"""
        select port from sessions
        where state = "{IDLE_STATE}"
        limit 1
        """).fetchall()
        assert len(idle_rows) > 0
        port = idle_rows[0]["port"]
        result = self.current_sessions[port]
        self.conn.execute(f"""
        update sessions
        set state = "{IN_USE_STATE}"
        where port = {port}
        """)
        self.create_idle_session()
        return result

    def create_idle_session(self):
        free_ports = self.free_ports()
        if not free_ports:
            print("Note: NOT creating a new session, because free ports are exhausted.")
            return
        new_session = Session(
            secret=secrets.token_urlsafe(16),
            port=free_ports.pop(),
            process=None,
            sio_url="")
        # start raveboard on the selected port
        new_session.process = subprocess.Popen([
            str(new_session.port) if arg == "{port}" else arg
            for arg in self.session_launch_args
        ])
        # enter into current sessions
        new_session.sio_url = f"{self.hostname}:{new_session.port}"
        self.current_sessions[new_session.port] = new_session
        self.conn.execute(f"""
        insert or replace into sessions (port, state, sio_uri, last_used, secret)
        values (
            {new_session.port},
            "{IDLE_STATE}",
            "{new_session.sio_url}",
            "{datetime.now().strftime("%Y/%m/%d %H:%M:%S")}",
            "{new_session.secret}")
        """)

    def is_authorized(self, sio_uri, secret_token):
        token_rows = self.conn.execute(f"""
        select secret from sessions where sio_uri = "{sio_uri}"
        """).fetchall()
        if not token_rows:
            print(f"Session not authorized: {sio_uri}, {secret_token}")
            return False
        return token_rows[0]["secret"] == secret_token

    def update(self):
        # Kill dead sessions
        # Launch up to num_idle_instances
        pass

    def set_killme(self):
        pass
