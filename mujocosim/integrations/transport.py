"""DDS deployment defaults shared by applications and notebook clients."""
import os
import platform
import sys

from rsim.transport.descriptor import transport_config


# Large UDP datagrams can be lost even between local WSL processes. Fragment
# samples in DDS instead of depending on IP fragmentation in the virtual NIC.
# Keep discovery automatic: WSL interface names and addresses can change.
WSL_DDS_CONFIG = '''<CycloneDDS><Domain Id="any"><General>
  <MaxMessageSize>1200 B</MaxMessageSize>
  <MaxRexmitMessageSize>1200 B</MaxRexmitMessageSize>
  <FragmentSize>1025 B</FragmentSize>
</General></Domain></CycloneDDS>'''


def configure_transport(transport=None):
    """Resolve an RSim transport and apply WSL defaults before opening DDS.

    Call in each independently launched process, before its first Runtime.
    Worker processes inherit the configuration. An explicit CYCLONEDDS_URI
    (including an empty value) always takes precedence; other backends and
    operating systems keep their defaults.
    """
    config = transport_config(transport)
    if (config.backend == 'cyclonedds' and sys.platform == 'linux'
            and 'microsoft' in platform.release().lower()):
        os.environ.setdefault('CYCLONEDDS_URI', WSL_DDS_CONFIG)
    return config
