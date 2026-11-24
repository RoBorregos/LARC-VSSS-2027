"""Strategy catalog. To add a new one: create its file and register it here."""
# Why must a strategy be imported before it can be added to the dictionary?
from .basic import Basic
from .rrt_striker import RRTStriker
from .still import Still

# The key is the name used in the command line: --blue basic
STRATEGIES = {
    "basic": Basic,
    "still": Still,
    "rrt": RRTStriker,
}
