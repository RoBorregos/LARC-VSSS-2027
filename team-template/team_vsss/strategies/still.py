from ..world import Command, World


class Still:
    """Does nothing. Useful as an opponent to test if your strategy scores."""

    # Why does a "still" robot behave like a wall instead of being pushed?
    def decide(self, world: World) -> list[Command]:
        return [Command(r.id) for r in world.mine]
